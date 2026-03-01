#include "pointcloud_group_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>

#include "page_manager.h"
#include "cmd_group_manager.h"
#include "manager_utils.h"

extern struct mg_mgr mgr_;

static std::vector<websocket_info> ws_info_vec;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server %d providing: ptcloud connected\n", ch);
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: ptcloud disconnected\n");
}

static void TofDataCallback(int ch, char *data, int bytes, void *context)
{
    int ws_index = 0;

    for (int i = 0; i < ws_info_vec.size(); i++)
    {
        if (ws_info_vec[i].ch == ch)
        {
            ws_index = i;
            break;
        }
    }

    if (ws_info_vec[ws_index].connected)
    {
        // validate data
        int n_packets;
        tof2_data_t *data_array = pipe_validate_tof2_data_t(data, bytes, &n_packets);
        if (data_array == NULL)
            return;

        if (pipe_client_bytes_in_pipe(ch) > 0)
        {
            return;
        }

        struct mg_connection *c;

        for (unsigned int i = 0; i < ws_info_vec[ws_index].connection_ids.size(); i++)
        {
            for (c = mgr_.conns; c != NULL; c = c->next)
            {
                if (c->id != ws_info_vec[ws_index].connection_ids[i])
                    continue;
                // check if we are backing up the socket
                // if so, just drop the packet for this connection
                if (c->send.len > 2 * bytes)
                {
                    // fprintf(stderr, "dropping pointcloud\n");
                    continue;
                }
                mg_ws_send(c, (char *)&data_array[n_packets - 1], sizeof(tof2_data_t), WEBSOCKET_OP_BINARY);
            }
        }
    }
}

static void PtcloudDataCallback(int ch, point_cloud_metadata_t meta, void *data, void *context)
{
    int ws_index = 0;

    for (int i = 0; i < ws_info_vec.size(); i++)
    {
        if (ws_info_vec[i].ch == ch)
        {
            ws_index = i;
            break;
        }
    }

    static char *ptc_data = nullptr;

    if (!ws_info_vec[ws_index].connected)
        return;

    int ptc_meta_size = sizeof(point_cloud_metadata_t);
    int ptc_body_size = pipe_point_cloud_meta_to_size_bytes(meta);
    int ptc_total_size = ptc_meta_size + ptc_body_size;

    // validate data
    if (meta.magic_number != POINT_CLOUD_MAGIC_NUMBER)
        return;
    else if (ptc_body_size <= 0)
        return;

    // Skip if we are falling behind
    if (pipe_client_bytes_in_pipe(ch) > 0)
        return;

    ptc_data = (char *)malloc(ptc_total_size);
    memcpy(ptc_data, &meta, ptc_meta_size);
    memcpy(ptc_data + ptc_meta_size, data, ptc_body_size);

    struct mg_connection *c;

    for (size_t i = 0; i < ws_info_vec[ws_index].connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info_vec[ws_index].connection_ids[i])
                continue;

            // check if we are backing up the socket
            // if so, just drop the packet for this connection, but mark sent as true to prevent closure
            if (c->send.len != 0 || c->recv.len != 0)
            {
                // fprintf(stderr, "dropping vio packet\n");
                continue;
            }
            mg_ws_send(c, (char *)ptc_data, ptc_total_size, WEBSOCKET_OP_BINARY);
        }
    }

    free(ptc_data);
    ptc_data = nullptr;
}

void PointcloudGroupManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
    static int ch = -1;

    if (c->is_closing)
    {
        for (size_t i = 0; i < ws_info_vec.size(); i++)
        {
            websocket_info &cur_info = ws_info_vec[i];

            // Remove this connection from our connection ids
            cur_info.connection_ids.erase(std::remove(cur_info.connection_ids.begin(), cur_info.connection_ids.end(), c->id), cur_info.connection_ids.end());

            // Close the pipe and remove the ws_info, if no longer sending to any websockets
            if (cur_info.connection_ids.empty())
            {
                pipe_client_close(cur_info.ch);
                ws_info_vec.erase(ws_info_vec.begin() + i);
            }
            return;
        }

        return;
    }

    // Chop off the /ptc/ of the uri to get the ptcloud name
    const char *tmp = &(hm->uri.ptr[strlen(PointcloudGroupManagerClaimString) - 1]);

    // And remove all of the https garbage, we just want the pipe name
    int len = std::min(strchr(tmp, ' ') - tmp, strchr(tmp, '/') - tmp);
    char in_name[64];

    // Copy name in and terminate the string
    strncpy(in_name, tmp, len);
    in_name[len] = 0;

    int j = 0;
    for (int i = 0; in_name[i] != 0; i++)
    {
        if (valid_char(in_name[i]))
            in_name[j++] = in_name[i];
    }
    in_name[j] = 0;

    // make sure we dont double sub to a topic
    for (size_t i = 0; i < ws_info_vec.size(); i++)
    {
        if (strcmp(ws_info_vec[i].topic_name.c_str(), in_name) == 0)
        {
            ws_info_vec[i].connection_ids.push_back(c->id);
            return;
        }
    }

    ch = pipe_client_get_next_available_channel();

    websocket_info curr_conn;
    curr_conn.ch = ch;
    curr_conn.connection_ids.push_back(c->id);
    curr_conn.connected = true;
    curr_conn.topic_name = in_name;
    ws_info_vec.push_back(curr_conn);

    pipe_client_set_connect_cb(ch, _connect_cb, NULL);
    pipe_client_set_disconnect_cb(ch, _disconnect_cb, NULL);
    // now we have the name of our pointcloud, check if it's *special* tof
    if(pipe_is_type(in_name, "tof2_data_t"))
    {
        fprintf(stderr, "tof selected\n");
        pipe_client_set_simple_helper_cb(ch, TofDataCallback, c);
        pipe_client_open(ch, in_name, PROCESS_NAME,
                         CLIENT_FLAG_EN_SIMPLE_HELPER, TOF2_RECOMMENDED_READ_BUF_SIZE);
    }
    else
    {
        pipe_client_set_point_cloud_helper_cb(ch, PtcloudDataCallback, c);
        pipe_client_open(ch, in_name, PROCESS_NAME,
                         CLIENT_FLAG_EN_POINT_CLOUD_HELPER, (64 * 1024 * 1024));
        fprintf(stderr, "opening channel %d for %s\n", ch, in_name);
    }
}
