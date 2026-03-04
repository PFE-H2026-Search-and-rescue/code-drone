#include "mapper_ptcloud_manager.h"

#include <modal_pipe.h>
#include <voxl_cutils.h>

#include <algorithm>

#include "page_manager.h"
#include "manager_utils.h"

// mesh specific
#define ALIGNED_PTCLOUD_NAME "voxl_mapper_aligned_ptcloud"
#define ALIGNED_PTCLOUD_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR ALIGNED_PTCLOUD_NAME "/")

extern struct mg_mgr mgr_;

// since the costmap manager will only be subscribing to a single topic, we only need one websocket_info struct
static websocket_info ws_info;

static bool ws_connected = false;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: aligned ptcloud connected\n");
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: aligned ptcloud disconnected\n");
}

static void PtcloudDataCallback(int ch, point_cloud_metadata_t meta, void *data, void *context)
{
    static char *aligned_data = nullptr;

    if (!ws_info.connected)
        return;

    struct mg_connection *c;

    int aligned_meta_size = sizeof(point_cloud_metadata_t);
    int aligned_body_size = pipe_point_cloud_meta_to_size_bytes(meta);
    int aligned_total_size = aligned_meta_size + aligned_body_size;

    aligned_data = (char *)malloc(aligned_total_size);
    memcpy(aligned_data, &meta, aligned_meta_size);
    memcpy(aligned_data + aligned_meta_size, data, aligned_body_size);

    for (unsigned int i = 0; i < ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info.connection_ids[i])
                continue;
            // check if we are backing up the socket
            // if so, just drop the packet for this connection
            if (c->send.len > 2 * aligned_total_size)
            {
                // fprintf(stderr, "dropping aligned ptcloud packet\n");
                continue;
            }

            mg_ws_send(c, aligned_data, aligned_total_size, WEBSOCKET_OP_BINARY);
        }
    }
    free(aligned_data);
    aligned_data = nullptr;
}

void PtcloudManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{

    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());

        // Close the pipe if no longer sending to any websockets
        if (ws_info.connection_ids.empty())
        {
            ws_info.connected = false;
            pipe_client_close(ws_info.ch);
        }
        return;
    }

    if (!ws_info.connected)
    {
        ws_info.connected = true;

        int ch = pipe_client_get_next_available_channel();

        pipe_client_set_connect_cb(ch, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(ch, _disconnect_cb, NULL);
        pipe_client_set_point_cloud_helper_cb(ch, PtcloudDataCallback, c);
        int ret = pipe_client_open(ch, ALIGNED_PTCLOUD_LOCATION, PROCESS_NAME,
                                   CLIENT_FLAG_EN_POINT_CLOUD_HELPER, (64 * 1024 * 1024));

        ws_info.ch = ch;
    }

    ws_info.connection_ids.push_back(c->id);
}
