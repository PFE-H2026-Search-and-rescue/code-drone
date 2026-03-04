#include "costmap_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>
#include "page_manager.h"
#include "manager_utils.h"

#define COSTMAP_NAME "voxl_mapper_costmap"
#define COSTMAP_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR COSTMAP_NAME "/")

extern struct mg_mgr mgr_;

// since the costmap manager will only be subscribing to a single topic, we only need one websocket_info struct
static websocket_info ws_info;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: costmap connected\n");
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: costmap disconnected\n");
}

static void CostmapDataCallback(int ch, point_cloud_metadata_t meta, void *data, void *context)
{
    static char *costmap_data = nullptr;

    if (!ws_info.connected)
        return;

    struct mg_connection *c;

    int costmap_meta_size = sizeof(point_cloud_metadata_t);
    int costmap_body_size = pipe_point_cloud_meta_to_size_bytes(meta);
    int costmap_total_size = costmap_meta_size + costmap_body_size;

    costmap_data = (char *)malloc(costmap_total_size);
    memcpy(costmap_data, &meta, costmap_meta_size);
    memcpy(costmap_data + costmap_meta_size, data, costmap_body_size);

    for (size_t i = 0; i < ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info.connection_ids[i])
                continue;

            if (c->send.len > 2 * costmap_total_size)
            {
                // fprintf(stderr, "dropping costmap packet\n");
                continue;
            }

            mg_ws_send(c, costmap_data, costmap_total_size, WEBSOCKET_OP_BINARY);
        }
    }

    free(costmap_data);
    costmap_data = nullptr;
}

void CostmapManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;

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
        pipe_client_set_point_cloud_helper_cb(ch, CostmapDataCallback, c);
        int ret = pipe_client_open(ch, COSTMAP_LOCATION, PROCESS_NAME,
                                   CLIENT_FLAG_EN_POINT_CLOUD_HELPER, (64 * 1024 * 1024));

        ws_info.ch = ch;
    }

    ws_info.connection_ids.push_back(c->id);
}
