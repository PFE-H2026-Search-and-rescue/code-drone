#include "cpu_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>
#ifndef __ANDROID__
#include <cpu_monitor_interface.h>
#endif // __ANDROID__
#include <unistd.h>

#include <algorithm>

#include "page_manager.h"
#include "manager_utils.h"
#include "pipe_channels.h"

#define CPU_NAME "cpu_monitor"
#define CPU_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR CPU_NAME "/")

extern struct mg_mgr mgr_;

// since the costmap manager will only be subscribing to a single topic, we only need one websocket_info struct
static websocket_info ws_info;

// called whenever we connect or reconnect to the server
static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: cpu connected\n");
}
// called whenever we disconnect from the server
static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: cpu disconnected\n");
}

static void CpuDataCallback(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context)
{
#ifndef __ANDROID__
    if (!ws_info.connected)
        return;

    int n_packets;
    cpu_stats_t *data_array = modal_cpu_validate_pipe_data(data, bytes, &n_packets);

    if (data_array == NULL)
        return;
    if (n_packets <= 0)
        return;

    struct mg_connection *c;

    for (size_t i = 0; i < ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info.connection_ids[i])
                continue;

            // check if we are backing up the socket
            // if so, just drop the packet for this connection
            if (c->send.len > 1024)
            {
                // fprintf(stderr, "dropping cpu packet\n");
                continue;
            }

            mg_ws_send(c, (char *)&data_array[n_packets - 1], sizeof(cpu_stats_t), WEBSOCKET_OP_BINARY);
        }
    }
#endif //__ANDROID__
}

void CpuManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
#ifndef __ANDROID__

    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());

        // Close the pipe if no longer sending to any websockets
        if (ws_info.connection_ids.empty())
        {
            ws_info.connected = false;
            pipe_client_close(CPU_PIPE_CH);
        }
        return;
    }

    if (!ws_info.connected)
    {
        ws_info.connected = true;

        pipe_client_set_connect_cb(CPU_PIPE_CH, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(CPU_PIPE_CH, _disconnect_cb, NULL);
        pipe_client_set_simple_helper_cb(CPU_PIPE_CH, CpuDataCallback, c);
        int ret = pipe_client_open(CPU_PIPE_CH, CPU_LOCATION, PROCESS_NAME,
                                   EN_PIPE_CLIENT_SIMPLE_HELPER, (sizeof(cpu_stats_t) * 21));

    }

    ws_info.connection_ids.push_back(c->id);
#endif // __ANDROID__
}
