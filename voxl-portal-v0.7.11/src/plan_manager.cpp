#include "plan_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>

#include "page_manager.h"
#include "manager_utils.h"

// this one is just for sending control cmds back to voxl-mapper
#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/"

// this is for rendering paths, debug points, etc
#define RENDER_NAME "voxl_planner_render"
#define RENDER_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR RENDER_NAME "/")

extern struct mg_mgr mgr_;

// this manager will subscribe to two channels, but only communicates with one
static websocket_info ws_info;
static int plan_cmd_ch;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: plan connected\n");
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: plan disconnected\n");
}

static void PlanDataCallback(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context)
{
    if (!ws_info.connected)
        return;

    struct mg_connection *c;

    for (unsigned int i = 0; i < ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info.connection_ids[i])
                continue;

            mg_ws_send(c, data, bytes, WEBSOCKET_OP_BINARY);
        }
    }
}

void PlanManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
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
        plan_cmd_ch = pipe_client_get_next_available_channel();
        pipe_client_set_connect_cb(plan_cmd_ch, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(plan_cmd_ch, _disconnect_cb, NULL);
        if (int ret = pipe_client_open(plan_cmd_ch, PLAN_LOCATION, PROCESS_NAME, 0, 1024 * 4))
        {
            pipe_print_error(ret);
            fprintf(stderr, "\n\nFailed to open pipe: %s\n\n\n", PLAN_LOCATION);
        }

        int ch = pipe_client_get_next_available_channel();
        pipe_client_set_connect_cb(ch, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(ch, _disconnect_cb, NULL);
        pipe_client_set_simple_helper_cb(ch, PlanDataCallback, NULL);
        if (int ret = pipe_client_open(ch, RENDER_LOCATION, PROCESS_NAME,
                                       EN_PIPE_CLIENT_SIMPLE_HELPER, 1024 * 1024 * 64))
        {
            pipe_print_error(ret);
            fprintf(stderr, "\n\nFailed to open pipe: %s\n\n\n", RENDER_LOCATION);
        }

        ws_info.ch = ch;
    }

    ws_info.connection_ids.push_back(c->id);
}

void sendPlanCmd(char *data, int bytes)
{
    writeToControlPipe(PLAN_LOCATION "/control", data, bytes);
}
