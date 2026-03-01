#include "mesh_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>
#include <limits>

#include "page_manager.h"
#include "manager_utils.h"

#define MESH_NAME "voxl_mesh"
#define MESH_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR MESH_NAME "/"

#define MESH_ENDPOINT_NAME "voxl_mesh_endpoint"
#define MESH_ENDPOINT_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR MESH_ENDPOINT_NAME "/"

extern struct mg_mgr mgr_;

// since the costmap manager will only be subscribing to a single topic, we only need one websocket_info struct
static websocket_info ws_info;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: mesh connected\n");
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: mesh disconnected\n");
}

static void MeshDataCallback(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context)
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

            // check if we are backing up the socket
            // if so, just drop the packet for this connection
            if (c->send.len > 2 * bytes)
            {
                // fprintf(stderr, "dropping mesh packet\n");
                continue;
            }

            mg_ws_send(c, data, bytes, WEBSOCKET_OP_BINARY);
        }
    }
}

void MeshManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
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
        pipe_client_set_simple_helper_cb(ch, MeshDataCallback, c);
        int ret = pipe_client_open(ch, MESH_LOCATION, PROCESS_NAME,
                                    EN_PIPE_CLIENT_SIMPLE_HELPER, (64 * 1024 * 1024));

        ws_info.ch = ch;
    }

    ws_info.connection_ids.push_back(c->id);
}

static void MeshEndpointCallback(__attribute__((unused)) int ch, char *data, int bytes, void *context)
{
    struct mg_connection *c = (mg_connection*)context;

    mg_http_reply(c, 200, "", "%s\n", data);

    // self closure, lil sketchy
    pipe_client_close(ch);
}

void MeshApiCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data){
    // basically we do not need to do any safety checks or anything here
    // just need to sub to our mapper_saved_mesh_pipe
    // send a control command to it with PLY format (only for now)
    // then have a listener cb that handles proper closure and http writing
    int ch = pipe_client_get_next_available_channel();

    pipe_client_set_connect_cb(ch, _connect_cb, NULL);
    pipe_client_set_disconnect_cb(ch, _disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(ch, MeshEndpointCallback, c);
    int ret = pipe_client_open(ch, MESH_ENDPOINT_LOCATION, PROCESS_NAME,
                                EN_PIPE_CLIENT_SIMPLE_HELPER | CLIENT_FLAG_DISABLE_AUTO_RECONNECT, (64 * 1024 * 1024));

    struct mg_http_message *hm = (struct mg_http_message *)ev_data;

    if (mg_http_match_uri(hm, MeshApiPlyString)){
        char command[] = "ply";
        writeToControlPipe(MESH_ENDPOINT_LOCATION "/control", command, strlen(command));
    }
    else if (mg_http_match_uri(hm, MeshApiObjString)){
        char command[] = "obj";
        writeToControlPipe(MESH_ENDPOINT_LOCATION "/control", command, strlen(command));
    }
    else if (mg_http_match_uri(hm, MeshApiGltfString)){
        char command[] = "gltf";
        writeToControlPipe(MESH_ENDPOINT_LOCATION "/control", command, strlen(command));
    }
    else if (mg_http_match_uri(hm, MeshApiEsdfString)){
        char command[] = "esdf";
        writeToControlPipe(MESH_ENDPOINT_LOCATION "/control", command, strlen(command));
    }
    else {
        fprintf(stderr, "ERROR: Recieved unknown mesh format\n");
    }
    return;
}

void sendMapperCmd(char *data, int bytes)
{
    writeToControlPipe(MESH_LOCATION "/control", data, bytes);
}