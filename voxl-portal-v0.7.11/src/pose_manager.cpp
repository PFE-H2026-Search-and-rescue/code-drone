/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include "pose_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>

#include "page_manager.h"
#include "manager_utils.h"

#define POSE_NAME "vvhub_body_wrt_fixed"

extern struct mg_mgr mgr_;

// since the pose manager will only be subscribing to a single topic, we only need one websocket_info struct
static websocket_info ws_info;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing pose connected\n");
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing pose disconnected\n");
}

static void PoseDataCallback(int ch, char *data, int bytes, void *context)
{
    if (!ws_info.connected)
        return;

    struct mg_connection *c;

    // if there was an error OR no packets received, just return;
    int n_packets = 0;
    pose_vel_6dof_t *data_array = pipe_validate_pose_vel_6dof_t(data, bytes, &n_packets);
    if (data_array == NULL)
        return;
    if (n_packets <= 0)
        return;

    for (size_t i = 0; i < ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info.connection_ids[i])
                continue;

            // check if we are backing up the socket (arbitrary choice of 1Kb since pose is small)
            // if so, just drop the packet for this connection
            if (c->send.len >= 1024)
            {
                // fprintf(stderr, "dropping pose packet %lu\n", c->send.len);
                continue;
            }

            mg_ws_send(c, (char *)&data_array[n_packets - 1], sizeof(pose_vel_6dof_t), WEBSOCKET_OP_BINARY);
            // prev_msg_size = bytes;
        }
    }
}

void PoseManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
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
        ws_info.connected = true
        ;
        int ch = pipe_client_get_next_available_channel();

        pipe_client_set_connect_cb(ch, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(ch, _disconnect_cb, NULL);
        pipe_client_set_simple_helper_cb(ch, PoseDataCallback, NULL);
        pipe_client_open(ch, POSE_NAME, PROCESS_NAME, EN_PIPE_CLIENT_SIMPLE_HELPER, sizeof(pose_vel_6dof_t) * 500);

        ws_info.ch = ch;
    }
    
    ws_info.connection_ids.push_back(c->id);
}
