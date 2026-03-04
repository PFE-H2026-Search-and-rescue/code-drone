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
#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <map>

#include "vio_manager.h"
#include "page_manager.h"
#include "cmd_group_manager.h"
#include "manager_utils.h"

#define QVIO_NAME "qvio"
#define QVIO_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR QVIO_NAME "/"

#define OV_NAME "ov"
#define OV_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR OV_NAME "/"

extern struct mg_mgr mgr_;

static std::vector<websocket_info> ws_info_vec;
std::map<std::string, std::string> structs_map;

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server %d providing: vio connected\n", ch);
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context) {
    printf("Server %d providing: vio disconnected\n", ch);
}

static void VioDataCallback(int ch, char* data, int bytes, void* context) {
    int ws_index = 0;
    for (int i = 0; i < ws_info_vec.size(); i++){
        if (ws_info_vec[i].ch == ch ){
            ws_index = i;
            break;
        }
    }

    if (ws_info_vec[ws_index].connected){
        //printf("\nSTRING FROM MAP: %s\n", structs_map[ws_info_vec[ws_index].topic_name + "\n"].c_str());
        if (strcmp(structs_map[ws_info_vec[ws_index].topic_name + "\n"].c_str(), "ext_vio_data_t") == 0) {
            //mg_ws_send(c, "ext_vio_data_t", 14, WEBSOCKET_OP_TEXT);

            int n_packets = 0;

            ext_vio_data_t* data_array = pipe_validate_ext_vio_data_t(data, bytes, &n_packets);

            // if there was an error OR no packets received, just return;
            if (data_array == NULL) return;
            if (n_packets <= 0) return;

            // Skip if we are falling behind
            if (pipe_client_bytes_in_pipe(ch) > 0)
                return;

            struct mg_connection* c;

            for (unsigned int i = 0; i < ws_info_vec[ws_index].connection_ids.size(); i++){
                for (c = mgr_.conns; c != NULL; c = c->next){
                    if (c->id != ws_info_vec[ws_index].connection_ids[i]) continue;

                    // check if we are backing up the socket
                    // if so, just drop the packet for this connection, but mark sent as true to prevent closure
                    if (c->send.len != 0 || c->recv.len != 0){
                        // fprintf(stderr, "dropping vio packet\n");
                        continue;
                    }

                    mg_ws_send(c, (char*)&data_array[n_packets - 1], sizeof(ext_vio_data_t), WEBSOCKET_OP_BINARY);
                }
            }
        }
        else if (strcmp(structs_map[ws_info_vec[ws_index].topic_name + "\n"].c_str(), "pose_vel_6dof_t") == 0) {
            int n_packets = 0;

            pose_vel_6dof_t* data_array = pipe_validate_pose_vel_6dof_t(data, bytes, &n_packets);

            // if there was an error OR no packets received, just return;
            if (data_array == NULL) return;
            if (n_packets <= 0) return;

            // Skip if we are falling behind
            if (pipe_client_bytes_in_pipe(ch) > 0)
                return;

            struct mg_connection* c;

            for (unsigned int i = 0; i < ws_info_vec[ws_index].connection_ids.size(); i++){
                for (c = mgr_.conns; c != NULL; c = c->next){
                    if (c->id != ws_info_vec[ws_index].connection_ids[i]) continue;

                    // check if we are backing up the socket
                    // if so, just drop the packet for this connection, but mark sent as true to prevent closure
                    if (c->send.len != 0 || c->recv.len != 0){
                        // fprintf(stderr, "dropping vio packet\n");
                        continue;
                    }

                    mg_ws_send(c, (char*)&data_array[n_packets - 1], sizeof(pose_vel_6dof_t), WEBSOCKET_OP_BINARY);
                }
            }
        }
        else if (strcmp(structs_map[ws_info_vec[ws_index].topic_name + "\n"].c_str(), "vio_data_t") == 0) {
            int n_packets = 0;

            vio_data_t* data_array = pipe_validate_vio_data_t(data, bytes, &n_packets);

            // if there was an error OR no packets received, just return;
            if (data_array == NULL) return;
            if (n_packets <= 0) return;

            // Skip if we are falling behind
            if (pipe_client_bytes_in_pipe(ch) > 0)
                return;

            struct mg_connection* c;

            for (unsigned int i = 0; i < ws_info_vec[ws_index].connection_ids.size(); i++){
                for (c = mgr_.conns; c != NULL; c = c->next){
                    if (c->id != ws_info_vec[ws_index].connection_ids[i]) continue;

                    // check if we are backing up the socket
                    // if so, just drop the packet for this connection, but mark sent as true to prevent closure
                    if (c->send.len != 0 || c->recv.len != 0){
                        // fprintf(stderr, "dropping vio packet\n");
                        continue;
                    }

                    mg_ws_send(c, (char*)&data_array[n_packets - 1], sizeof(vio_data_t), WEBSOCKET_OP_BINARY);
                }
            }
        }

    }

    return;
}

void VioManagerCallback(struct mg_connection* c, int ev, void* ev_data, void* fn_data) {

    struct mg_http_message* hm = (struct mg_http_message*)ev_data;

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
        }
        return;
    }

    // Chop off the /ptc/ of the uri to get the ptcloud name
    const char *tmp = &(hm->uri.ptr[strlen(VioManagerClaimString) - 1]);
    // And remove all of the https garbage, we just want the pipe name
    int len = std::min(strchr(tmp, ' ') - tmp, strchr(tmp, '/') - tmp);
    char in_name[64];
    // Copy name in and terminate the string
    strncpy(in_name, tmp, len);
    in_name[len] = 0;

    int j = 0;
    for (int i = 0; in_name[i] != 0; i++) {
        if (valid_char(in_name[i])) in_name[j++] = in_name[i];
    }
    in_name[j] = 0;

    // make sure we dont double sub to a topic
    for (unsigned int i = 0; i < ws_info_vec.size(); i++){
        if ( strcmp(ws_info_vec[i].topic_name.c_str(), in_name) == 0 ){
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
    pipe_client_set_simple_helper_cb(ch, VioDataCallback, NULL);
    pipe_client_open(ch, (std::string(MODAL_PIPE_DEFAULT_BASE_DIR) + std::string(in_name) + "/").c_str(), PROCESS_NAME, EN_PIPE_CLIENT_SIMPLE_HELPER, EXT_VIO_RECOMMENDED_READ_BUF_SIZE);
    printf("opening pipe ch %d %s\n", ch, in_name);
}


void ExtractFeaturesCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        /*
        * Get instances of ext_vio_data_t
        */
        
        std::string adb_command = "voxl-list-pipes -t ext_vio_data_t";

        FILE* pipe = popen(adb_command.c_str(), "r"); 
        if (!pipe) {
            std::cerr << "Error executing command." << std::endl;
        }

        char buffer[64];
        std::string result = "";
        //memcpy(buffer, "0", 64);

        // Read the command output line by line
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
            structs_map[result] = "ext_vio_data_t";
            //printf("RETRIEVED INFO: %s", result.c_str());
            mg_ws_send(c, result.c_str(), strlen(result.c_str()), WEBSOCKET_OP_TEXT);
            result = "";
        }

        pclose(pipe); // Close the pipe

        /*
        * Get instances of pose_vel_6dof_t
        */

        adb_command = "voxl-list-pipes -t pose_vel_6dof_t";

        pipe = popen(adb_command.c_str(), "r"); // Open a pipe to execute the command
        if (!pipe) {
            std::cerr << "Error executing command." << std::endl;
        }

        result = "";

        // Read the command output line by line
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
            structs_map[result] = "pose_vel_6dof_t";
            //printf("RETRIEVED INFO: %s", result.c_str());
            mg_ws_send(c, result.c_str(), strlen(result.c_str()), WEBSOCKET_OP_TEXT);
            result = "";
        }

        pclose(pipe); // Close the pipe

        /* 
        * Get instances of vio_data_t
        */
        
        adb_command = "voxl-list-pipes -t vio_data_t";

        pipe = popen(adb_command.c_str(), "r"); // Open a pipe to execute the command
        if (!pipe) {
            std::cerr << "Error executing command." << std::endl;
        }

        result = "";

        // Read the command output line by line
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
            structs_map[result] = "vio_data_t";
            //printf("RETRIEVED INFO: %s", result.c_str());
            mg_ws_send(c, result.c_str(), strlen(result.c_str()), WEBSOCKET_OP_TEXT);
            result = "";
        }

        pclose(pipe); // Close the pipe
    }
}

void ResetQVIOCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    char msg[128] = {0};
    memcpy(msg, "reset_vio_hard", 14);
    if (ev == MG_EV_HTTP_MSG) {
        writeToControlPipe(QVIO_LOCATION "/control", msg, strlen(msg));
    }
}

void ResetOVCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    char msg[128] = {0};
    memcpy(msg, "reset_vio_hard", 14);
    if (ev == MG_EV_HTTP_MSG) {
        writeToControlPipe(OV_LOCATION "/control", msg, strlen(msg));
    }
}
