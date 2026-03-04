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
#include <regex>

#include "log_manager.h"
#include "page_manager.h"
#include "cmd_group_manager.h"
#include "manager_utils.h"


extern struct mg_mgr mgr_;

void LogManagerCallback(struct mg_connection* c, int ev, void* ev_data, void* fn_data) {

    struct mg_http_message* hm = (struct mg_http_message*)ev_data;

    // Chop off the /log/ of the uri to get the log name?
    const char *tmp = &(hm->uri.ptr[strlen(LogManagerClaimString) - 1]);
    // And remove all of the https garbage, we just want the pipe name ----- looking for the first space i think since we provide extra slashes
    int len = strchr(tmp, ' ') - tmp;
    char in_name[64];
    // Copy name in and terminate the string
    strncpy(in_name, tmp, len);
    in_name[len] = 0;

    int j = 0;
    for (int i = 0; in_name[i] != 0; i++) {
        if (valid_char(in_name[i])) in_name[j++] = in_name[i];
    }
    in_name[j] = 0;

    // replacing %2F with /
    // necessary for easy handling of paths in http requests
    std::string full_path = in_name;
    full_path = std::regex_replace(full_path, std::regex("%2F"), "/");
    
    printf("starting log at: %s\n", full_path.c_str());
    // now just need to start up voxl-replay with -p this path (child process)
    pid_t child_pid = fork();
    if (child_pid < 0){
        perror("fork");
    }
    else if (child_pid == 0){
        int ret = execl("/usr/bin/voxl-replay", "voxl-replay", "-p", full_path.c_str(), "-y", NULL);
        return;
    }

    mg_http_reply(c, 200, "", "%s", "good");
}
