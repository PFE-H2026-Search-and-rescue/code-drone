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
#ifndef PAGE_MANAGER_H
#define PAGE_MANAGER_H

#define PROCESS_NAME "voxl-portal"
#define FLAG_EXPERIMENTAL_REST_API 0x01

#include <unordered_map>
#include <vector>
#include <string>

extern "C"
{
#include <mongoose.h>
}

class PageManager
{
    public:
        static PageManager* GetInstance()
        {
            static PageManager pm;
            return &pm;
        }

        void Run(char* web_root, int flags);
        static void ServerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

        int flags = 0;

    private:
        PageManager(){}

        int RegisterPages();
        

        int num_pages_    = 0;
        int num_managers_ = 0;

     
        static const int    MAX_SUB_MANAGERS = 16;
        mg_event_handler_t  sub_managers_          [MAX_SUB_MANAGERS];
        char                sub_manager_tags_      [MAX_SUB_MANAGERS][256];
        void*               sub_manager_cb_data_   [MAX_SUB_MANAGERS];

        static const int    MAX_NUM_PAGES = 64;
        mg_event_handler_t  handlers_[MAX_NUM_PAGES];
        char                handler_uris_[MAX_NUM_PAGES][256];
        void*               handler_cb_data_[MAX_NUM_PAGES];

        std::unordered_map<std::string, mg_event_handler_t> callbacks_;
        std::vector<std::string> claim_strings;
};


#endif // PAGE_MANAGER_H