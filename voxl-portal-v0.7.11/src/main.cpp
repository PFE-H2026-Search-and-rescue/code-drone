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

#include <getopt.h>
#include <string.h>
#include <modal_start_stop.h>
#include "page_manager.h"
#include "api/portal_api.h"

#define WEBROOT_PATH_LEN 256
static int    _flags = 0;
static char   _webroot_path[WEBROOT_PATH_LEN];

static void _print_usage(void)
{
    printf("Usage: voxl-portal <options>\n");
    printf("Options:\n");
    printf("-w <path>         Override webroot path (default /data/web_root\n");
    printf("-e                Enable experimental stuff\n");
    printf("-h                Show help.\n");
}

static int _parse_opts(int argc, char* argv[])
{
    static struct option long_options[] =
    {
        {"experimental",          no_argument,       0, 'e'},
        {"webroot",               required_argument, 0, 'w'},
        {"help",                  no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int option;

    while ((option = getopt_long (argc, argv, "ew:h", &long_options[0], &option_index)) != -1) {
        switch(option){

        case 'e': 
            _flags |= FLAG_EXPERIMENTAL_REST_API;
            break;

        case 'w':
            printf("using alternative root: %s\n", optarg);
            strncpy(_webroot_path, optarg, WEBROOT_PATH_LEN);
            break;

        case 'h':
            _print_usage();
            return -1;

        default:
            _print_usage();
            return -1;
        }
    }

    // enable API for certain platforms for now
    if(portal_api_v1_suported()){
        _flags |= FLAG_EXPERIMENTAL_REST_API;
    }

    // optind is for the extra arguments which are not parsed
    if(optind < argc){
        printf("extra arguments:\n");
        for (; optind < argc; optind++) {
            printf("\t%s\n", argv[optind]);
        }
    }


    return 0;
}


int main(int argc, char** argv) {
    strncpy(_webroot_path, "/data/web_root", WEBROOT_PATH_LEN);

    // parse opts first
    if(_parse_opts(argc, argv)) return -1;

    ////////////////////////////////////////////////////////////////////////////////
    // gracefully handle an existing instance of the process and associated PID file
    //////////////////////////////////////////////////////
    //////////////////////////

    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(kill_existing_process(PROCESS_NAME, 2.0)<-2) return -1;

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    make_pid_file(PROCESS_NAME);

	PageManager::GetInstance()->Run(_webroot_path, _flags);
	return 0;
}
