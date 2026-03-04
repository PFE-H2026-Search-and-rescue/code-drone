#include "benchmark_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>
#include <string.h>
#include <dirent.h>

#include "page_manager.h"
#include "manager_utils.h"
#include "pipe_channels.h"

static const char *s_root_dir = "/data/benchmark-results";

extern struct mg_mgr mgr_;

void BenchmarkManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;

    if (ev == MG_EV_HTTP_MSG) 
    {
        if (mg_http_match_uri(hm, "/benchmark/load_dirs")) 
        {
            printf("uri: /benchmark/load_dirs\n");
            struct dirent *entry;
            DIR *dp = opendir(s_root_dir);
            
            if (dp == NULL) {
                mg_http_reply(c, 500, "Content-Type: text/plain\r\n", "Directory not found\n");
                return;
            }

            char json_response[4096] = "[";
            int first = 1;

            while ((entry = readdir(dp)) != NULL) {
                if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                    continue;
                }

                // Check if entry is a directory
                if (entry->d_type != DT_DIR) {
                    continue;
                }

                if (!first) {
                    strcat(json_response, ", ");
                }
                char file_entry[256];
                snprintf(file_entry, sizeof(file_entry), "\"%s\"", entry->d_name);
                strcat(json_response, file_entry);
                first = 0;
            }

            closedir(dp);
            
            strcat(json_response, "]");

            mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json_response);
            return;
        } 

        if (mg_http_match_uri(hm, "/benchmark/file/#"))
        {
            char file_path[512];
            snprintf(file_path, sizeof(file_path), "%s%.*s", s_root_dir, 
                     (int)(hm->uri.len - 10 - 5), hm->uri.ptr + 10 + 5);
            printf("file_path: %s\n", file_path);

            FILE *fp = fopen(file_path, "r");
            if (fp) {
                fseek(fp, 0, SEEK_END);
                long file_size = ftell(fp);
                rewind(fp);

                char *content = malloc(file_size + 1);
                if (content) {
                    fread(content, 1, file_size, fp);
                    content[file_size] = '\0';

                    mg_http_reply(c, 200, "Content-Type: text/csv\r\n", "%s", content);
                    free(content);
                } else {
                    mg_http_reply(c, 500, "Content-Type: text/plain\r\n", "Memory allocation failed\n");
                }
                fclose(fp);
            } else {
                mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "File not found\n");
            }
            return;
        }
    }    
    return;
}
