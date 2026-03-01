/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
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

#include "api/portal_api.h"
#include <cstdlib>
#include <string>
#define PORTAL_API_FACTORY_MODE_FILE                 "/data/modalai/sku.txt"
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION     "/api/v1/platform/check-calibration"
#define PORTAL_API_V1_PLATFORM_STATUS                "/api/v1/platform/status.json"
#define PORTAL_API_V1_PLATFORM_HEALTH_CHECK          "/api/v1/platform/health-check"
#define PORTAL_API_V1_PLATFORM_INSPECT_SERVICES      "/api/v1/platform/inspect-services"
#define PORTAL_API_V1_VOXL_PX4_STATUS                "/api/v1/voxl-px4/status.json"
#define PORTAL_API_V1_VOXL_PX4_COMMAND               "/api/v1/voxl-px4/command" //?cmd=XX&arg=YY
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA1     "/api/v1/platform/check-calibration-1"
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA2     "/api/v1/platform/check-calibration-2"
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA3     "/api/v1/platform/check-calibration-3"
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA4     "/api/v1/platform/check-calibration-4"
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA5     "/api/v1/platform/check-calibration-5"
#define PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA6     "/api/v1/platform/check-calibration-6"
#define PORTAL_API_V1_PLATFORM_INSPECT_VIBRATION      "/api/v1/platform/inspect-vibration"

bool portal_api_v1_suported(void) {
    //printf("checking portal api supported");
    bool enable = false;
    FILE* file = fopen(PORTAL_API_FACTORY_MODE_FILE, "r");
    if (file != NULL) {
        char buffer[1024];
        size_t len = fread(buffer, sizeof(char), sizeof(buffer), file);
        if (ferror(file) != 0) {
            printf("error reading sku.txt");
        }
        fclose(file);
        // D0005 - Starling-V2
        if (strstr(buffer, "D0005") != NULL) {
            enable = true;
        }
        // D0006 - Sentinel Family
        else if (strstr(buffer, "D0006") != NULL) {
            enable = true;
        }
        // D0008 - FPV Family
        else if (strstr(buffer, "D0008") != NULL) {
            enable = true;
        }
        // D0004 - rb5
        else if (strstr(buffer, "D0004") != NULL) {
            enable = true;
        }
        else if (strstr(buffer, "D0010") != NULL) {
            enable = true;
        }
        else if (strstr(buffer, "D0012") != NULL) {
            enable = true;
        }
        else if (strstr(buffer, "D0013") != NULL) {
            enable = true;
        }
        // Need to update and add all compatibale drone types I am adding mine for testing: 

        else if (strstr(buffer, "D0014") != NULL) {
            enable = true;
        }
    }
    return enable;
}


void portal_api_v1_platform(struct mg_connection * c, struct mg_http_message * hm) {

    if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_STATUS)) {
        char buf[1024];
        char sys_img_buf[1024];
        char kernel_buf[1024];
        char machine_buf[1024];
        char factory_mode[1024];
        FILE *fp;

        fp = popen("more /etc/version | tr -d '\n'", "r");
        if (fp != NULL) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                memcpy(sys_img_buf, buf, 1024);
            }
            pclose(fp);
        }
        fp = popen("uname -a | tr -d '\n'", "r");
        if (fp != NULL) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                memcpy(kernel_buf, buf, 1024);
            }
            pclose(fp);
        }
        fp = popen("voxl-platform | tr -d '\n'", "r");
        if (fp != NULL) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                memcpy(machine_buf, buf, 1024);
            }
            pclose(fp);
        }
        fp = popen("more /data/modalai/sku.txt | tr -d '\n'", "r");
        if (fp != NULL) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                memcpy(factory_mode, buf, 1024);
            }
            pclose(fp);
        }
        snprintf(buf, 1024, 
            "{\"%s\":\"%s\", \"%s\":\"%s\", \"%s\":\"%s\", \"%s\":\"%s\"}", 
            "system-image", sys_img_buf, "kernel", kernel_buf, "machine", machine_buf, "factory-mode", factory_mode);

        const char *headers = "content-type: text/json\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, buf);

    }
    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_HEALTH_CHECK)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "voxl-health-check -r --json");
       // printf("running command: %s\n", cmd_buf);

        // default to a valid failure response
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");

        fp = popen(cmd_buf, "r");

        if (fp != NULL) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                
                memcpy(resp_buf, buf, 8192);
            }
            pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/json\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }

    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_INSPECT_SERVICES)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "voxl-inspect-services -j");
       // printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");

        if (fp != NULL) {
            
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                
                memcpy(resp_buf, buf, 8192);
    
            pclose(fp);
            }
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/json\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }
    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "voxl-check-calibration -j");
       // printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");

        if (fp != NULL) {
            
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                
                memcpy(resp_buf, buf, 8192);
    
            pclose(fp);
            }
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/json\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }
    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA1)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "head -n 4 /data/modalai/voxl-imu-server.cal");
       // printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }

    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA2)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "tail -n 4 /data/modalai/opencv_tracking_intrinsics.yml");
       // printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }

    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA3)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "tail -n 3 /data/px4/param/parameters_gyro.cal");
        //printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }

    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA4)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "tail -n 3 /data/px4/param/parameters_acc.cal");
        //printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }

    else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA5)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "cat /data/px4/param/parameters_level.cal");
        //printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }

else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_CHECK_CALIBRATION_FILE_DATA6)) {
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "tail -n 3 /data/px4/param/parameters_mag.cal");
        //printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }

        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }


else if (mg_http_match_uri(hm, PORTAL_API_V1_PLATFORM_INSPECT_VIBRATION)) {
        
        
        
        
        char cmd_buf[1024];
        char buf[8192];
        char resp_buf[8192];
        FILE *fp;

        snprintf(cmd_buf, 1024, "voxl-inspect-vibration imu_apps -l -t");

        //commenting out this print because it executes a ton of times
        //printf("running command: %s\n", cmd_buf);
        
        // default to a valid failure response
        
        snprintf(resp_buf, 8192, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");
        
        fp = popen(cmd_buf, "r");


        if (fp != NULL) {
            memset(resp_buf,0,sizeof(resp_buf));

            while (fgets(buf,sizeof(buf), fp) != NULL){
                strcat(resp_buf,buf);
           }
           pclose(fp);
        }
        else {
             printf("ERROR: running command: %s\n", cmd_buf);
        }
        


        const char *headers = "content-type: text/plain\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);
    }


}


void portal_api_v1_voxl_px4(struct mg_connection * c, struct mg_http_message * hm) {

    char cmd_buf[1024];
    char buf[1024];
    char resp_buf[1024];

    if (mg_http_match_uri(hm, PORTAL_API_V1_VOXL_PX4_STATUS)) {
        
        FILE *fp;

        snprintf(cmd_buf, 1024, "portal-parser -s voxl-px4 -c status");
        printf("running command: %s\n", cmd_buf);

        // default to a valid failure response
        snprintf(resp_buf, 1024, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");

        fp = popen(cmd_buf, "r");

        if (fp != NULL) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                // use the response from the portal-parser
                memcpy(resp_buf, buf, 1024);
            }
            pclose(fp);
        }

        const char *headers = "content-type: text/json\r\n";
        int status_code = 200;
        mg_http_reply(c, status_code, headers, resp_buf);


    } else if (mg_http_match_uri(hm, PORTAL_API_V1_VOXL_PX4_COMMAND)) {
        char cmd[64], arg[64];

        if(mg_http_get_var(&hm->query, "cmd", cmd, sizeof(cmd))){
            if(mg_http_get_var(&hm->query, "arg", arg, sizeof(arg))){
                
                snprintf(cmd_buf, 1024, "portal-parser -s voxl-px4 -c %s -a %s", cmd, arg);
                printf("running command: %s\n", cmd_buf);

                FILE *fp;

                // default to a valid failure response
                snprintf(resp_buf, 1024, 
                    "{\"%s\":\"%s\", \"%s\":\"%s\"}", 
                    "command", cmd_buf, "success", "false");

                fp = popen(cmd_buf, "r");
                if (fp != NULL) {
                    if (fgets(buf, sizeof(buf), fp) != NULL) {
                        // use the response from the portal-parser
                        memcpy(resp_buf, buf, 1024);
                    }
                    pclose(fp);
                }

                const char *headers = "content-type: text/json\r\n";
                int status_code = 200;
                mg_http_reply(c, status_code, headers, resp_buf);
            }
        }

    } 
}

/// @brief Handle Portal API v1 HTTP request
/// @param c connection to use
/// @param hm incoming HTTP message
void portal_api_v1(struct mg_connection * c, struct mg_http_message * hm){
    if(strstr((const char *)hm->uri.ptr, "api/v1/platform")){
        //printf("calling platform");
        portal_api_v1_platform(c, hm);
    
    } else if(strstr((const char *)hm->uri.ptr, "api/v1/voxl-px4")){
        printf("calling px4");
        portal_api_v1_voxl_px4(c, hm);

    } else if (strstr((const char *)hm->uri.ptr, "api/v1/reboot")) {
        // portal_api_v1_voxl_reboot(c, hm)
    }
}
