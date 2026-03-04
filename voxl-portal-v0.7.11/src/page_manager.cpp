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
#include "page_manager.h"

#include <modal_start_stop.h>
#include <string.h>
#include "cmd_group_manager.h"
#include "costmap_manager.h"
#include "cpu_manager.h"
#include "debug_version.h"
#include "header_manager.h"
#include "imu_manager.h"
#include "mapper_ptcloud_manager.h"
#include "mesh_manager.h"
#include "plan_manager.h"
#include "pointcloud_group_manager.h"
#include "pose_manager.h"
#include "api/portal_api.h"
#include "vio_manager.h"
#include "video_group_manager.h"
#include "log_manager.h"
#include "rc_calibration_manager.h"
#include "baro_calibration_manager.h"
#include "actuator_test.h"
#include "benchmark_manager.h" 
#include "gps_graph_manager.h"

struct mg_mgr mgr_;

void PageManager::Run(char *web_root, int flags)
{
   
    this->flags = flags;
    main_running = 1;
    // mg_log_set("3");
    // start signal handler so we can exit cleanly
    if (enable_signal_handler() == -1)
    {
        fprintf(stderr, "ERROR: failed to start signal handler\n");
        return;
    }

    mg_mgr_init(&mgr_);
    
    num_pages_ = 0;
    RegisterPages();
    mg_http_listen(&mgr_, "http://0.0.0.0:80", PageManager::ServerCallback, web_root);
    printf("voxl-portal started up\n");
    while (main_running)
        mg_mgr_poll(&mgr_, 30);

    mg_mgr_free(&mgr_);
    pipe_client_close_all();
}

void PageManager::ServerCallback(struct mg_connection *c, int ev,
                                 void *ev_data, void *fn_data)
{
    if (ev == MG_EV_WS_MSG) // Websocket msg, text or bin
    {
        
        // When we upgrade to websocket the claim string was saved in the fn data
        std::string &claim_str = *(static_cast<std::string*>(c->fn_data));

        // Got websocket frame. Received data is wm->data.
        struct mg_ws_message *wm = (struct mg_ws_message *)ev_data;

        char msg[128] = {0};
        memcpy(msg, wm->data.ptr, wm->data.len);
        mg_iobuf_del(&c->recv, 0, c->recv.len);

        if (claim_str == PlanManagerClaimString)
        {
            sendPlanCmd(msg, sizeof(msg));
        }
        else if (claim_str == MeshManagerClaimString)
        {
            sendMapperCmd(msg, sizeof(msg));
        }
        else if (claim_str == CheckCameraH264ClaimString)
        {
            PageManager *pm = PageManager::GetInstance();
            pm->callbacks_[claim_str](c, ev, msg, NULL);
        }
        
    }
    else if (ev == MG_EV_HTTP_MSG) //  HTTP request/response 
    {
       
        struct mg_http_message *hm = (struct mg_http_message *)ev_data;

        PageManager *pm = PageManager::GetInstance();

        // API request?
        if((hm->uri.ptr != NULL) && strstr((const char *)hm->uri.ptr, "/api/v1")){
           portal_api_v1(c,hm);
        } else {
            bool page_found = false;
            // Iterate over all websocket strings to check if we should upgrade and
            // then call the appropriate callback
            //fprintf(stderr,'before for loop');
            printf("Received URI: %.*s\n", (int)hm->uri.len, hm->uri.ptr);
            
            for (std::string &claim_str : pm->claim_strings)
            {   
                // printf("claim str: %s, %d\n", claim_str.c_str(), mg_http_match_uri(hm, claim_str.c_str()));
                if (mg_http_match_uri(hm, claim_str.c_str()))
                {
                    // printf("matched uri str: %s\n", claim_str.c_str());
                    // Upgrade all to ws connection EXCEPT pure http request/response groups
                    if (claim_str != BenchmarkManagerClaimString && claim_str != BenchmarkFileClaimmString && claim_str != CmdGroupManagerClaimString && claim_str != VideoGroupManagerClaimString && claim_str != MeshApiClaimString  && claim_str != LogManagerClaimString)
                    {
                        printf("doing mg_ws_upgrade\n");
                        mg_ws_upgrade(c, hm, NULL);
                        // Save the claim string inside the connection
                        c->fn_data = static_cast<void*>(&claim_str);
                    }

                    // printf("doing callback\n");
                    // Lookup and run the appropriate callback
                    pm->callbacks_[claim_str](c, ev, ev_data, NULL);

                    page_found = true;
                    break;
                }
            }

            printf("page found %d\n", page_found);
            // If not page found serve the default directory
            if (!page_found)
            {
                struct mg_http_serve_opts opts = {.root_dir = (char *)fn_data};
                mg_http_serve_dir(c, hm, &opts);
            }
        }
    }
    else if (ev == MG_EV_WS_CTL)
    {
        struct mg_ws_message *ws_msg = (struct mg_ws_message *)ev_data;

        PageManager *pm = PageManager::GetInstance();

        switch (ws_msg->flags & 0x0F)
        {
        case WEBSOCKET_OP_CLOSE:
        {
            // Set closing flag to true
            c->is_closing = true;

            // We previously stored the claim string in the fn_data pointer so
            // now we use it to find the callback handler (which should delete it)
            std::string &claim_str = *(static_cast<std::string*>(c->fn_data));
            pm->callbacks_[claim_str](c, ev, ev_data, NULL);
            break;
        }

        default:
            break;
        }
    }
    (void)fn_data;
}

int PageManager::RegisterPages()
{
    claim_strings = {
        BenchmarkManagerClaimString, BenchmarkFileClaimmString, VideoGroupManagerClaimString, CmdGroupManagerClaimString, PointcloudGroupManagerClaimString,
        VideoStatsManagerClaimString, ImuManagerClaimString, PoseManagerClaimString,
        MeshManagerClaimString, PlanManagerClaimString, CostmapManagerClaimString,
        PtcloudManagerClaimString, CpuManagerClaimString, HeaderManagerClaimString,
        MeshApiClaimString, FftManagerClaimString, SnapshotManagerClaimString, MapperManagerClaimString, 
        SnapshotCheckClaimString, VersionInfoClaimString, H264ClaimString, 
        SuiteInfoClaimString, CloseH264ClaimString, CloseSnapshotClaimString, VioManagerClaimString, VIOCloseCameraClaimString, 
        ExtractFeaturesClaimString, ResetQVIOClaimString, ResetOVClaimString, CheckCameraH264ClaimString, CloseDownLastSingleCamClaimString,
        RCCalibrationClaimString, BaroCalibrationClaimString, TestActuatorsClaimString, GpsGraphClaimString,ImuTDKClaimString};

    // Group Managers vs Managers:
    // Group Managers deal with a group of MPA topics to properly display information.
    // As such they accept additional commands past their claim string.
    // For example the video manager may need to handle multiple camera topics
    // in order to properly display data.
    //
    // Managers only deal with a single MPA topic. For example, the IMU handler
    // only deals with the data coming through the IMU topic.

    // Group Manager Callbacks
    //  Video manager handles all requests marked /video*
    callbacks_[VideoGroupManagerClaimString] = VideoGroupManagerCallback;

    // CMD manager handles all requests marked /_cmd*
    callbacks_[CmdGroupManagerClaimString] = CmdGroupManagerCallback;

    // Ptc manager handles all requests marked /pointcloud*
    callbacks_[PointcloudGroupManagerClaimString] = PointcloudGroupManagerCallback;

    //
    callbacks_[VioManagerClaimString] = VioManagerCallback;

    // Manager Callbacks
    callbacks_[VideoStatsManagerClaimString] = VideoStatsManagerCallback;
    callbacks_[SnapshotManagerClaimString] = SnapshotManagerCallback;
    callbacks_[SnapshotCheckClaimString] = SnapshotCheckCallback;
    callbacks_[MapperManagerClaimString] = MapperManagerCallback;
    callbacks_[ImuManagerClaimString] = ImuManagerCallback;
    callbacks_[ImuTDKClaimString] = ImuManagerCallback;
    callbacks_[PoseManagerClaimString] = PoseManagerCallback;
    callbacks_[MeshManagerClaimString] = MeshManagerCallback;
    callbacks_[PlanManagerClaimString] = PlanManagerCallback;
    callbacks_[CostmapManagerClaimString] = CostmapManagerCallback;
    callbacks_[PtcloudManagerClaimString] = PtcloudManagerCallback;
    callbacks_[CpuManagerClaimString] = CpuManagerCallback;
    callbacks_[HeaderManagerClaimString] = HeaderManagerCallback;
    callbacks_[LogManagerClaimString] = LogManagerCallback;
    callbacks_[VIOCloseCameraClaimString] = VIOCloseCameraCallback;
    callbacks_[MeshApiClaimString] = MeshApiCallback;
    callbacks_[VersionInfoClaimString] = VersionInfoCallback;
    callbacks_[H264ClaimString] = H264Callback;
    callbacks_[SuiteInfoClaimString] = SuiteInfoCallback;
    callbacks_[CloseH264ClaimString] = CloseH264Callback;
    callbacks_[CloseSnapshotClaimString] = CloseSnapshotCallback;
    callbacks_[ExtractFeaturesClaimString] = ExtractFeaturesCallback;
    callbacks_[ResetQVIOClaimString] = ResetQVIOCallback;
    callbacks_[ResetOVClaimString] = ResetOVCallback;
    callbacks_[CheckCameraH264ClaimString] = CheckCameraH264Callback;
    callbacks_[CloseDownLastSingleCamClaimString] = CloseDownLastSingleCamCallback;
    callbacks_[RCCalibrationClaimString] = RCCalibrationCallback;
    callbacks_[BaroCalibrationClaimString] = BaroCalibrationCallback;
    callbacks_[TestActuatorsClaimString] = ActuatorTestCallback;
    callbacks_[BenchmarkManagerClaimString] = BenchmarkManagerCallback;
    callbacks_[BenchmarkFileClaimmString] = BenchmarkManagerCallback;
    callbacks_[GpsGraphClaimString] = GpsGraphManagerCallback;
    #ifdef BUILD_QRB5165
    callbacks_[FftManagerClaimString] = FftManagerCallback;
    #endif

    return 0;
}
