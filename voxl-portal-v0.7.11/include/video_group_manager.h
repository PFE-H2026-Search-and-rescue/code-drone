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
#ifndef VIDEO_MANAGER_H
#define VIDEO_MANAGER_H

extern "C"
{
#include "mongoose.h"

#define VideoGroupManagerClaimString "/video_raw/*"
#define VideoStatsManagerClaimString "/video_stats"
#define SnapshotManagerClaimString "/snapshot/*"
#define SnapshotCheckClaimString "/check/*"
#define H264ClaimString "/H264/*"
#define CloseH264ClaimString "/close_h264/*"
#define CloseSnapshotClaimString "/close_snapshot/*"
#define VIOCloseCameraClaimString "/vio_close/*"
#define CheckCameraH264ClaimString "/check_h264/*"
#define CloseDownLastSingleCamClaimString "/close_last_single/*"

void VideoGroupManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void VideoStatsManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void SnapshotManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void SnapshotCheckCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void H264Callback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void CloseH264Callback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void CloseSnapshotCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void VIOCloseCameraCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void CheckCameraH264Callback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void CloseDownLastSingleCamCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);
}

#endif // PAGE_VIDEO_H
