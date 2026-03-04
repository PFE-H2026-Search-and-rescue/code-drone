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
#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H


extern "C" {
#include "mongoose.h"

#define ImuCmdClaimString   "/_cmd/list_imus"
#define CmdGroupManagerClaimString "/_cmd/*"
#define CameraCmdClaimString "/_cmd/list_cameras"
#define PtcloudCmdClaimString "/_cmd/list_ptclouds"
#define PoseCmdClaimString "/_cmd/list_poses"
#define VioCmdClaimString "/_cmd/list_vio"
#define LogCmdClaimString "/_cmd/list_logs"
#define DownloadLogCmdClaimString "/_cmd/download_log"
#define PlatformCmdClaimString "/_cmd/platform"
#define HealthCheckCmdClaimString "/_cmd/health_check_enabled"
#define MagCalibrationStartClaimString "/_cmd/mag_calibration_start"
#define MagCalibrationStopClaimString "/_cmd/mag_calibration_stop"
#define BaroCalibrationStartClaimString "/_cmd/baro_calibration_start"
#define BaroCalibrationContinueClaimString "/_cmd/baro_calibration_continue"
#define BaroCalibrationSaveParamsString "/_cmd/baro_calibration_save_params"
#define RCCalibrationString "/_cmd/rc_calibration"
#define ParamSetString "/_cmd/param_set"
#define ParamSetIntString "/_cmd/param_set_int"
#define ParamGetString "/_cmd/param_request"
#define CameraControlCmdString "/_cmd/cam_control"
#define ActuatorTestCmdString "/_cmd/actuator_test"

#define valid_char(x) ((x >= ' ' && x <= '~') || (x == '\n'))

void CmdGroupManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);
}

void setParam(char* param_id_buffer, float param_val, int param_id_length);
void setParam(char* param_id_buffer, int32_t param_val, int param_id_length);
void request_getParam(char* param_id_bufer, int param_id_length);
void calibrationFinished();
int foramt_param_id_buffer(char* param_id_buffer, int param_id_length);

#endif
