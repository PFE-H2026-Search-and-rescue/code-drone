#include "cmd_group_manager.h"
#include <dirent.h>
#ifdef ENABLE_MAVLINK_SUPPORT // mavlink is optional but necessary for battery/gps/flight data
#include <c_library_v2/development/mavlink.h>
#endif
#include "api/portal_api.h"
#include "manager_utils.h"
#include "header_manager.h"
#include "rc_calibration_manager.h"
#include "baro_calibration_manager.h"
#include "actuator_test.h"

#ifdef __ANDROID__
#warning building for Android
#define CAMERA_CMD "grep -lir camera_image_metadata_t /data/data/com.modalai.sensors.server/mpa/*/info | sed -e s-/data/data/com.modalai.sensors.server/mpa/--g -e s-/info--g"
#define POSE_CMD "grep -lir pose_vel_6dof_t /data/data/com.modalai.sensors.server/mpa/*/info | sed -e s-/data/data/com.modalai.sensors.server/mpa/--g -e s-/info--g"
#define PTCLOUD_CMD "grep -lir point_cloud_metadata_t /data/data/com.modalai.sensors.server/*/info | sed -e s-/data/data/com.modalai.sensors.server/mpa/--g -e s-/info--g"
#else
#define CAMERA_CMD "grep -lir camera_image_metadata_t /run/mpa/*/info | sed -e s-/run/mpa/--g -e s-/info--g"
#define IMU_CMD "voxl-list-pipes -t imu_data_t"
#define POSE_CMD "grep -lir pose_vel_6dof_t /run/mpa/*/info | sed -e s-/run/mpa/--g -e s-/info--g"
#define PTCLOUD_CMD "grep -lirE '(point_cloud_metadata_t|tof2_data_t|rangefinder_data_t)' /run/mpa/*/info | sed -e s-/run/mpa/--g -e s-/info--g"
#define VIO_CMD "grep -lir '\"vio_data_t\"' /run/mpa/*/info | sed -e s-/run/mpa/--g -e s-/info--g"
#define LOG_CMD "ls /etc/systemd/system | grep '^voxl.*\\.service$'"
#endif // __ANDROID__
// Messages sent from the portal are tagged with this component ID to
// differentiate their origin from packets from QGC or PX4
#define PORTAL_COMPID			MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

// save sysid of the autopilot
// start with default of 1 that mode APs default to
static uint8_t current_sysid = 1;
extern int jpg_quality;

static uint8_t autopilot_monitor_get_sysid(void)
{
	return current_sysid;
}

int motorVals[4] = {0, 0, 0, 0};

void CmdGroupManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    if (ev == MG_EV_HTTP_MSG)
    {
        struct mg_http_message *hm = (struct mg_http_message *)ev_data;

        if (mg_http_match_uri(hm, CameraCmdClaimString))
        {
            char response[512];
            FILE *fp = popen(CAMERA_CMD, "r");

            if (fp == NULL)
            {
                printf("Failed to run command\n");
                exit(1);
            }

            char *current_pos = response;
            int len;

            while (fgets(current_pos, sizeof(response), fp))
            {
                int len = strlen(current_pos);
                current_pos[len - 1] = ' ';
                current_pos = &current_pos[len];
            }

            *current_pos = 0;

            pclose(fp);

            mg_http_reply(c, 200, "", "%s", response);
        }
        else if (mg_http_match_uri(hm, ImuCmdClaimString))
        {
            char response[512];
            FILE *fp = popen(IMU_CMD, "r");

            if (fp == NULL)
            {
                printf("Failed to run command\n");
                exit(1);
            }

            char *current_pos = response;
            int len;

            while (fgets(current_pos, sizeof(response), fp))
            {
                int len = strlen(current_pos);
                current_pos[len - 1] = ' ';
                current_pos = &current_pos[len];
            }

            *current_pos = 0;

            pclose(fp);

            mg_http_reply(c, 200, "", "%s", response);
        }
        else if (mg_http_match_uri(hm, PtcloudCmdClaimString))
        {
            char response[512];
            FILE *fp = popen(PTCLOUD_CMD, "r");

            if (fp == NULL)
            {
                printf("Failed to run command\n");
                exit(1);
            }

            char *current_pos = response;
            int len;

            while (fgets(current_pos, sizeof(response), fp))
            {
                int len = strlen(current_pos);
                current_pos[len - 1] = ' ';
                current_pos = &current_pos[len];
            }

            *current_pos = 0;

            pclose(fp);

            mg_http_reply(c, 200, "", "%s", response);
        }
        else if (mg_http_match_uri(hm, PoseCmdClaimString))
        {
            char response[512];
            FILE *fp = popen(POSE_CMD, "r");

            if (fp == NULL)
            {
                printf("Failed to run command\n");
                exit(1);
            }

            char *current_pos = response;
            int len;

            while (fgets(current_pos, sizeof(response), fp))
            {
                int len = strlen(current_pos);
                current_pos[len - 1] = ' ';
                current_pos = &current_pos[len];
            }

            *current_pos = 0;

            pclose(fp);

            mg_http_reply(c, 200, "", "%s", response);
        }
        else if (mg_http_match_uri(hm, VioCmdClaimString)) {
            char response[512];
            FILE *fp = popen(VIO_CMD, "r");

            if (fp == NULL) {
                printf("Failed to run command\n");
                exit(1);
            }

            char *current_pos = response;
            int len;

            while (fgets(current_pos, sizeof(response), fp)) {
                int len = strlen(current_pos);
                current_pos[len - 1] = ' ';
                current_pos = &current_pos[len];
            }

            *current_pos = 0;

            pclose(fp);

            mg_http_reply(c, 200, "", "%s", response);
        }
        else if (mg_http_match_uri(hm, LogCmdClaimString)) {
            char response[8192];
            FILE *fp = popen(LOG_CMD, "r");

            if (fp == NULL) {
                printf("Failed to run command\n");
                exit(1);
            }

            char *current_pos = response;
            int len;

            while (fgets(current_pos, sizeof(response), fp)) {
                int len = strlen(current_pos);
                current_pos[len - 1] = ' ';
                current_pos = &current_pos[len];
            }

            *current_pos = 0;

            pclose(fp);

            mg_http_reply(c, 200, "", "%s", response);
        }
        else if (mg_http_match_uri(hm, DownloadLogCmdClaimString)) {
            int size = 1024;
            char buf[100] = "";

            if (mg_http_get_var(&hm->query, "log_name", buf, sizeof(buf))) {
                printf("Request to download log: %s\n", buf);
                char response[8192];

                char command[256];

                strcat(command, "journalctl -u ");

                strcat(command, buf);
                strcat(command, " > /tmp/log_output.txt");

                printf("Executing command: %s\n", command);
                FILE *fp = popen(command, "r");

                if (fp == NULL) {
                    printf("Failed to run command\n");
                    exit(1);
                }

                char *current_pos = response;
                int len;

                while (fgets(current_pos, sizeof(response), fp)) {
                    int len = strlen(current_pos);
                    current_pos[len - 1] = ' ';
                    current_pos = &current_pos[len];
                }

                *current_pos = 0;

                pclose(fp);



                struct mg_http_message *hm = (struct mg_http_message *) ev_data;
                struct mg_http_serve_opts opts;
        
                // Initialize opts with default values
                memset(&opts, 0, sizeof(opts));
                
                // Set the appropriate MIME type for your log file
                opts.mime_types = "text/plain";
                
                // Optional extra headers
                opts.extra_headers = "Content-Disposition: attachment\r\n";
                mg_http_serve_file(c, hm, "/tmp/log_output.txt", &opts);
            }
        }
        else if (mg_http_match_uri(hm, PlatformCmdClaimString))
        {
            #ifdef BUILD_QRB5165 // voxl2
            mg_http_reply(c, 200, "", "QRB");
            #else
            mg_http_reply(c, 200, "", "APQ");
            #endif
        }
        else if (mg_http_match_uri(hm, HealthCheckCmdClaimString))
        {

            /*
            #ifdef BUILD_QRB5165 // voxl2
            if(portal_api_v1_suported())
                 mg_http_reply(c, 200, "", "true");
            else
                 mg_http_reply(c, 200, "", "false");
            #else
            mg_http_reply(c, 200, "", "false");
            #endif
            */

            if (portal_api_v1_suported()) {
                //printf("set to true");
                mg_http_reply(c, 200, "", "true");
            }
            else {
                //printf("set to false");
                mg_http_reply(c, 200, "", "false");
            }
        } else if (mg_http_match_uri(hm, MagCalibrationStartClaimString)) {
            char buf[10] = "";
            if (mg_http_get_var(&hm->query, "sensor", buf, sizeof(buf))) {
                printf("Start mag calibration request received\n");
                printf("Sensor: %s\n", buf);
                float param1 = 0;
                float param2 = 0;
                float param3 = 0;
                float param4 = 0;
                float param5 = 0;
                float param6 = 0;
                float param7 = 0;

                // We only support gyro, mag, and accel for now, but we can add more later if needed.
                if(strcmp(buf, "Gyro") == 0) {
                    param1 = 1;
                } else if(strcmp(buf, "Mag") == 0) {
                    printf("Starting mag calibration\n");
                    param2 = 1;
                } else if(strcmp(buf, "Accel") == 0) {
                    param5 = 1;
                } else if (strcmp(buf, "Level") == 0) {
                    param5 = 2;
                } else if (strcmp(buf, "RC") == 0) {
                    param4 = 1;
                    handleStep(0);
                } else if (strcmp(buf, "RC_trim") == 0) {
                    param4 = 2;
                } else {
                    mg_http_reply(c, 400, "", "Invalid sensor type");
                    return;
                }
                mavlink_message_t msg;
                mavlink_msg_command_long_pack(autopilot_monitor_get_sysid(), \
                    PORTAL_COMPID, &msg, autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1, \
                    MAV_CMD_PREFLIGHT_CALIBRATION, 0, param1, param2, param3, param4, param5, param6, param7);
    
                _send_mavlink_msg_to_ap(&msg);
                mg_http_reply(c, 200, "", "true");
            }
        } else if (mg_http_match_uri(hm, RCCalibrationString)) {
            char buf[10] = "";
            int step;
            if (mg_http_get_var(&hm->query, "step", buf, sizeof(buf))) {
                sscanf(buf, "%d", &step);
            }
            if (step == -1) {
                cancel_calibration();
            } else if (step == 1) {
                run_rc_calibration();
            } else {
                handleStep(step);
            }

            mg_http_reply(c, 200, "", "true");
        }
        else if (mg_http_match_uri(hm, MagCalibrationStopClaimString)) {
            mavlink_message_t msg;
            mavlink_msg_command_long_pack(autopilot_monitor_get_sysid(), \
                    PORTAL_COMPID, &msg, autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1, \
                    MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 0, 0, 0);
            _send_mavlink_msg_to_ap(&msg);
            mg_http_reply(c, 200, "", "true");
        } else if (mg_http_match_uri(hm, BaroCalibrationStartClaimString)) {
            run_baro_calibration();
            mg_http_reply(c, 200, "", "true");
        } else if (mg_http_match_uri(hm, BaroCalibrationContinueClaimString)) {
            continue_baro_calibration();
            mg_http_reply(c, 200, "", "true");
        } else if(mg_http_match_uri(hm, ActuatorTestCmdString)) {
            char buf_motor_num[10] = "";
            char buf_motor_val[10] = "";
            char buf_motor_test[10] = "";

            int motor_num;
            int motor_val;
            int test;

            if (mg_http_get_var(&hm->query, "motor_num", buf_motor_num, sizeof(buf_motor_num)) &&
                mg_http_get_var(&hm->query, "motor_val", buf_motor_val, sizeof(buf_motor_val)) &&
                mg_http_get_var(&hm->query, "test", buf_motor_test, sizeof(buf_motor_test))) {

                sscanf(buf_motor_num, "%d", &motor_num);
                sscanf(buf_motor_val, "%d", &motor_val);
                sscanf(buf_motor_test, "%d", &test);


                if (test == 1) {

                    for (size_t i = 0; i < 20; i++) {
                        send_motor_val(motor_num - 1, 0.15);
                    }

                    mg_http_reply(c, 200, "", "true");
                    return;
                }

            } else {
                mg_http_reply(c, 400, "", "false");
                return;
            }

            motorVals[motor_num - 1] = motor_val;
            mg_http_reply(c, 200, "", "true");

        } else if (mg_http_match_uri(hm, ParamSetString)) {
            // Handles setting a float parameter

            char param_id_buffer[PARAM_BUFFER_INITIAL_SIZE];
            char param_val_buffer[PARAM_BUFFER_INITIAL_SIZE];
            float param_val;

            int param_id_length = mg_http_get_var(&hm->query, "param_id", param_id_buffer, sizeof(param_id_buffer));

            if (mg_http_get_var(&hm->query, "param_val", param_val_buffer, 20)) {
                sscanf(param_val_buffer, "%f", &param_val);
            }

            setParam(param_id_buffer, param_val, param_id_length);
            mg_http_reply(c, 200, "", "true");
        } else if (mg_http_match_uri(hm, ParamSetIntString)) {
            // Handles setting an int parameter 

            char param_id_buffer[PARAM_BUFFER_INITIAL_SIZE];
            char param_val_buffer[PARAM_BUFFER_INITIAL_SIZE];
            int32_t param_val;

            int param_id_length = mg_http_get_var(&hm->query, "param_id", param_id_buffer, sizeof(param_id_buffer));

            if (mg_http_get_var(&hm->query, "param_val", param_val_buffer, 20)) {
                sscanf(param_val_buffer, "%d", &param_val);
            }

            setParam(param_id_buffer, param_val, param_id_length);
            mg_http_reply(c, 200, "", "true");
        } else if (mg_http_match_uri(hm, ParamGetString)) {
            // Handles getting a parameter of any type
            
            char param_id_buffer[PARAM_BUFFER_INITIAL_SIZE];
            int param_id_length = mg_http_get_var(&hm->query, "param_id", param_id_buffer, sizeof(param_id_buffer));

            request_getParam(param_id_buffer, param_id_length);
            mg_http_reply(c, 200, "", "true"); 
        } else if (mg_http_match_uri(hm, BaroCalibrationSaveParamsString)) {
            save_baro_calibration_params();
            mg_http_reply(c, 200, "", "true");
        } else if (mg_http_match_uri(hm, CameraControlCmdString)) {
            char cam_name_buffer[128];
            char control_id_buffer[32];
            char control_val_buffer[32];
            float control_val;

            memset(cam_name_buffer,0,sizeof(cam_name_buffer));

            int cam_name_length   = mg_http_get_var(&hm->query, "cam_name", cam_name_buffer, sizeof(cam_name_buffer));
            int control_id_length = mg_http_get_var(&hm->query, "control_id", control_id_buffer, sizeof(control_id_buffer));

            if (mg_http_get_var(&hm->query, "control_val", control_val_buffer, sizeof(control_val_buffer))) {
                sscanf(control_val_buffer, "%f", &control_val);
            }

            //printf("cam: %s, control_id: %s, control_val %f\n",cam_name_buffer, control_id_buffer, control_val);

            std::string control_cmd = std::string(control_id_buffer);

            bool valid_cmd = 0;
            std::string cmd;

            if (control_cmd == "zoom"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_misp_zoom " + std::to_string(control_val);
                valid_cmd = 1;
            } else if (control_cmd == "bitrate"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_misp_venc_mbps " + std::to_string(control_val);
                valid_cmd = 1;
            } else if (control_cmd == "quality"){
                valid_cmd = 0;  //no need to send out control message
                if (control_val <1)    control_val = 1;
                if (control_val > 100) control_val = 100;

                jpg_quality = control_val;
                //printf("updated jpeg quality to %d\n",jpg_quality);
            } else if (control_cmd == "en_eis"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_misp_en_eis " + std::to_string(control_val);
                valid_cmd = 1;
            } else if (control_cmd == "exposure"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_exp " + std::to_string(control_val);
                valid_cmd = 1;
            } else if (control_cmd == "gain"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_gain " + std::to_string(control_val);
                valid_cmd = 1;
            } else if (control_cmd == "gamma"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_misp_gamma " + std::to_string(control_val);
                valid_cmd = 1;
            }
              else if (control_cmd == "fps"){
                cmd = std::string("voxl-send-command ") + cam_name_buffer +" set_fps " + std::to_string(control_val);
                valid_cmd = 1;
            }
            else{
                printf("unknown control id : %s\n",control_cmd.c_str());
            }

            if (valid_cmd){
                std::string response;

                char temp_buf[512];

                FILE *fp = popen(cmd.c_str(), "r");

                if (fp == NULL)
                {
                    printf("Failed to run command\n");
                }
                else{

                    while (fgets(temp_buf, sizeof(temp_buf), fp))
                    {
                        response += temp_buf;
                    }
                    pclose(fp);
                    //printf("got response:\n");
                    //printf("%s\n",response.c_str());
                }
                /*
                FILE *fp = popen(cmd.c_str(), "r");

                if (fp == NULL)
                {
                    printf("Failed to run command\n");
                }
                else{
                    char response[512];
                    char *current_pos = response;
                    int len;

                    while (fgets(current_pos, sizeof(response), fp))
                    {
                        int len = strlen(current_pos);
                        current_pos[len - 1] = ' ';
                        current_pos = &current_pos[len];
                    }

                    *current_pos = 0;
                    pclose(fp);
                }
                */
            }

            mg_http_reply(c, 200, "", "true");
        }
    }
}


void setParam(char* param_id_buffer, float param_val, int param_id_length) {
    if (foramt_param_id_buffer(param_id_buffer, param_id_length) < 0) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), \
            PORTAL_COMPID, &msg, autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1, \
            param_id_buffer, param_val, MAV_PARAM_TYPE_REAL32);
    _send_mavlink_msg_to_ap(&msg);
}



void setParam(char* param_id_buffer, int32_t param_val, int param_id_length) {
    if (foramt_param_id_buffer(param_id_buffer, param_id_length) < 0) {
        return;
    }

    mavlink_message_t* msg = new mavlink_message_t;

    // Converts param_val to big endian / floating point
    union {
        int32_t integer;
        float floating;
    } data;

    // Copy the bytes into the union
    memcpy(&data.integer, &param_val, 4);

    mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), \
            PORTAL_COMPID, msg, autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1, \
            param_id_buffer, data.floating, MAV_PARAM_TYPE_INT32);
    _send_mavlink_msg_to_ap(msg);
}

void request_getParam(char* param_id_buffer, int param_id_length) {
    if (foramt_param_id_buffer(param_id_buffer, param_id_length) < 0) {
        return;
    }

    mavlink_message_t* msg = new mavlink_message_t;
    mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), \
            PORTAL_COMPID, msg, autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1, \
            param_id_buffer, -1); // param_index set to -1 signaling PX4 to use param_id_buffer not param_index
    _send_mavlink_msg_to_ap(msg);
}

void calibrationFinished() {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(autopilot_monitor_get_sysid(), \
        PORTAL_COMPID, &msg, autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1, \
        MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 0, 0, 0);

    _send_mavlink_msg_to_ap(&msg);
}

int foramt_param_id_buffer(char* param_id_buffer, int param_id_length) {
    // PX4 needs the param id buffer to be null terminated if its length is less than 16
    if (param_id_buffer == nullptr) {
        return -1;
    }


    if (param_id_length < PX4_MAX_PARAM_LENGTH) {
        param_id_buffer[PX4_MAX_PARAM_LENGTH - 1] = '\0';
    }

    return 1;
}
