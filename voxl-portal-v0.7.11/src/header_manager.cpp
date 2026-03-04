#include "header_manager.h"
#include "spoof_gcs.h"
#include "rc_calibration_manager.h"
#include "gps_graph_manager.h"

#include <voxl_cutils.h>
#ifdef ENABLE_MAVLINK_SUPPORT // mavlink is optional but necessary for battery/gps/flight data
#include <c_library_v2/development/mavlink.h>
#endif

#include <modal_pipe.h>

#include <algorithm>
#include <vector>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

#include "manager_utils.h"
#include "page_manager.h"
#include "pipe_channels.h"

#define FLIGHT_INFO_INDEX 0
#define STATUS_TEXT_INDEX 1
#define RC_CHANNELS_INDEX 2
#define PARAM_INDEX       5

extern struct mg_mgr mgr_;

websocket_info ws_info;

typedef struct flight_msg
{
	uint8_t msg_index;
	uint8_t armed;
	uint8_t num_sats;
	uint8_t main_mode;
	uint32_t sub_mode;
	uint8_t batt_remaining;
} __attribute__((packed)) flight_msg;

typedef struct param_msg
{
	uint8_t msg_index;
	char buffer[PX4_MAX_PARAM_LENGTH];
	int32_t param_val;
} __attribute__((packed)) param_msg;

// current "state" of the autopilot
static flight_msg curr_msg;

// param msg
static struct param_msg curr_param_msg;

// current rc channels msg
extern struct rc_channels_msg curr_rc_channels_msg;
extern uint16_t curr_rc_channels_array[18];

static void _flight_info_connect_cb(int ch, __attribute__((unused)) void *context)
{
	printf("Connected to voxl-mavlink-server gcs pipe ch %d\n", ch);
}

static void _flight_info_disconnect_cb(int ch, __attribute__((unused)) void *context)
{
	printf("Disconnected from voxl-mavlink-server gcs pipe ch %d\n", ch);
}

static void _onboard_connect_cb(int ch, __attribute__((unused)) void *context) {
	printf("Connected to voxl-mavlink-server onboard pipe ch %d\n", ch);
}

static void _onboard_disconnect_cb(int ch, __attribute__((unused)) void *context)
{
	printf("Disconnected from voxl-mavlink-server onboard pipe ch %d\n", ch);
}

int _send_mavlink_msg_to_ap(mavlink_message_t* msg) {
	int ret;
#ifdef ENABLE_MAVLINK_SUPPORT // mavlink is optional, most clients won't use it
	printf("Sending mavlink message to autopilot\n");
	ret = pipe_client_send_control_cmd_bytes(MAVLINK_PIPE_CH, msg, sizeof(mavlink_message_t));
#endif // ENABLE_MAVLINK_SUPPORT
	return ret;
}

static void FlightInfoCallback(int ch, char *data, int bytes, __attribute__((unused)) void *context)
{
#ifdef ENABLE_MAVLINK_SUPPORT // mavlink is optional, most clients won't use it


	int should_send = 0;

	if (!ws_info.connected) return;
	curr_msg.msg_index = FLIGHT_INFO_INDEX;
	curr_rc_channels_msg.msg_index = RC_CHANNELS_INDEX;
	curr_param_msg.msg_index = PARAM_INDEX;

	int n_packets, i;
	mavlink_message_t* msg_array = pipe_validate_mavlink_message_t(data, bytes, &n_packets);
	if(msg_array == NULL){
		pipe_client_flush(ch);
		return;
	}

	// loop through all packets, trimming out useful data
	for(i=0;i<n_packets;i++){

		mavlink_message_t* msg = &msg_array[i];
		if (msg->msgid == MAVLINK_MSG_ID_STATUSTEXT){
			char status_text[128];
			char *text = (char *)&_MAV_PAYLOAD(msg)[1]; // pointer to text in the mavlink msg
			memset(status_text, 0, 128);
			status_text[0] = STATUS_TEXT_INDEX;
			snprintf(&status_text[1], 127, "%s", text);
		}

		if (msg->msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
			curr_rc_channels_msg.chancount = mavlink_msg_rc_channels_get_chancount(msg);
			curr_rc_channels_array[0] = curr_rc_channels_msg.chan1_raw = mavlink_msg_rc_channels_get_chan1_raw(msg);
			curr_rc_channels_array[1] = curr_rc_channels_msg.chan2_raw = mavlink_msg_rc_channels_get_chan2_raw(msg);
			curr_rc_channels_array[2] = curr_rc_channels_msg.chan3_raw = mavlink_msg_rc_channels_get_chan3_raw(msg);
			curr_rc_channels_array[3] = curr_rc_channels_msg.chan4_raw = mavlink_msg_rc_channels_get_chan4_raw(msg);
			curr_rc_channels_array[4] = curr_rc_channels_msg.chan5_raw = mavlink_msg_rc_channels_get_chan5_raw(msg);
			curr_rc_channels_array[5] = curr_rc_channels_msg.chan6_raw = mavlink_msg_rc_channels_get_chan6_raw(msg);
			curr_rc_channels_array[6] = curr_rc_channels_msg.chan7_raw = mavlink_msg_rc_channels_get_chan7_raw(msg);
			curr_rc_channels_array[7] = curr_rc_channels_msg.chan8_raw = mavlink_msg_rc_channels_get_chan8_raw(msg);
			curr_rc_channels_array[8] = curr_rc_channels_msg.chan9_raw = mavlink_msg_rc_channels_get_chan9_raw(msg);
			curr_rc_channels_array[9] = curr_rc_channels_msg.chan10_raw = mavlink_msg_rc_channels_get_chan10_raw(msg);
			curr_rc_channels_array[10] = curr_rc_channels_msg.chan11_raw = mavlink_msg_rc_channels_get_chan11_raw(msg);
			curr_rc_channels_array[11] = curr_rc_channels_msg.chan12_raw = mavlink_msg_rc_channels_get_chan12_raw(msg);
			curr_rc_channels_array[12] = curr_rc_channels_msg.chan13_raw = mavlink_msg_rc_channels_get_chan13_raw(msg);
			curr_rc_channels_array[13] = curr_rc_channels_msg.chan14_raw = mavlink_msg_rc_channels_get_chan14_raw(msg);
			curr_rc_channels_array[14] = curr_rc_channels_msg.chan15_raw = mavlink_msg_rc_channels_get_chan15_raw(msg);
			curr_rc_channels_array[15] = curr_rc_channels_msg.chan16_raw = mavlink_msg_rc_channels_get_chan16_raw(msg);
			curr_rc_channels_array[16] = curr_rc_channels_msg.chan17_raw = mavlink_msg_rc_channels_get_chan17_raw(msg);
			curr_rc_channels_array[17] = curr_rc_channels_msg.chan18_raw = mavlink_msg_rc_channels_get_chan18_raw(msg);			

			struct mg_connection *c;
			for (c = mgr_.conns; c != NULL; c = c->next){
				if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
					continue;
				}

				// check if we are backing up the socket
                // if so, just drop the packet for this connection
                if (c->send.len > sizeof(struct rc_channels_msg))
                {
                    return;
                }

				char rc_channel_buffer[sizeof(struct rc_channels_msg)];
				memcpy(rc_channel_buffer, &curr_rc_channels_msg, sizeof(struct rc_channels_msg));
				mg_ws_send(c, rc_channel_buffer, sizeof(struct rc_channels_msg), WEBSOCKET_OP_BINARY);
			}
		}

		// ignore messages not from the autopilot (e.g. discard GCS messages)
		if(msg->compid != MAV_COMP_ID_AUTOPILOT1) continue;

		// handle mavlink error messages
		if (msg->msgid == MAVLINK_MSG_ID_STATUSTEXT){
			char status_text[128];
			char *text = (char *)&_MAV_PAYLOAD(msg)[1]; // pointer to text in the mavlink msg
			memset(status_text, 0, 128);
			status_text[0] = STATUS_TEXT_INDEX;
			snprintf(&status_text[1], 127, "%s", text);

			struct mg_connection *c;
			for (c = mgr_.conns; c != NULL; c = c->next){
				if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
					continue;
				}
				// TODO check if we can send strlen+2
				mg_ws_send(c, status_text, 128, WEBSOCKET_OP_BINARY);
			}
		}
		else if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT){

			bool armed = (mavlink_msg_heartbeat_get_base_mode(msg) & MAV_MODE_FLAG_SAFETY_ARMED);
			uint8_t main_mode = (mavlink_msg_heartbeat_get_custom_mode(msg) & 0x00FF0000) >> 16;
			uint32_t sub_mode = (mavlink_msg_heartbeat_get_custom_mode(msg) & 0xFF000000) >> 24;

			curr_msg.armed = armed;
			curr_msg.main_mode = main_mode;
			curr_msg.sub_mode = sub_mode;
			should_send = 1;
		}
		else if (msg->msgid == MAVLINK_MSG_ID_GPS_RAW_INT){
			curr_msg.num_sats = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
			should_send = 1;
		} else if (msg->msgid == MAVLINK_MSG_ID_GPS_STATUS) {
			uint8_t satellite_prn[20];
			uint8_t satellite_snr[20];

			mavlink_msg_gps_status_get_satellite_prn(msg, satellite_prn);
			mavlink_msg_gps_status_get_satellite_snr(msg, satellite_snr);

			send_satellite_info(satellite_prn, satellite_snr);
		}
		else if (msg->msgid == MAVLINK_MSG_ID_SYS_STATUS){
			curr_msg.batt_remaining = mavlink_msg_sys_status_get_battery_remaining(msg);
			should_send = 1;
		} else if (msg->msgid == MAVLINK_MSG_ID_PARAM_VALUE) { 
			// Only works with int parameters right now

			char buffer[PX4_MAX_PARAM_LENGTH];
			mavlink_msg_param_value_get_param_id(msg, buffer);

			memset(curr_param_msg.buffer, 0, PX4_MAX_PARAM_LENGTH); // inits the buffer back to all 0s before copyinig the name
			memcpy(curr_param_msg.buffer, buffer, PX4_MAX_PARAM_LENGTH);

			union {
				int32_t integer;
				float floating;
			} data;

			float val_float = mavlink_msg_param_value_get_param_value(msg);
			memcpy(&data.integer, &val_float, 4); // have to copy the raw bytes rather just float to int conversion

			uint8_t param_type = mavlink_msg_param_value_get_param_type(msg);

			if (param_type == MAV_PARAM_TYPE_INT32) {
				curr_param_msg.param_val = data.integer;
			} else {
				curr_param_msg.param_val = data.floating;
			}

			struct mg_connection *c;
			for (c = mgr_.conns; c != NULL; c = c->next){
				if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
					continue;
				}

				char param_msg_buffer[sizeof(struct param_msg)];
				memcpy(param_msg_buffer, &curr_param_msg, sizeof(struct param_msg));

				while (c->send.len > sizeof(struct param_msg)) // waits for connection to clear before sending the param buffer
				{
					usleep(50);
				}

				mg_ws_send(c, param_msg_buffer, sizeof(struct param_msg), WEBSOCKET_OP_BINARY);
			}
		}
		else{
			continue;
		}
	}

	if(should_send){
		struct mg_connection *c;
		for (c = mgr_.conns; c != NULL; c = c->next){
			if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
				continue;
			}
			mg_ws_send(c, (char *)&curr_msg, sizeof(curr_msg), WEBSOCKET_OP_BINARY);
		}
	}
#endif // ENABLE_MAVLINK_SUPPORT
	return;
}

void HeaderManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
#ifdef ENABLE_MAVLINK_SUPPORT // mavlink is optional but necessary for battery/gps/flight data

	if (c->is_closing)
	{
		// Remove this connection from our connection ids
		ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());

		// Close the pipe if no longer sending to any websockets
		if (ws_info.connection_ids.empty())
		{
			ws_info.connected = false;

			// no need to close and reopen the pipe every time the page refreshes
			// pipe_client_close(MAVLINK_PIPE_CH);
		}

		return;
	}

	if (!ws_info.connected)
	{
		ws_info.connected = true;

		if(!pipe_client_is_connected(MAVLINK_PIPE_CH)){
			pipe_client_set_connect_cb(MAVLINK_PIPE_CH, _onboard_connect_cb, NULL);
			pipe_client_set_disconnect_cb(MAVLINK_PIPE_CH, _onboard_connect_cb, NULL);
			pipe_client_set_simple_helper_cb(MAVLINK_PIPE_CH, FlightInfoCallback, c);
			pipe_client_open(MAVLINK_PIPE_CH, MAVLINK_ONBOARD_PIPE, PROCESS_NAME,
									CLIENT_FLAG_EN_SIMPLE_HELPER,\
									MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE);

			printf("opening mavlink connection\n");
			start_fake_gcs();
		}

	}
	

	ws_info.connection_ids.push_back(c->id);
#endif // ENABLE_MAVLINK_SUPPORT
}

void MapperManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
	const char* adbCommand = "ls /usr/bin/voxl-mapper > /dev/null 2>&1";
	int result = std::system(adbCommand);
    if (result == 0) {
        mg_ws_send(c, "FOUND", 5, WEBSOCKET_OP_TEXT);
    } else {
        mg_ws_send(c, "NOT_FOUND", 9, WEBSOCKET_OP_TEXT);
    }
}
