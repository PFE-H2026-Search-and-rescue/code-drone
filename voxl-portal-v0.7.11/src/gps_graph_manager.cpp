#include "gps_graph_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>
#include <turbojpeg.h>
#include <iostream>

#include "cmd_group_manager.h"
#include "manager_utils.h"
#include "page_manager.h"

#define MSG_INDEX_SATELLITE_INFO  1

extern struct mg_mgr mgr_;
static websocket_info ws_info;

static uint8_t current_sysid = 1;

static uint8_t autopilot_monitor_get_sysid(void)
{
	return current_sysid;
}

void GpsGraphManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (c == nullptr) {
        fprintf(stderr, "Error reading from barometer calibration callback\n");
        return;
    }

    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());

        // Close the pipe if no longer sending to any websockets
        if (ws_info.connection_ids.empty())
        {
            ws_info.connected = false;
        }
        return;
    }

    if (!ws_info.connected)
    {
        ws_info.connected = true;
    }

    ws_info.connection_ids.push_back(c->id);
}


void send_satellite_info(uint8_t* satellite_prn, uint8_t* satellite_snr) {
    if (satellite_prn == nullptr || satellite_snr == nullptr) {
        fprintf(stderr, "Error: received empty data sending satellite info");
    }

    struct satellite_info satelliteInfoMsg;
    satelliteInfoMsg.msg_index = MSG_INDEX_SATELLITE_INFO;

    memcpy(satelliteInfoMsg.satellite_prn, satellite_prn, 20);
    memcpy(satelliteInfoMsg.satellite_snr, satellite_snr, 20);

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&satelliteInfoMsg, sizeof(struct satellite_info), WEBSOCKET_OP_BINARY);
    }
}
