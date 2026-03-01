#include "spoof_gcs.h"
#include <unistd.h>
#include <pthread.h>
#include <modal_pipe_client.h>
#include <modal_start_stop.h>
#include <c_library_v2/common/mavlink.h>
#include "header_manager.h"
#include "pipe_channels.h"

static int running = 0;

void start_fake_gcs()
{
    if(running) return;

    pthread_t thread_id;
    pthread_create(&thread_id, NULL, run, NULL);
    running = 1;
}

static void *run(void *arg) {

     mavlink_message_t* message = new mavlink_message_t;

    const uint8_t system_id = 255;
    const uint8_t base_mode = 0xc0;
    const uint8_t custom_mode = 0;
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_MISSIONPLANNER,
        MAVLINK_COMM_0,
        message,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        base_mode,
        custom_mode,
        MAV_STATE_ACTIVE);

    while(1) {
        pipe_client_send_control_cmd_bytes(MAVLINK_PIPE_CH, message, sizeof(mavlink_message_t));
        usleep(250000);
    }

    return NULL;
}
