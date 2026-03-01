#include "actuator_test.h"
#include <unistd.h>
#include <pthread.h>
#include <modal_pipe_client.h>
#include <modal_start_stop.h>
#include <c_library_v2/common/mavlink.h>
#include "cmd_group_manager.h"
#include "header_manager.h"
#include "pipe_channels.h"

pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;

void ActuatorTestCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (c == nullptr) {
        fprintf(stderr, "Error reading from Actuator Test callback\n");
        return;
    }

    pthread_t threadId;
    pthread_create(&threadId, NULL, run, NULL);
}


static void *run(void *arg) {
    int prevMotorVal[4] = {0, 0, 0, 0};


    while (true) {

        for (size_t motorNum = 0; motorNum < 4; motorNum++) { 

            if (motorVals[motorNum] <= 1 && prevMotorVal[motorNum] <= 1) {
                prevMotorVal[motorNum] = motorVals[motorNum];
                usleep(50000);
                continue;
            } 

            prevMotorVal[motorNum] = motorVals[motorNum];

            mavlink_message_t msg;
            mavlink_msg_command_long_pack(1, \
                    MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 1, MAV_COMP_ID_AUTOPILOT1, \
                    MAV_CMD_ACTUATOR_TEST, 1, static_cast<float>(motorVals[motorNum]) / 100, 1, 0, 0, static_cast<float>(1101 + motorNum), 0, 0);

            _send_mavlink_msg_to_ap(&msg);
        }
        usleep(250000);
    }
}

void send_motor_val(int motorNum, float motorVal) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(1, \
            MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 1, MAV_COMP_ID_AUTOPILOT1, \
            MAV_CMD_ACTUATOR_TEST, 1, motorVal, 8.0, 0, 0, static_cast<float>(1101 + motorNum), 0, 0);

    _send_mavlink_msg_to_ap(&msg);
}