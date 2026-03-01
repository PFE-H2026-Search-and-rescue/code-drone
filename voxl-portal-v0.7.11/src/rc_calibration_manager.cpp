#include "rc_calibration_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>
#include <turbojpeg.h>

#include "cmd_group_manager.h"
#include "manager_utils.h"
#include "page_manager.h"

#include <algorithm>
#include <vector>

rc_channels_msg curr_rc_channels_msg;
uint16_t curr_rc_channels_array[18];

extern struct mg_mgr mgr_;
static websocket_info ws_info;

static int currentStep;
static bool cancelCalibration = false;
static std::vector<ChannelInfo> channelInfoArray = std::vector<ChannelInfo>(18);

static int8_t throttleCh;
static int8_t yawCh;
static int8_t rollCh;
static int8_t pitchCh;
static int8_t switchCh1;
static int8_t switchCh2;

void RCCalibrationCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    if (c == nullptr) {
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
            pipe_client_close(ws_info.ch);
        }
        return;
    }

    if (!ws_info.connected)
    {
        ws_info.connected = true;
    }

    ws_info.connection_ids.push_back(c->id);
}

void send_rc_calibration_msg(int step) {
    struct rc_calibration_msg rc_cal_msg;
    rc_cal_msg.msg_index = 3;
    rc_cal_msg.step = step;


    if (!ws_info.connected) {
        return;
    }

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&rc_cal_msg, sizeof(struct rc_calibration_msg), WEBSOCKET_OP_BINARY);
    }
}

void send_calibration_complete_msg() {
    struct rc_calibration_complete_msg rc_cal_complete_msg;
    
    if (throttleCh < 0 || yawCh < 0 || rollCh < 0 || pitchCh < 0) {
        return;
    }

    rc_cal_complete_msg.msg_index = 4;
    rc_cal_complete_msg.throttleInfo = channelInfoArray[throttleCh];
    rc_cal_complete_msg.throttleInfo.channel = throttleCh;

    rc_cal_complete_msg.yawInfo = channelInfoArray[yawCh];
    rc_cal_complete_msg.yawInfo.channel = yawCh;

    rc_cal_complete_msg.rollInfo = channelInfoArray[rollCh];
    rc_cal_complete_msg.rollInfo.channel = rollCh;

    rc_cal_complete_msg.pitchInfo = channelInfoArray[pitchCh];
    rc_cal_complete_msg.pitchInfo.channel = pitchCh;

    if (!ws_info.connected) {
        return;
    }

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&rc_cal_complete_msg, sizeof(struct rc_calibration_complete_msg), WEBSOCKET_OP_BINARY);
    }
}


void run_rc_calibration() {
    pthread_t thread_id;
    pthread_create(&thread_id, nullptr, run_rc_calibration_helper, nullptr);
}


void *run_rc_calibration_helper(void* args) {
    // Runs steps 1 through eleven collecting the min, max for throttle, yaw, roll, pitch
    handleStep(1);
}

void cancel_calibration() {
    cancelCalibration = true;
    currentStep = 0;
}

void handleStep(int currentStep) {

    switch (currentStep) {

        case 0:
            handleStep0();
            break;

        case 1:
            handleStep1();

        case 2:
            handleStep2();

        case 3:
            handleStep3();

        case 4:
            handleStep4();
        
        case 5:
            handleStep5();

        case 6:
            handleStep6();

        case 7:
            handleStep7();

        case 8:
            handleStep8();

        case 9:
            handleStep9();

        case 10:
            handleStep10();

        case 11:
            handleStep11();
            break;

        case 12:
            handleStep12();
            break;

        default:
            break;
    }
}

void handleStep0() {
    send_rc_calibration_msg(0);

    // saves the current values as the trim
    channelInfoArray[0].rcTrim = curr_rc_channels_array[0];
    channelInfoArray[1].rcTrim = curr_rc_channels_array[1];
    channelInfoArray[2].rcTrim = curr_rc_channels_array[2];
    channelInfoArray[3].rcTrim = curr_rc_channels_array[3];
    channelInfoArray[4].rcTrim = curr_rc_channels_array[4];
    channelInfoArray[5].rcTrim = curr_rc_channels_array[5];
    channelInfoArray[6].rcTrim = curr_rc_channels_array[6];
    channelInfoArray[7].rcTrim = curr_rc_channels_array[7];
    channelInfoArray[8].rcTrim = curr_rc_channels_array[8];
    channelInfoArray[9].rcTrim = curr_rc_channels_array[9];
    channelInfoArray[10].rcTrim = curr_rc_channels_array[10];
    channelInfoArray[11].rcTrim = curr_rc_channels_array[11];
    channelInfoArray[12].rcTrim = curr_rc_channels_array[12];
    channelInfoArray[13].rcTrim = curr_rc_channels_array[13];
    channelInfoArray[14].rcTrim = curr_rc_channels_array[14];
    channelInfoArray[15].rcTrim = curr_rc_channels_array[15];
    channelInfoArray[16].rcTrim = curr_rc_channels_array[16];
    channelInfoArray[17].rcTrim = curr_rc_channels_array[17];
}

void handleStep1() {
    send_rc_calibration_msg(1);
    cancelCalibration = false;

    throttleCh = inputStickDetect();
    int32_t maxVal = stickSettleComplete(throttleCh, true);

    channelInfoArray[throttleCh].rcMax = maxVal;
    channelInfoArray[throttleCh].reversed = checkReversed(throttleCh);
}

void handleStep2() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(2);

    if (throttleCh < 0) {
        return;
    }

    int32_t minVal = stickSettleComplete(throttleCh, true);
    channelInfoArray[throttleCh].rcMin = minVal;
}

void handleStep3() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(3);

    std::vector<int8_t> channels_to_avoid = {throttleCh};

    yawCh = inputStickDetect(channels_to_avoid);
    int32_t minVal = stickSettleComplete(yawCh, true);
    channelInfoArray[yawCh].rcMin = minVal;
}

void handleStep4() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(4);

    if (yawCh < 0) {
        return;
    }

    int32_t maxVal = stickSettleComplete(yawCh, true);
    channelInfoArray[yawCh].rcMax = maxVal;
    channelInfoArray[yawCh].reversed = checkReversed(yawCh);
}

void handleStep5() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(5);

    std::vector<int8_t> channels_to_avoid = {throttleCh, yawCh};

    rollCh = inputStickDetect(channels_to_avoid);
    int32_t minVal = stickSettleComplete(rollCh, true);
    channelInfoArray[rollCh].rcMin = minVal;
}

void handleStep6() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(6);

    if (rollCh < 0) {
        return;
    }

    int32_t maxVal = stickSettleComplete(rollCh, true);
    channelInfoArray[rollCh].rcMax = maxVal;
    channelInfoArray[rollCh].reversed = checkReversed(rollCh);
}

void handleStep7() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(7);

    std::vector<int8_t> channels_to_avoid = {throttleCh, yawCh, rollCh};

    pitchCh = inputStickDetect(channels_to_avoid);
    int32_t minVal = stickSettleComplete(pitchCh, true);
    channelInfoArray[pitchCh].rcMin = minVal;
}

void handleStep8() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(8);

    if (pitchCh < 0) {
        return;
    }

    int32_t maxVal = stickSettleComplete(pitchCh, true);
    channelInfoArray[pitchCh].rcMax = maxVal;
    channelInfoArray[pitchCh].reversed = checkReversed(pitchCh);
}

void handleStep9() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(9);

    pthread_t thread_id[4];
    pthread_create(&thread_id[0], nullptr, wait_center, new int(throttleCh));
    pthread_create(&thread_id[1], nullptr, wait_center, new int(pitchCh));
    pthread_create(&thread_id[2], nullptr, wait_center, new int(yawCh));
    pthread_create(&thread_id[3], nullptr, wait_center, new int(rollCh));

    for (size_t i = 0; i < 4; i++) {
        pthread_join(thread_id[i], nullptr);
    }
}

void *wait_center(void* args) {
    int ch = *static_cast<int*>(args);

    stickSettleComplete(ch, false);
}

void handleStep10() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(10);

    std::vector<int8_t> channels_to_avoid = {throttleCh, yawCh, rollCh, yawCh};

    switchCh1 = inputStickDetect(channels_to_avoid, true);

    std::vector<int32_t> switch1Vals = switchSettleComplete(switchCh1);
    int32_t minVal1 = *std::min_element(switch1Vals.begin(), switch1Vals.end());
    int32_t maxVal1 = *std::max_element(switch1Vals.begin(), switch1Vals.end());
    channelInfoArray[switchCh1].rcMin = minVal1;
    channelInfoArray[switchCh1].rcMax = maxVal1;
    channelInfoArray[switchCh1].rcTrim = (maxVal1 + minVal1) / 2;

    channels_to_avoid.push_back(switchCh1);

    switchCh2 = inputStickDetect(channels_to_avoid, true);

    std::vector<int32_t> switch2Vals = switchSettleComplete(switchCh2);
    int32_t minVal2 = *std::min_element(switch2Vals.begin(), switch2Vals.end());
    int32_t maxVal2 = *std::max_element(switch2Vals.begin(), switch2Vals.end());
    channelInfoArray[switchCh2].rcMin = minVal2;
    channelInfoArray[switchCh2].rcMax = maxVal2;
    channelInfoArray[switchCh2].rcTrim = (maxVal2 + minVal2) / 2;

}

void handleStep11() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(11);
    send_calibration_complete_msg();
    calibrationFinished();
}

void handleStep12() {
    if (cancelCalibration) {
        return;
    }

    send_rc_calibration_msg(12);

    saveMin(throttleCh);
    saveTrim(throttleCh);
    saveMax(throttleCh);
    saveReversed(throttleCh);
    saveChannelMapping(throttleCh, "THROTTLE");

    saveMin(yawCh);
    saveTrim(yawCh);
    saveMax(yawCh);
    saveReversed(yawCh);
    saveChannelMapping(yawCh, "YAW");

    saveMin(rollCh);
    saveTrim(rollCh);
    saveMax(rollCh);
    saveReversed(rollCh);
    saveChannelMapping(rollCh, "ROLL");

    saveMin(pitchCh);
    saveTrim(pitchCh);
    saveMax(pitchCh);
    saveReversed(pitchCh);
    saveChannelMapping(pitchCh, "PITCH");

    saveMin(switchCh1);
    saveTrim(switchCh1);
    saveMax(switchCh1);

    saveMin(switchCh2);
    saveTrim(switchCh2);
    saveMax(switchCh2);
}

void saveMin(int8_t ch) {
    if (ch < 0) {
        return;
    }

    float min = channelInfoArray[ch].rcMin;
    std::string param_id_string = std::string("RC") + std::to_string(ch + 1) + std::string("_MIN");
    char* param_id_buffer = const_cast<char*>(param_id_string.data()); // +1, 0 indexed -> 1 indexed
    setParam(param_id_buffer, min, param_id_string.length());
}


void saveTrim(int8_t ch) {
    if (ch < 0) {
        return;
    }

    float trim = channelInfoArray[ch].rcTrim;
    std::string param_id_string = std::string("RC") + std::to_string(ch + 1) + std::string("_TRIM");
    char* param_id_buffer = const_cast<char*>(param_id_string.data()); // +1, 0 indexed -> 1 indexed
    setParam(param_id_buffer, trim, param_id_string.length());
}

void saveMax(int8_t ch) {
    if (ch < 0) {
        return;
    }

    float max = channelInfoArray[ch].rcMax;
    std::string param_id_string = std::string("RC") + std::to_string(ch + 1) + std::string("_MAX");
    char* param_id_buffer = const_cast<char*>(param_id_string.data()); // +1, 0 indexed -> 1 indexed
    setParam(param_id_buffer, max, param_id_string.length());
}

void saveReversed(int8_t ch) {
    if (ch < 0) {
        return;
    }

    float reversed = channelInfoArray[ch].reversed ? -1.0 : 1.0;
    std::string param_id_string = std::string("RC") + std::to_string(ch + 1) + std::string("_REV");
    char* param_id_buffer = const_cast<char*>(param_id_string.data()); // +1, 0 indexed -> 1 indexed
    setParam(param_id_buffer, reversed, param_id_string.length());
}

void saveChannelMapping(int32_t ch, std::string ch_name) {
    std::string param_id_string = std::string("RC_MAP_") + ch_name;
    char* param_id_buffer = const_cast<char*>(param_id_string.data());
    setParam(param_id_buffer, ch + 1, param_id_string.length()); // +1, 0 indexed -> 1 indexed
}


int8_t inputStickDetect(const std::vector<int8_t>& channels_to_avoid, bool sticks) {
    int64_t startTime = VCU_time_monotonic_ns();
    int32_t diff;

    while (true) { // wait until either we find the channel being moved or the callibration gets cancelled

        if (sticks && (VCU_time_monotonic_ns() - startTime) > ((long) SWITCH_TIMEOUT_MSECS * 1000000)) {
            return -1; // timeout looking for switch channel
        }

        if (cancelCalibration) {
            return -1;
        }

        for (size_t i = 0; i < 18; i++) {
            if (std::find(channels_to_avoid.begin(), channels_to_avoid.end(), i) != channels_to_avoid.end()) {
                continue;
            }

            diff = channelInfoArray[i].rcTrim - curr_rc_channels_array[i];
            if (abs(diff) < RC_CAL_MOVE_DELTA) {
                continue; // need to keep searching for the channel being moved
            }

            // got here, found the channel being moved!
            return i;
        }
    }

    return -1; // calibration cancelled
}

int32_t stickSettleComplete(int ch, bool min_or_max) {
    int32_t prevVal = curr_rc_channels_array[ch];
    int64_t startTime = VCU_time_monotonic_ns();
    int32_t diff;

    if (startTime < 0) {
        return -1;
    }

    while ((abs(curr_rc_channels_array[ch] - prevVal) > STICK_DETECT_SETTLE_MSECS) || 
        ((VCU_time_monotonic_ns() - startTime) < ((long) STICK_DETECT_SETTLE_MSECS * 1000000))) {

            if (cancelCalibration) {
                return -1; // received cancel signal
            }

            if (abs(curr_rc_channels_array[ch] - prevVal) > STICK_DETECT_SETTLE_MSECS) {
                startTime = VCU_time_monotonic_ns();
                prevVal = curr_rc_channels_array[ch];
            }

            diff = channelInfoArray[ch].rcTrim - curr_rc_channels_array[ch];

            if (min_or_max && (abs(diff) < RC_CAL_MOVE_DELTA)) { // Need to move the stick to min or max
                startTime = VCU_time_monotonic_ns();
                prevVal = curr_rc_channels_array[ch];
            } else  if (!min_or_max && abs(diff) > RC_CAL_MOVE_DELTA) { // Need to keep the sticks centered
                startTime = VCU_time_monotonic_ns();
                prevVal = curr_rc_channels_array[ch];
            } 
    }

    return curr_rc_channels_array[ch];
}

std::vector<int32_t> switchSettleComplete(int ch) {
    int32_t prevVal = curr_rc_channels_array[ch];
    int64_t startTime = VCU_time_monotonic_ns();
    std::vector<int32_t> vals = {prevVal};

    if (startTime < 0) {
        return std::vector<int32_t>();
    }

    while ((VCU_time_monotonic_ns() - startTime) < ((long) SWITCH_DETECT_MSECS * 1000000)) {
            if (std::find(vals.begin(), vals.end(), curr_rc_channels_array[ch]) == vals.end()) {
                vals.push_back(curr_rc_channels_array[ch]);
        }
    }

    return vals;
}

bool checkReversed(int8_t ch) {
    if (channelInfoArray[ch].rcMax < channelInfoArray[ch].rcTrim) {
        return true;
    }

    return false;
}
