#pragma once

#include <stdint.h>
#include <fcntl.h> 
#include <sys/stat.h>
#include <vector>
#include <string>
#include <algorithm>

#define RC_CAL_MOVE_DELTA              300
#define RC_CAL_SETTLE_DELTA            20
#define RC_CAL_MIN_DELTA               100
#define STICK_DETECT_SETTLE_MSECS      3000
#define SWITCH_DETECT_MSECS            3000
#define SWITCH_TIMEOUT_MSECS           5000
#define RCCalibrationClaimString       "/rc"


typedef struct rc_channels_msg
{
	uint8_t msg_index;
	uint8_t chancount;
	uint16_t chan1_raw;
	uint16_t chan2_raw;
	uint16_t chan3_raw;
	uint16_t chan4_raw;
	uint16_t chan5_raw;
	uint16_t chan6_raw;
	uint16_t chan7_raw;
	uint16_t chan8_raw;
	uint16_t chan9_raw;
	uint16_t chan10_raw;
	uint16_t chan11_raw;
	uint16_t chan12_raw;
	uint16_t chan13_raw;
	uint16_t chan14_raw;
	uint16_t chan15_raw;
	uint16_t chan16_raw;
	uint16_t chan17_raw;
	uint16_t chan18_raw;
	uint8_t rssi;
} __attribute__((packed)) rc_channels_msg;

typedef struct rc_calibration_msg
{
    uint8_t msg_index;
    uint8_t step;
} __attribute__((packed)) rc_calibration_msg;

typedef struct ChannelInfo {
	int8_t channel = -1;
    bool reversed;
    int32_t rcMin;
    int32_t rcMax;
    int32_t rcTrim;
} __attribute__((packed)) ChannelInfo;

typedef struct rc_calibration_complete_msg
{
    uint8_t msg_index;
	struct ChannelInfo throttleInfo;
	struct ChannelInfo yawInfo;
	struct ChannelInfo rollInfo;
	struct ChannelInfo pitchInfo;
} __attribute__((packed)) rc_calibration_complete_msg;

void RCCalibrationCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void run_rc_calibration();
void *run_rc_calibration_helper(void* args);
void send_rc_calibration_msg(int step);
void cancel_calibration();

void handleStep(int currentStep);
void handleStep0();
void handleStep1();
void handleStep2();
void handleStep3();
void handleStep4();
void handleStep5();
void handleStep6();
void handleStep7();
void handleStep8();
void handleStep9();
void handleStep10();
void handleStep11();
void handleStep12();

int8_t inputStickDetect(const std::vector<int8_t>& channels_to_avoid = std::vector<int8_t>(), bool sticks = false);
int32_t stickSettleComplete(int ch, bool min_or_max);
std::vector<int32_t> switchSettleComplete(int ch);
bool checkReversed(int8_t ch);
void *wait_center(void* args);

void saveMin(int8_t ch);
void saveTrim(int8_t ch);
void saveMax(int8_t ch);
void saveReversed(int8_t ch);
void saveChannelMapping(int32_t ch, std::string ch_name);
