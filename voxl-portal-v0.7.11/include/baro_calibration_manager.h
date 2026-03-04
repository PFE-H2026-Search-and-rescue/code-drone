#ifndef BARO_CALIBRATION_MANAGER_H
#define BARO_CALIBRATION_MANAGER_H

#include <stdint.h>
#include <fcntl.h> 
#include <sys/stat.h>
#include <sys/wait.h>
#include <fstream>

#include <algorithm>
#include <vector>

#define BaroCalibrationClaimString "/baro"

#define CALIBRATION_STAGE_TOO_HOT 1
#define CALIBRATION_STAGE_HIT_CONTINUE 2

typedef struct baro_calibration_params {
    uint8_t msg_index;
    float param_vals[11];
} __attribute__((packed)) baro_calibration_params;

typedef struct baro_calibration_progress {
    uint8_t msg_index;
    uint16_t pos;
    uint8_t calibration_stage;
} __attribute__((packed)) baro_calibration_progress;

typedef struct save_params_success {
    uint8_t msg_index;
    uint8_t success;
} __attribute__((packed)) save_params_success;

static uint8_t autopilot_monitor_get_sysid(void);

void BaroCalibrationCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void run_baro_calibration();
void continue_baro_calibration();
static void *run_baro_calibration_helper(void *arg);
void process_baro_log();
std::vector<std::string> split_line(std::string input);
void get_baro_params();

void send_baro_calibration_stage(uint8_t calibration_stage, uint16_t pos=0);
void send_baro_calibration_params(struct baro_calibration_params baroCalParamsMsg);
void send_baro_calibration_progress(uint16_t pos);
void send_save_params_success();
void save_baro_calibration_params();
void save_baro_calibration_params_helper();
void read_baro_cal_progress(void* args);
void wait_for_continue_command(void* args);

#endif