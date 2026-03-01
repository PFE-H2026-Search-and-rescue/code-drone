let calibrate_baro_button = document.getElementById('calibrate-baro-button')
let tc_b0_enable = document.getElementById('tc-b-enable')
let tc_b0_id = document.getElementById('tc-b0-id')
let tc_b0_tref = document.getElementById('tc-b0-tref')
let tc_b0_tmin = document.getElementById('tc-b0-tmin')
let tc_b0_tmax = document.getElementById('tc-b0-tmax')
let tc_b0_x0 = document.getElementById('tc-b0-x0')
let tc_b0_x1 = document.getElementById('tc-b0-x1')
let tc_b0_x2 = document.getElementById('tc-b0-x2')
let tc_b0_x3 = document.getElementById('tc-b0-x3')
let tc_b0_x4 = document.getElementById('tc-b0-x4')
let tc_b0_x5 = document.getElementById('tc-b0-x5')
let baro_progress = document.getElementById('baro-progress')
let too_hot_container = document.getElementById('too-hot-container')
let too_hot_temp = document.getElementById('too-hot-temp')
let continue_calibration_button = document.getElementById('continue-calibration-button')
let too_hot_instruction = document.getElementById('too-hot-instruction')

let BARO_PARAM_INDEX = 2
let BARO_PROGESS_INDEX = 3;
let SAVE_PARAMS_SUCCESS_INDEX = 4;
let MAX_BARO_PROGRESS = 1250.0;

let CALIBRATION_STAGE_TOO_HOT = 1
let CALIBRATION_STAGE_HIT_CONTINUE = 2

continue_calibration_button.disabled = true;

function onCalibrateBaroButtonClicked() {
    request_baro_calibration()
    calibrate_baro_button.disabled = true;
}

function onCalibrationBaroSaveParamsButtonClicked() {
    request_save_baro_params()
}

function request_baro_calibration() {
    fetch('/_cmd/baro_calibration_start').then(response => {
    })
}

function request_save_baro_params() {
    fetch('/_cmd/baro_calibration_save_params').then(response => {
    })
}

function onCalibrateBaroParams(baro_param_msg) {
    baro_progress.innerHTML = 'Progress: 100%'

    tc_b0_enable.innerHTML = baro_param_msg.param_vals[0].toFixed(0)
    tc_b0_id.innerHTML = baro_param_msg.param_vals[1].toFixed(0)
    tc_b0_tref.innerHTML = baro_param_msg.param_vals[2].toFixed(3)
    tc_b0_tmin.innerHTML = baro_param_msg.param_vals[3].toFixed(3)
    tc_b0_tmax.innerHTML = baro_param_msg.param_vals[4].toFixed(3)
    tc_b0_x0.innerHTML = baro_param_msg.param_vals[5].toFixed(3)
    tc_b0_x1.innerHTML = baro_param_msg.param_vals[6].toFixed(3)
    tc_b0_x2.innerHTML = baro_param_msg.param_vals[7].toFixed(3)
    tc_b0_x3.innerHTML = baro_param_msg.param_vals[8].toFixed(3)
    tc_b0_x4.innerHTML = baro_param_msg.param_vals[9].toFixed(3)
    tc_b0_x5.innerHTML = baro_param_msg.param_vals[10].toFixed(3)
}

function onCalibrateBaroProgress(baro_progress_msg) {

    if (baro_progress_msg.calibration_stage == CALIBRATION_STAGE_HIT_CONTINUE) {
        too_hot_temp.innerHTML = "Current Temp: 35°C"
        too_hot_instruction.innerHTML = "Reached 35°C, turn fan off and hit continue to start the calibration"
        continue_calibration_button.disabled = false;
        return;
    } else if (baro_progress_msg.calibration_stage == CALIBRATION_STAGE_TOO_HOT) {
        too_hot_temp.innerHTML = "Current Temp: " + baro_progress_msg.pos / 10 + "°C"
        too_hot_container.style.display = "flex"
    } else {
        too_hot_container.style.display = "none"
        if (Math.abs(baro_progress_msg.pos - MAX_BARO_PROGRESS) < 10) {
            calibrate_baro_button.disabled = false;
            baro_progress.innerHTML = 'Progress: 100%'
        } else {
            baro_progress.innerHTML = 'Progress: ' + ((baro_progress_msg.pos / MAX_BARO_PROGRESS) * 100).toFixed(2) + '%'
        }
    }
}

function onSaveParamsSuccess(save_params_success_msg) {
    console.log(save_params_success_msg.success)

    if (save_params_success_msg.success == 1) {
        baro_progress.innerHTML = 'Saved parameters successfully'
    }
}

function onContinueButtonClicked() {
    too_hot_container.style.display = "none"
    baro_progress.innerHTML = 'Calibration in progress make sure fan is off'

    fetch('/_cmd/baro_calibration_continue').then(response => {
    })
}


var url_base = get_url_base();
var url_port = get_url_port();


var baro_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/baro");
baro_ws.binaryType = "arraybuffer";

baro_ws.onmessage = function (evt) {
    var msg = evt.data;

    var parser = new jParser(msg, {
        msg_type: {
            index: 'uint8',
        },
        baro_calibration_params: {
            param_vals: ['array', 'float32', 11]
        },
        baro_calibration_progress: {
            pos: 'uint16',
            calibration_stage: 'uint8'
        },
        save_params_success: {
            success: 'uint8'
        }
    });

    var msg_type = parser.parse('msg_type');

    switch(msg_type.index) {

        case (BARO_PARAM_INDEX):
            var baro_param_msg = parser.parse('baro_calibration_params')
            onCalibrateBaroParams(baro_param_msg)
            break

        case (BARO_PROGESS_INDEX):
            var baro_progress_msg = parser.parse('baro_calibration_progress')
            onCalibrateBaroProgress(baro_progress_msg)
            break
        
        case (SAVE_PARAMS_SUCCESS_INDEX):
            var save_params_success_msg = parser.parse('save_params_success')
            onSaveParamsSuccess(save_params_success_msg)
            break 
    }
}