const rollBall = document.getElementById('rollBall')
const pitchBall = document.getElementById('pitchBall')
const yawBall = document.getElementById('yawBall')
const throttleBall = document.getElementById('throttleBall')

const ball1 = document.getElementById('ball1')
const ball2 = document.getElementById('ball2')
const ball3 = document.getElementById('ball3')
const ball4 = document.getElementById('ball4')
const ball5 = document.getElementById('ball5')
const ball6 = document.getElementById('ball6')
const ball7 = document.getElementById('ball7')
const ball8 = document.getElementById('ball8')
const ball9 = document.getElementById('ball9')
const ball10 = document.getElementById('ball10')
const ball11 = document.getElementById('ball11')
const ball12 = document.getElementById('ball12')
const ball13 = document.getElementById('ball13')
const ball14 = document.getElementById('ball14')
const ball15 = document.getElementById('ball15')
const ball16 = document.getElementById('ball16')
const ball17 = document.getElementById('ball17')
const ball18 = document.getElementById('ball18')

const calibrateRCButton = document.getElementById('calibrateRCButton')
const cancelCalButton = document.getElementById('cancelCalButton')
const nextButton = document.getElementById('nextButton')
const displayText = document.getElementById('displayText')
const calibration_img = document.getElementById('calibration_img')
let mode = document.getElementById('mode-select')

cancelCalButton.disabled = true;
let currentStep = 0;

const setupRCMessage = "Center the Throttle stick as shown in diagram.\nReset all transmitter trims to center.\n\nPlease ensure all motor power is disconnected from the vehicle.\n\nClick Next to continue"
const msgThrottleUp = "Move the Throttle stick all the way up and hold it there..."
const msgThrottleDown = "Move the Throttle stick all the way down and leave it there..."
const msgYawLeft = "Move the Yaw stick all the way to the left and hold it there..."
const msgYawRight = "Move the Yaw stick all the way to the right and hold it there..."
const msgRollLeft = "Move the Roll stick all the way to the left and hold it there..."
const msgRollRight = "Move the Roll stick all the way to the right and hold it there..."
const msgPitchDown = "Move the Pitch stick all the way down and hold it there..."
const msgPitchUp = "Move the Pitch stick all the way up and hold it there..."
const msgPitchCenter = "Allow the Pitch stick to move back to center..."
const msgSwitchMinMax = "Move all the transmitter switches and/or dials back and forth to their extreme positions."
const msgComplete = "All settings have been captured. Click Next to write the new parameters to your board."

const path_radio_center = "../images/radioCenter.png"
const path_radio_switch_min_max = "../images/radioSwitchMinMax.png"

const path_radio_right_up = "../images/radioRightUp.png"
const path_radio_right_down = "../images/radioRightDown.png"
const path_radio_right_left = "../images/radioRightLeft.png"
const path_radio_right_right = "../images/radioRightRight.png"

const path_radio_left_up = "../images/radioLeftUp.png"
const path_radio_left_down = "../images/radioLeftDown.png"
const path_radio_left_left = "../images/radioLeftLeft.png"
const path_radio_left_right = "../images/radioLeftRight.png"

let rcRawArray = new Array(18).fill(0)
let switchChannelArray = new Array()

let rollCh = -1
let rollDiff = -1
let rollTrim = -1
let rollReversed = false

let pitchCh = -1
let pitchDiff = -1
let pitchTrim = -1
let pitchReversed = false

let yawCh = -1
let yawDiff = -1
let yawTrim = -1
let yawReversed = false

let throttleCh = -1
let throttleDiff = -1
let throttleTrim = -1
let throttleReversed = false

class rcChInfo {
    constructor() {
        this.diff = -1
        this.min = -1
        this.max = -1
        this.trim = -1
        this.reversed = false
    }
}
const rcChInfoArray = Array.from({ length: 18 }, () => new rcChInfo());

const rcMinRegex = /^RC(\d+)_MIN$/
const rcMaxRegex = /^RC(\d+)_MAX$/
const rcTrimRegex = /^RC(\d+)_TRIM$/
const rcRevRegex = /^RC(\d+)_REV$/


let RC_CALIBRATION_INDEX = 3;
let RC_CALIBRATION_COMPLETE_INDEX = 4;

var url_base = get_url_base();
var url_port = get_url_port();


var rc_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/rc");
rc_ws.binaryType = "arraybuffer";

function get_initial_rc_settings() {
    // Sends a request to get all of the flight mode and switch mapping parameters

    request_get_param("RC_MAP_THROTTLE")
    request_get_param("RC_MAP_YAW")
    request_get_param("RC_MAP_ROLL")
    request_get_param("RC_MAP_PITCH")
}

function get_initial_rc_vals() {
    if ((rollCh == -1) || 
        (yawCh == -1) ||
        (rollCh == -1) ||
        (pitchCh == -1)) {
        return; // need to wait for all the channel mappings to come in
    }

    for (let i = 1; i <= 18; i++) {
        request_get_param("RC" + i + "_MIN")
        request_get_param("RC" + i + "_MAX")
        request_get_param("RC" + i + "_TRIM")
        request_get_param("RC" + i + "_REV")
    }
}

function request_get_param(param_id) {
    fetch('/_cmd/param_request?param_id=' + param_id).then(response => {
    })
}

function onParamMsg(param_msg) {
    // Updates the gui to show that a parameter changed

    let param_string = ('' + param_msg.buffer)
    let cleaned_param_string = param_string.replace(/[^A-Za-z0-9_]/g, '') // removes anything that's not a A-Z, an underscore, or a digit 

    switch (cleaned_param_string) {

        case ("RC_MAP_THROTTLE"):
            throttleCh = param_msg.param_val -1
            get_initial_rc_vals()
            break

        case ("RC_MAP_YAW"):
            yawCh = param_msg.param_val -1
            get_initial_rc_vals()
            break
            
        case ("RC_MAP_ROLL"):
            rollCh = param_msg.param_val -1
            get_initial_rc_vals()
            break

        case ("RC_MAP_PITCH"):
            pitchCh = param_msg.param_val -1
            get_initial_rc_vals()
            break
                
        default:
            break
    }

    if (rcMinRegex.test(cleaned_param_string)) {
        let match = cleaned_param_string.match(rcMinRegex)
        let ch = parseInt(match[1], 10)
        rcChInfoArray[ch-1].min = param_msg.param_val
    } else if (rcMaxRegex.test(cleaned_param_string)) {
        let match = cleaned_param_string.match(rcMaxRegex)
        let ch = parseInt(match[1], 10)
        rcChInfoArray[ch-1].max = param_msg.param_val
    } else if (rcTrimRegex.test(cleaned_param_string)) {
        let match = cleaned_param_string.match(rcTrimRegex)
        let ch = parseInt(match[1], 10)
        rcChInfoArray[ch-1].trim = param_msg.param_val
    } else if (rcRevRegex.test(cleaned_param_string)) {
        let match = cleaned_param_string.match(rcRevRegex)
        let ch = parseInt(match[1], 10)
        rcChInfoArray[ch-1].rev = param_msg.param_val
    }
}

rc_ws.onmessage = function (evt) {
    var msg = evt.data;

    var parser = new jParser(msg, {
        msg_type: {
            index: 'uint8',
        },
        rc_calibration_msg: {
            step: 'uint8'
        }, 
        rc_calibration_complete_msg: {
            throttleInfo: {
                channel: 'int8',
                reversed: 'uint8',
                rcMin: 'int32',
                rcMax: 'int32',
                rcTrim: 'int32'
            },
            yawInfo: {
                channel: 'int8',
                reversed: 'uint8',
                rcMin: 'int32',
                rcMax: 'int32',
                rcTrim: 'int32'
            },
            rollInfo: {
                channel: 'int8',
                reversed: 'uint8',
                rcMin: 'int32',
                rcMax: 'int32',
                rcTrim: 'int32'
            },
            pitchInfo: {
                channel: 'int8',
                reversed: 'uint8',
                rcMin: 'int32',
                rcMax: 'int32',
                rcTrim: 'int32'
            }
        }
    });

    var msg_type = parser.parse('msg_type');

    switch(msg_type.index) {
        case (RC_CALIBRATION_INDEX):
            var rc_calibration_msg = parser.parse('rc_calibration_msg')
            handleState(rc_calibration_msg.step)
            break;
        case (RC_CALIBRATION_COMPLETE_INDEX):
            var rc_calibration_complete_msg = parser.parse('rc_calibration_complete_msg')
            onRCCalibrationCompleteMsg(rc_calibration_complete_msg)
            break;
    }
}

function moveBall(percentage, ball=ball1) {

    if (percentage < 0 || percentage > 1 || isNaN(percentage)) {
        return;
    }

    // Calculate the left position for the ball
    const newPosition = ((percentage) * 450) + 50;

    // Move the ball
    ball.style.left = `${newPosition}px`;
}

function moveBallSmall(percentage, ball=ball1) {
    if (percentage < 0) {
        percentage = 0
    }

    if (percentage > 1) {
        percentage = 1
    }

    if (isNaN(percentage)) {
        return
    }

    // Calculate the left position for the ball
    const newPosition = ((percentage) * 225) + 25;

    // Move the ball
    ball.style.left = `${newPosition}px`;
}

function onRCCalibrationCompleteMsg(rc_calibration_complete_msg) {

    rollCh = rc_calibration_complete_msg.rollInfo.channel
    rcChInfoArray[rollCh].max = rc_calibration_complete_msg.rollInfo.rcMax
    rcChInfoArray[rollCh].min = rc_calibration_complete_msg.rollInfo.rcMin
    rcChInfoArray[rollCh].trim = rc_calibration_complete_msg.rollInfo.rcTrim
    rcChInfoArray[rollCh].reversed = rc_calibration_complete_msg.rollInfo.reversed

    pitchCh = rc_calibration_complete_msg.pitchInfo.channel
    rcChInfoArray[pitchCh].max = rc_calibration_complete_msg.pitchInfo.rcMax
    rcChInfoArray[pitchCh].min = rc_calibration_complete_msg.pitchInfo.rcMin
    rcChInfoArray[pitchCh].trim = rc_calibration_complete_msg.pitchInfo.rcTrim
    rcChInfoArray[pitchCh].reversed = rc_calibration_complete_msg.pitchInfo.reversed

    piyawChtchCh = rc_calibration_complete_msg.yawInfo.channel
    rcChInfoArray[yawCh].max = rc_calibration_complete_msg.yawInfo.rcMax
    rcChInfoArray[yawCh].min = rc_calibration_complete_msg.yawInfo.rcMin
    rcChInfoArray[yawCh].trim = rc_calibration_complete_msg.yawInfo.rcTrim
    rcChInfoArray[yawCh].reversed = rc_calibration_complete_msg.yawInfo.reversed

    throttleCh = rc_calibration_complete_msg.throttleInfo.channel
    rcChInfoArray[throttleCh].max = rc_calibration_complete_msg.throttleInfo.rcMax
    rcChInfoArray[throttleCh].min = rc_calibration_complete_msg.throttleInfo.rcMin
    rcChInfoArray[throttleCh].trim = rc_calibration_complete_msg.throttleInfo.rcTrim
    rcChInfoArray[throttleCh].reversed = rc_calibration_complete_msg.throttleInfo.reversed
}

function onRCChannelsMsg(rc_channel_info) {
    
    // copies the raw channel data into the rcRawArray
    rcRawArray[0] = rc_channel_info.chan1_raw 
    rcRawArray[1] = rc_channel_info.chan2_raw 
    rcRawArray[2] = rc_channel_info.chan3_raw 
    rcRawArray[3] = rc_channel_info.chan4_raw 
    rcRawArray[4] = rc_channel_info.chan5_raw 
    rcRawArray[5] = rc_channel_info.chan6_raw 
    rcRawArray[6] = rc_channel_info.chan7_raw 
    rcRawArray[7] = rc_channel_info.chan8_raw 
    rcRawArray[8] = rc_channel_info.chan9_raw 
    rcRawArray[9] = rc_channel_info.chan10_raw 
    rcRawArray[10] = rc_channel_info.chan11_raw 
    rcRawArray[11] = rc_channel_info.chan12_raw 
    rcRawArray[12] = rc_channel_info.chan13_raw 
    rcRawArray[13] = rc_channel_info.chan14_raw 
    rcRawArray[14] = rc_channel_info.chan15_raw 
    rcRawArray[15] = rc_channel_info.chan16_raw 
    rcRawArray[16] = rc_channel_info.chan17_raw 
    rcRawArray[17] = rc_channel_info.chan18_raw 

    moveBallSmall((rcRawArray[0] - 1000) / 1000, ball1)
    moveBallSmall((rcRawArray[1] - 1000) / 1000, ball2)
    moveBallSmall((rcRawArray[2] - 1000) / 1000, ball3)
    moveBallSmall((rcRawArray[3] - 1000) / 1000, ball4)
    moveBallSmall((rcRawArray[4] - 1000) / 1000, ball5)
    moveBallSmall((rcRawArray[5] - 1000) / 1000, ball6)
    moveBallSmall((rcRawArray[6] - 1000) / 1000, ball7)
    moveBallSmall((rcRawArray[7] - 1000) / 1000, ball8)
    moveBallSmall((rcRawArray[8] - 1000) / 1000, ball9)
    moveBallSmall((rcRawArray[9] - 1000) / 1000, ball10)
    moveBallSmall((rcRawArray[10] - 1000) / 1000, ball11)
    moveBallSmall((rcRawArray[11] - 1000) / 1000, ball12)
    moveBallSmall((rcRawArray[12] - 1000) / 1000, ball13)
    moveBallSmall((rcRawArray[13] - 1000) / 1000, ball14)
    moveBallSmall((rcRawArray[14] - 1000) / 1000, ball15)
    moveBallSmall((rcRawArray[15] - 1000) / 1000, ball16)
    moveBallSmall((rcRawArray[16] - 1000) / 1000, ball17)
    moveBallSmall((rcRawArray[17] - 1000) / 1000, ball18)

    // // Roll - Channel 1
    if (rollCh >= 0) {
        let rollTrim = rcChInfoArray[rollCh].trim
        let rollDiff = Math.abs(rcChInfoArray[rollCh].max - rcChInfoArray[rollCh].min)
        let rollReversed = rcChInfoArray[rollCh].reversed

        let rollPercentDone = ((rcRawArray[rollCh] - rollTrim) / rollDiff) + 0.5;

        if (rollReversed) {
            rollPercentDone = 1 - rollPercentDone
        }

        moveBall(rollPercentDone, rollBall)
    }

    // // Pitch - Channel 3
    if (pitchCh >= 0) {
        let pitchTrim = rcChInfoArray[pitchCh].trim
        let pitchDiff = Math.abs(rcChInfoArray[pitchCh].max - rcChInfoArray[pitchCh].min)
        let pitchReversed = rcChInfoArray[pitchCh].reversed

        let pitchPercentDone = ((rcRawArray[pitchCh] - pitchTrim) / pitchDiff) + 0.5;
        
        if (pitchReversed) {
            pitchPercentDone = 1 - pitchPercentDone
        }

        moveBall(pitchPercentDone, pitchBall)
    }

    // // Yaw - Channel 4
    if (yawCh >= 0) {
        let yawTrim = rcChInfoArray[yawCh].trim
        let yawDiff = Math.abs(rcChInfoArray[yawCh].max - rcChInfoArray[yawCh].min)
        let yawReversed = rcChInfoArray[yawCh].reversed

        let yawPercentDone = ((rcRawArray[yawCh] - yawTrim) / yawDiff) + 0.5;
        
        if (yawReversed) {
            yawPercentDone = 1 - yawPercentDone
        }

        moveBall(yawPercentDone, yawBall)
    }

    // // Throttle
    if (throttleCh >= 0) {
        let throttleTrim = rcChInfoArray[throttleCh].trim
        let throttleDiff = Math.abs(rcChInfoArray[throttleCh].max - rcChInfoArray[throttleCh].min)
        let throttleReversed = rcChInfoArray[throttleCh].reversed

        let throttlePercentDone = ((rcRawArray[throttleCh] - throttleTrim) / throttleDiff) + 0.5;
        
        if (throttleReversed) {
            throttlePercentDone = 1 - throttlePercentDone
        }
        
        moveBall(throttlePercentDone, throttleBall)
    }
}

function calibrateRC() {
    fetch('/_cmd/mag_calibration_start?sensor=RC').then(response => {
    });
}

function copyTrims() {
    fetch('/_cmd/mag_calibration_start?sensor=RC_trim').then(response => {
    });
}

function sendStep(step) {
    fetch('/_cmd/rc_calibration?step=' + step).then(response => {
    });
}

function setupCurrentState() {    
    calibration_img.src = path_radio_switch_min_max
    displayText.innerHTML = setupRCMessage
    currentStep = 0
}

function nextButtonClicked() {
    currentStep += 1
    sendStep(currentStep)    
}

function cancelCalButtonClicked() {
    sendStep(-1)

    currentStep = 0
    calibrateRCButton.disabled = false
    nextButton.disabled = false
    cancelCalButton.disabled = true
    displayText.innerHTML = ""
    calibration_img.src = path_radio_center
}

function handleState(cs) {
    currentStep = cs

    switch (cs) {

        case 0:
            handleStep0()
            break 

        case 1:
            handleStep1()
            break // wait for them to hit the next button before starting the calibration

        case 2:
            handleStep2()
            break

        case 3:
            handleStep3()
            break

        case 4:
            handleStep4()
            break

        case 5:
            handleStep5()
            break

        case 6:
            handleStep6()
            break

        case 7:
            handleStep7()
            break

        case 8:
            handleStep8()
            break

        case 9:
            handleStep9()
            break

        case 10:
            handleStep10()
            break

        case 11:
            handleStep11()
            return // wait for them to hit the next button before saving the calibration

        case 12:
            saveParams()
            cancelCalButtonClicked() // all done, go back to start
            break


        default:
            return
    }
}

async function handleStep0() { 
    // gets the trims

    displayText.innerHTML = setupRCMessage
    calibration_img.src = path_radio_center

    calibrateRCButton.disabled = true
}


async function handleStep1() {
    // gets the max value for the throttle stick

    displayText.innerHTML = msgThrottleUp
    calibration_img.src = "../images/" + mode.value + "/radioThrottleUp.png"
    
    nextButton.disabled = true
    cancelCalButton.disabled = false
}


function handleStep2() {
    // gets the min value for the throttle stick

    displayText.innerHTML = msgThrottleDown
    calibration_img.src = "../images/" + mode.value + "/radioThrottleDown.png"
}

function handleStep3() {
    // gets the min value for the yaw stick

    displayText.innerHTML = msgYawLeft
    calibration_img.src = "../images/" + mode.value + "/radioYawLeft.png"
}

function handleStep4() {
    // gets the max value for the yaw stick

    displayText.innerHTML = msgYawRight
    calibration_img.src = "../images/" + mode.value + "/radioYawRight.png"

}

function handleStep5() {
    // gets the min value for roll

    displayText.innerHTML = msgRollLeft
    calibration_img.src = "../images/" + mode.value + "/radioRollLeft.png"
}

function handleStep6() {
    // gets the max val for roll

    displayText.innerHTML = msgRollRight
    calibration_img.src = "../images/" + mode.value + "/radioRollRight.png"
}


function handleStep7() {
    // gets the min val for pitch

    displayText.innerHTML = msgPitchDown
    calibration_img.src = "../images/" + mode.value + "/radioPitchDown.png"
}


function handleStep8() {
    // gets the max val for pitch

    displayText.innerHTML = msgPitchUp
    calibration_img.src = "../images/" + mode.value + "/radioPitchUp.png"
}

function handleStep9() {
    // just waits for sticks to center

    displayText.innerHTML = msgPitchCenter
    calibration_img.src = path_radio_center
}

function handleStep10() {
    // gets two of the switch's min and max

    displayText.innerHTML = msgSwitchMinMax
    calibration_img.src = path_radio_switch_min_max
}

function handleStep11() {
    // waits for sticks to center

    displayText.innerHTML = msgComplete
    calibration_img.src = path_radio_center
    nextButton.disabled = false
}

function saveParams() {

    cancelCalButton.disabled = true
    calibrateRCButton.disabled = false
}


document.addEventListener('DOMContentLoaded', () => {
    moveBall(0.25, ball1);
    moveBall(0.25, ball2); 
    moveBall(0.25, ball3);
    moveBall(0.25, ball4);
});