// Global const
const CALIBRATION_START_PREFIX = "calibration started: ";
const CALIBRATION_COMPLETE_PREFIX = "calibration done: ";
const CALIBRATION_PROGRESS_PREFIX = "progress";
const CALIBRATION_CANCEL_PREFIX = "calibration failed";
const CALIBRATION_FAIL_PREFIX = "calibration cancelled";

let calibrationStatus = document.getElementById("calibration-status");
let orientationCalAreaHelpText = document.getElementById("orientationCalAreaHelpText");
let progressCalPercentText = document.getElementById("progressPercentText");

// Calibration Images
let accelDownImage = document.getElementById("accel_down")
let accelUpImage = document.getElementById("accel_up")
let accelFrontImage = document.getElementById("accel_front")
let accelBackImage = document.getElementById("accel_back")
let accelLeftImage = document.getElementById("accel_left")
let accelRightImage = document.getElementById("accel_right")

// Mag Calibration Arrows
let accelDownArrow = document.getElementById("accel_down_arrow");
let accelUpArrow = document.getElementById("accel_up_arrow");
let accelFrontArrow = document.getElementById("accel_front_arrow");
let accelBackArrow = document.getElementById("accel_back_arrow");
let accelLeftArrow = document.getElementById("accel_left_arrow");
let accelRightArrow = document.getElementById("accel_right_arrow");

// Calibration Statuses
let calibrationDownStatus = 0
let calibrationUpsideDownStatus = 0
let calibrationLeftStatus = 0
let calibrationRightStatus = 0
let calibrationNoseDownStatus = 0
let calibrationTailDownStatus = 0

// Calibration Buttons
let startCalibrationButton = document.getElementById("start_calibration_button")
let cancelCalibrationButton = document.getElementById("cancel_calibration_button")

let calibrationTypeInProgress = null;
let accelCalInProgress = false;
let gyroCalInProgress = false;
let magCalInProgress = false;
let levelCalInProgress = false;

let sensor = document.getElementById("sensor-select");
sensor.addEventListener('input', () => {
    if (calibrationTypeInProgress) {
        console.log('sensor val changed, cancelling calibration');
        cancelCalibration();
    }
});

setCalibrationImageVisibility(sensor.value);

sensor.addEventListener('change', function() {
    setCalibrationImageVisibility(sensor.value);
  }
);

function setCalibrationImageVisibility(sensorVal) {
    if (sensorVal == "Gyro" || sensorVal == "Level") {
        accelUpImage.style.display = 'none';
        accelFrontImage.style.display = 'none';
        accelBackImage.style.display = 'none';
        accelLeftImage.style.display = 'none';
        accelRightImage.style.display = 'none';
    } else {
        accelUpImage.style.display = '';
        accelFrontImage.style.display = '';
        accelBackImage.style.display = '';
        accelLeftImage.style.display = '';
        accelRightImage.style.display = '';
    }

    accelDownArrow.style.display = 'none';
    accelUpArrow.style.display = 'none';
    accelFrontArrow.style.display = 'none';
    accelBackArrow.style.display = 'none';
    accelLeftArrow.style.display = 'none';
    accelRightArrow.style.display = 'none';

    accelDownImage.style.borderColor = "red";
    accelUpImage.style.borderColor = "red";
    accelLeftImage.style.borderColor = "red";
    accelRightImage.style.borderColor = "red";
    accelFrontImage.style.borderColor = "red";
    accelBackImage.style.borderColor = "red";

    calibrationDownStatus = 0
    calibrationUpsideDownStatus = 0
    calibrationLeftStatus = 0
    calibrationRightStatus = 0
    calibrationNoseDownStatus = 0
    calibrationTailDownStatus = 0
};


function calibrateSensor() {
    let sensor = document.getElementById("sensor-select").value;

    fetch('/_cmd/mag_calibration_start?sensor=' + sensor).then(response => {
        // spinner.style.display = "none";
    });
}

async function disableButtons() {
    alert("Waiting for calibration to cancel...");
    startCalibrationButton.disabled = true;
    cancelCalibrationButton.disabled = true;

    while (calibrationTypeInProgress) {
        await new Promise(r => setTimeout(r, 2000));
    }
    startCalibrationButton.disabled = false;
    cancelCalibrationButton.disabled = false;

}

function cancelCalibration() {

    fetch('/_cmd/mag_calibration_stop').then(response => {
    });
    disableButtons();
}

// Helper functions
function onCalibrationMessage(text) {
    if (!text.startsWith(CALIBRATION_START_PREFIX) && (calibrationTypeInProgress == null)) {
        console.log("invalid message: " + text);
        return;
    }

    if (calibrationTypeInProgress == null) {

        calibrationStatus.innerHTML = "Calibration Started" 
        text = text.slice(CALIBRATION_START_PREFIX.length);

        // Parse the calibration type: e.g. accel, gyro, mag from the MAVLINK_MSG_STATUS_TEXT
        parts = text.split(" ");
        text = parts[1];
        console.log("part " + text);

        calibrationTypeInProgress = text;
    
        // Reset logic should be here
        if (text == "accel" || text == "mag" || text == "gyro" || text == "level") {
            console.log("Valid sensor for calibration");
            if(text == "accel") {
                accelCalInProgress = true;
            } else if(text == "mag") {
                magCalInProgress = true;
            } else if(text == "gyro") {
                gyroCalInProgress = true;
            } else if (text == "level") {
                levelCalInProgress = true;
            }

            progressCalPercentText.innerHTML = "Calibration Progress: <0>%"; 
            updateOrientationCalArea("Place your vehicle into one of the incomplete orientations shown below and hold it still");
        }

    } else if (calibrationTypeInProgress && (accelCalInProgress || magCalInProgress || gyroCalInProgress || levelCalInProgress)) {

   
        calibrationStatus.innerHTML = "Calibration in Progress" 
        //TODO replace with switch
        //We are in the correct orientation
        if (text.endsWith("orientation detected")) {
            handleOrientationDetected(text.split(" ")[0], magCalInProgress);
            return;
        }
        // Switch to the next side
        if (text.endsWith("side done, rotate to a different side")) {
            updateOrientationCalArea("Side done: place your vehicle into one of the incomplete orientations shown below and hold it still");
            handleSideDone(text.split(" ")[0], magCalInProgress);
            return;
        }
        // Already completed
        if (text.endsWith("side already completed")) {
            updateOrientationCalArea("Orientation previously completed: place your vehicle into one of the incomplete orientations shown below and hold it still");
            return;
        }

        // Update progress
        if (text.startsWith(CALIBRATION_PROGRESS_PREFIX)) {
            progress_elem = text.split(" ");
            progress = progress_elem[1];


            const match = progress.match(/<(\d+)>/);

            progressVal = 0
            if (match && match[1]) {
                progressVal = parseInt(match[1])
            }

            if (gyroCalInProgress || levelCalInProgress) {
                if (calibrationDownStatus == 2) {
                    return;
                }

                accelDownImage.style.borderColor = "yellow";
            }
    
            progressCalPercentText.innerHTML = "Calibration Progress: " + Math.min(progressVal, 100) + "%"; 
            console.log("Progress: " + progressVal);
        }
    
        // Calibration complete!
        if (text.startsWith(CALIBRATION_COMPLETE_PREFIX)) {
            if (gyroCalInProgress || levelCalInProgress) {
                calibrationDownStatus = 2
                accelDownImage.style.borderColor = "green";
            }

            stopCalibration("Calibration success");
            return;
        }

        if (text.startsWith(CALIBRATION_CANCEL_PREFIX)) {
            setCalibrationImageVisibility(sensor.value);
            stopCalibration("Calibration cancelled");
            return;

        }

        if (text.startsWith(CALIBRATION_FAIL_PREFIX)) {
            setCalibrationImageVisibility(sensor.value);
            stopCalibration("Calibration failed");
            return;
        }

    }

    else {

        console.log("Unknown state");
    }
}

function handleOrientationDetected(side, magCalInProgress) {
    if (magCalInProgress) {
        updateOrientationCalArea("Rotate the vehicle continuously until marked complete");

    }
    else {
        updateOrientationCalArea("Hold still in the current orientation");
    }

    switch (side) {
        case "down":
            if (calibrationDownStatus == 2) {
                break;
            }

            calibrationDownStatus = 1

            accelDownImage.style.borderColor = "yellow";
            if (magCalInProgress) {
                accelDownArrow.style.display = 'inline-block';
             }
            break;
        case "up":
            if (calibrationUpsideDownStatus == 2) {
                break;
            }

            calibrationUpsideDownStatus = 1

            accelUpImage.style.borderColor = "yellow";
            if (magCalInProgress) {
                accelUpArrow.style.display = 'inline-block';
             }
            break;
        case "left":
            if (calibrationLeftStatus == 2) {
                break;
            }

            calibrationLeftStatus = 1

            accelLeftImage.style.borderColor = "yellow";
            if (magCalInProgress) {
                accelLeftArrow.style.display = 'inline-block';
             }
            break;
        case "right":
            if (calibrationRightStatus == 2) {
                break;
            }

            calibrationRightStatus = 1

            accelRightImage.style.borderColor = "yellow";
            if (magCalInProgress) {
                accelRightArrow.style.display = 'inline-block';
             }
            break;
        case "front":
            if (calibrationNoseDownStatus == 2) {
                break;
            }

            calibrationNoseDownStatus = 1

            accelFrontImage.style.borderColor = "yellow";
            if (magCalInProgress) {
                accelFrontArrow.style.display = 'inline-block';
             }
            break;
        case "back":
            if (calibrationTailDownStatus == 2) {
                break;
            }

            calibrationTailDownStatus = 1

            accelBackImage.style.borderColor = "yellow";
            if (magCalInProgress) {
                accelBackArrow.style.display = 'inline-block';
             }
            break;

    }

}

function resetCalibrationState() {
    orientationCalInProgress = false;
    calibrationTypeInProgress = null;
    // Reset all in progress sensors
    magCalInProgress = false;
    accelCalInProgress = false;
    gyroCalInProgress = false;
    levelCalInProgress = false;

    // Reset the orientation cal text
    updateOrientationCalArea("");

    // Reset the progress percent
    progressCalPercentText.innerHTML = ""; 
}


function handleSideDone(side) {
    switch (side) {
        case "down":
            calibrationDownStatus = 2

            accelDownImage.style.borderColor = "green";
            if (magCalInProgress) {
                accelDownArrow.style.display = 'none';
             }
            break;
        case "up":
            calibrationUpsideDownStatus = 2

            accelUpImage.style.borderColor = "green";
            if (magCalInProgress) {
                accelUpArrow.style.display = 'none';
             }
            break;
        case "left":
            calibrationLeftStatus = 2

            accelLeftImage.style.borderColor = "green";
            if (magCalInProgress) {
                accelLeftArrow.style.display = 'none';
             }
            break;
        case "right":
            calibrationRightStatus = 2

            accelRightImage.style.borderColor = "green";
            if (magCalInProgress) {
                accelRightArrow.style.display = 'none';
             }
            break;
        case "front":
            calibrationNoseDownStatus = 2

            accelFrontImage.style.borderColor = "green";
            if (magCalInProgress) {
                accelFrontArrow.style.display = 'none';
             }
            break;
        case "back":
            calibrationTailDownStatus = 2

            accelBackImage.style.borderColor = "green";
            if (magCalInProgress) {
                accelBackArrow.style.display = 'none';
             }
            break;
    }
}

function updateOrientationCalArea(message) {
    const orientationCalArea = document.getElementById("orientationCalAreaHelpText");
    orientationCalArea.innerHTML = message;
}

function stopCalibration(status) {
    calibrationStatus.innerHTML = status;
    resetCalibrationState();
}


