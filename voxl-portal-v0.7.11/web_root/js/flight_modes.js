let mode_channel_select = document.getElementById('mode-channel-select')
let flight_mode_1_select = document.getElementById('flight-mode-1-select')
let flight_mode_2_select = document.getElementById('flight-mode-2-select')
let flight_mode_3_select = document.getElementById('flight-mode-3-select')
let flight_mode_4_select = document.getElementById('flight-mode-4-select')
let flight_mode_5_select = document.getElementById('flight-mode-5-select')
let flight_mode_6_select = document.getElementById('flight-mode-6-select')

let arm_select = document.getElementById('arm-select')
let kill_select = document.getElementById('kill-select')
let offboard_select = document.getElementById('offboard-select')
let landing_select = document.getElementById('landing-select')
let loiter_select = document.getElementById('loiter-select')
let return_select = document.getElementById('return-select')
let flt_mode_ch = -1


function get_initial_fm_settings() {
    // Sends a request to get all of the flight mode and switch mapping parameters

    request_get_param("RC_MAP_FLTMODE")
    request_get_param("COM_FLTMODE1")
    request_get_param("COM_FLTMODE2")
    request_get_param("COM_FLTMODE3")
    request_get_param("COM_FLTMODE4")
    request_get_param("COM_FLTMODE5")
    request_get_param("COM_FLTMODE6")
    request_get_param("RC_MAP_ARM_SW")
    request_get_param("RC_MAP_KILL_SW")
    request_get_param("RC_MAP_OFFB_SW")
    request_get_param("RC_MAP_GEAR_SW")
    request_get_param("RC_MAP_LOITER_SW")
    request_get_param("RC_MAP_RETURN_SW")
}

function update_param_int(param_id, val) {
    fetch('/_cmd/param_set_int?param_id=' + param_id + '&param_val=' + val).then(response => {
    })
}

function request_get_param(param_id) {
    fetch('/_cmd/param_request?param_id=' + param_id).then(response => {
    })
}

mode_channel_select.addEventListener('change', function() {
    update_param_int("RC_MAP_FLTMODE", mode_channel_select.value)
  }
);

flight_mode_1_select.addEventListener('change', function() {
    update_param_int("COM_FLTMODE1", flight_mode_1_select.value)
  }
);

flight_mode_2_select.addEventListener('change', function() {
    update_param_int("COM_FLTMODE2", flight_mode_2_select.value)
  }
);

flight_mode_3_select.addEventListener('change', function() {
    update_param_int("COM_FLTMODE3", flight_mode_3_select.value)
  }
);

flight_mode_4_select.addEventListener('change', function() {
    update_param_int("COM_FLTMODE4", flight_mode_4_select.value)
  }
);

flight_mode_5_select.addEventListener('change', function() {
    update_param_int("COM_FLTMODE5", flight_mode_5_select.value)
  }
);

flight_mode_6_select.addEventListener('change', function() {
    update_param_int("COM_FLTMODE6", flight_mode_6_select.value)
  }
);

arm_select.addEventListener('change', function() {
    update_param_int("RC_MAP_ARM_SW", arm_select.value)
  }
);

kill_select.addEventListener('change', function() {
    update_param_int("RC_MAP_KILL_SW", kill_select.value)
  }
);

offboard_select.addEventListener('change', function() {
    update_param_int("RC_MAP_OFFB_SW", offboard_select.value)
  }
);

landing_select.addEventListener('change', function() {
    update_param_int("RC_MAP_GEAR_SW", landing_select.value)
  }
);

loiter_select.addEventListener('change', function() {
    update_param_int("RC_MAP_LOITER_SW", loiter_select.value)
  }
);

return_select.addEventListener('change', function() {
    update_param_int("RC_MAP_RETURN_SW", return_select.value)
  }
);

function onParamMsg(param_msg) {
  // Updates the gui to show that a parameter changed

  let param_string = ('' + param_msg.buffer)
  let cleaned_param_string = param_string.replace(/[^A-Za-z0-9_]/g, '') // removes anything that's not a A-Z, an underscore, or a digit 

  switch (cleaned_param_string) {

    case ("RC_MAP_FLTMODE"):
      mode_channel_select.value = param_msg.param_val
      flt_mode_ch = param_msg.param_val
      break

    case ("COM_FLTMODE1"):
      flight_mode_1_select.value = param_msg.param_val
      break

    case ("COM_FLTMODE2"):
      flight_mode_2_select.value = param_msg.param_val
      break

    case ("COM_FLTMODE3"):
      flight_mode_3_select.value = param_msg.param_val
      break

    case ("COM_FLTMODE4"):
      flight_mode_4_select.value = param_msg.param_val
      break

    case ("COM_FLTMODE5"):
      flight_mode_5_select.value = param_msg.param_val
      break

    case ("COM_FLTMODE6"):
      flight_mode_6_select.value = param_msg.param_val
      break
    
    case ("RC_MAP_ARM_SW"):
      arm_select.value = param_msg.param_val
      break

    case ("RC_MAP_KILL_SW"):
      kill_select.value = param_msg.param_val
      break

    case ("RC_MAP_OFFB_SW"):
      offboard_select.value = param_msg.param_val
      break

    case ("RC_MAP_GEAR_SW"):
      landing_select.value = param_msg.param_val
      break

    case ("RC_MAP_LOITER_SW"):
      loiter_select.value = param_msg.param_val
      break

    case ("RC_MAP_RETURN_SW"):
      return_select.value = param_msg.param_val
      break


    default:
      break
  }
}

// Moving joystick animation
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


let rcRawArray = new Array(18).fill(0)

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
  handleToggleFltMode(rcRawArray[flt_mode_ch - 1])
}


function handleToggleFltMode(val) {
  flight_mode_1_select.style.border = "2px solid grey";
  flight_mode_2_select.style.border = "2px solid grey";
  flight_mode_3_select.style.border = "2px solid grey";
  flight_mode_4_select.style.border = "2px solid grey";
  flight_mode_5_select.style.border = "2px solid grey";
  flight_mode_6_select.style.border = "2px solid grey";

  if (val >= 1000 && val < 1160) {
    flight_mode_1_select.style.border = "2px solid yellow";
  } else if (val >= 1160 && val < 1320) {
    flight_mode_2_select.style.border = "2px solid yellow";
  } else if (val >= 1320 && val < 1480) {
    flight_mode_3_select.style.border = "2px solid yellow";
  } else if (val >= 1480 && val < 1640) {
    flight_mode_4_select.style.border = "2px solid yellow";
  } else if (val >= 1640 && val < 1800) {
    flight_mode_5_select.style.border = "2px solid yellow";
  } else if (val >= 1800 && val < 2000) {
    flight_mode_6_select.style.border = "2px solid yellow";
  }
}