let motorRangeAll = document.getElementById('motorRangeAll')
let motorRange1 = document.getElementById('motorRange1')
let motorRange2 = document.getElementById('motorRange2')
let motorRange3 = document.getElementById('motorRange3')
let motorRange4 = document.getElementById('motorRange4')
let enableSlidersCheckbox = document.getElementById('enableSlidersCheckbox')

let motor1Graphic = document.getAnimations('motor1Graphic')

let spinAgainButton = document.getElementById('spinAgainButton')
let identifyMotorsButton = document.getElementById('identifyMotorsButton')
let identifyMotorText = document.getElementById('identifyMotorText')

let esc1_select = document.getElementById('esc1')
let esc2_select = document.getElementById('esc2')
let esc3_select = document.getElementById('esc3')
let esc4_select = document.getElementById('esc4')

let esc1_rev = document.getElementById('esc1_rev')
let esc2_rev = document.getElementById('esc2_rev')
let esc3_rev = document.getElementById('esc3_rev')
let esc4_rev = document.getElementById('esc4_rev')

let identifyMotorsInProgress = false;

disable_motor_graphics()
toggleDisableSliders(true)

// Map from esc # -> motor #
let esc_map = new Map()
let prev_esc_map = new Map()
let prev_esc_map_rev = new Map()

var url_base = get_url_base();
var url_port = get_url_port();

let currMotor = 1
var actuator_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/test-actuators");

enableSlidersCheckbox.addEventListener('change', function() {
    if (this.checked) {
        toggleDisableSliders(false)
        toggleDisableESC(true)
        identifyMotorsButton.disabled = true
        spinAgainButton.disabled = true
    } else {
        toggleDisableSliders(true)
        toggleDisableESC(false)
        identifyMotorsButton.disabled = false
        spinAgainButton.disabled = false
    }
})

esc1_select.addEventListener('change', function() {
    set_esc_param(1, parseInt(esc1_select.value) - 100)
  }
);

esc2_select.addEventListener('change', function() {
    set_esc_param(2, parseInt(esc2_select.value) - 100)
  }
);

esc3_select.addEventListener('change', function() {
    set_esc_param(3, parseInt(esc3_select.value) - 100)
  }
);

esc4_select.addEventListener('change', function() {
    set_esc_param(4, parseInt(esc4_select.value) - 100)
  }
);

esc1_rev.addEventListener('change', function() {
    if (this.checked) {
        set_esc_rev_param(1, 1)
    } else {
        set_esc_rev_param(1, 0)
    }
})

esc2_rev.addEventListener('change', function() {
    if (this.checked) {
        set_esc_rev_param(2, 1)
    } else {
        set_esc_rev_param(2, 0)
    }
})

esc3_rev.addEventListener('change', function() {
    if (this.checked) {
        set_esc_rev_param(3, 1)
    } else {
        set_esc_rev_param(3, 0)
    }
})

esc4_rev.addEventListener('change', function() {
    if (this.checked) {
        set_esc_rev_param(4, 1)
    } else {
        set_esc_rev_param(4, 0)
    }
})

motorRangeAll.oninput = function(){

    const currentValue = this.value;

    request_motor_spin(1, currentValue, 0)
    request_motor_spin(2, currentValue, 0)
    request_motor_spin(3, currentValue, 0)
    request_motor_spin(4, currentValue, 0)
};


motorRange1.oninput = function(){

    const currentValue = this.value;

    request_motor_spin(1, currentValue, 0)
};


motorRange2.oninput = function(){

    const currentValue = this.value;

    request_motor_spin(2, currentValue, 0)
};


motorRange3.oninput = function(){

    const currentValue = this.value;

    request_motor_spin(3, currentValue, 0)
};


motorRange4.oninput = function(){

    const currentValue = this.value;

    request_motor_spin(4, currentValue, 0)
};


function identifyMotorsButtonClicked() {

    if (!checkPrevEscs()) {
        alert("Each ESC function must be assigned a unique motor # before you can identify them")
        return;
    }

    identifyMotorsButton.disabled = true
    enableSlidersCheckbox.disabled = true

    for (let i = 1; i <= 4; i++) {
        let motor_graphic = document.getElementById('motor_graphic_' + i.toString())
        motor_graphic.style.opacity = 0.25;
        motor_graphic.disabled = false;
        motor_graphic.style.cursor = 'pointer';
        request_esc_param(i)
    }


    spinAgainButton.style = "margin-left: 10px; display: block;"
    identifyMotorText.innerHTML = "Click on the motor that's spinning"
    identifyMotorsInProgress = true
    currMotor = 1;
    request_motor_spin(currMotor, 15, 1)
}

function spinAgainButtonClicked() {
    request_motor_spin(currMotor, 15, 1)
}

function motorButton(new_motor_num) {

    if (!identifyMotorsInProgress) {
        return
    }
    
    let motor_graphic = document.getElementById('motor_graphic_' + new_motor_num.toString())
    motor_graphic.style.opacity = 1;
    motor_graphic.disabled = true;
    motor_graphic.style.cursor = 'Default';

    esc_map.set(prev_esc_map.get(currMotor), new_motor_num)

    if (currMotor != 4) {
        currMotor++;
        request_motor_spin(currMotor, 15, 1);        
    } else {
        disable_motor_graphics()
    
        identifyMotorText.innerHTML = "Done picking motors, parameters saved automatically"
        identifyMotorsButton.disabled = false;
        enableSlidersCheckbox.disabled = false;
        identifyMotorsInProgress = false;

        saveESCParams()
    }
}

function disable_motor_graphics() {

    for (let i = 1; i <= 4; i++) {
        let motor_graphic = document.getElementById('motor_graphic_' + i.toString())
        motor_graphic.style.opacity = 1;
        motor_graphic.disabled = true;
        motor_graphic.style.cursor = 'Default';
    }
}

function saveESCParams() {

    for (const [esc_func, motor_num] of esc_map) {
        set_esc_param(esc_func, motor_num)
    }
}


function request_motor_spin(motor_num, throttle, test) {
    fetch('/_cmd/actuator_test?motor_num=' + motor_num + '&motor_val=' + throttle + '&test=' + test).then(response => {
    })
}


function set_esc_param(esc_func, motor_num) {
    fetch('/_cmd/param_set_int?param_id=VOXL_ESC_FUNC' + esc_func.toString() + '&param_val=' + (100 + motor_num).toString()).then(response => {
    })
}

function set_esc_rev_param(esc_func, rev) {
    fetch('/_cmd/param_set_int?param_id=VOXL_ESC_SDIR' + esc_func.toString() + '&param_val=' + (rev).toString()).then(response => {
    })
}


function request_esc_param(esc_func) {
    fetch('/_cmd/param_request?param_id=VOXL_ESC_FUNC' + esc_func.toString()).then(response => {
    })
}

function request_esc_rev_param(esc_func) {
    fetch('/_cmd/param_request?param_id=VOXL_ESC_SDIR' + esc_func.toString()).then(response => {
    })
}

function onParamMsg(param_msg) {
    let param_string = ('' + param_msg.buffer)
    let cleaned_param_string = param_string.replace(/[^A-Za-z0-9_]/g, '') // removes anything that's not a A-Z, an underscore, or a digit 

    const regex_esc_func = /VOXL_ESC_FUNC(\d+)/;
    const regex_esc_rev = /VOXL_ESC_SDIR(\d+)/;

    const match_esc_func = cleaned_param_string.match(regex_esc_func);
    const match_esc_rev = cleaned_param_string.match(regex_esc_rev)
    
    if (match_esc_func) {
        prev_esc_map.set(parseInt(param_msg.param_val) - 100, parseInt(match_esc_func[1])); // Convert the matched string to an integer
        prev_esc_map_rev.set(parseInt(match_esc_func[1]), parseInt(param_msg.param_val) - 100);

        let esc_elem = document.getElementById('esc' + match_esc_func[1])
        esc_elem.value = param_msg.param_val
    } else if (match_esc_rev) {
        let esc_rev_elem = document.getElementById('esc' + match_esc_rev[1] + '_rev')
        esc_rev_elem.checked = param_msg.param_val == 1 ? true : false;
    }
}

function checkPrevEscs() {
    let motorSet = new Set()
    let escSet = new Set()

    for (const [esc_func, motor_num] of prev_esc_map_rev) {
        motorSet.add(motor_num)
        escSet.add(esc_func)
    }


    if (motorSet.size != 4 || escSet.size != 4) {
        return false;
    }
    return true;
}

function get_initial_actuator_settings() {
    for (let i = 1; i <= 4; i++) {
        request_esc_param(i)
        request_esc_rev_param(i)
    }
}

function toggleDisableSliders(disable) {
    motorRangeAll.disabled = disable
    motorRange1.disabled = disable
    motorRange2.disabled = disable
    motorRange3.disabled = disable
    motorRange4.disabled = disable


    motorRangeAll.value = 0;
    motorRange1.value = 0;
    motorRange2.value = 0;
    motorRange3.value = 0;
    motorRange4.value = 0;

    for (let i = 1; i <= 4; i++) {
        request_motor_spin(i, 0, 0)
    }
}

function toggleDisableESC(disable) {
    esc1_select.disabled = disable
    esc2_select.disabled = disable
    esc3_select.disabled = disable
    esc4_select.disabled = disable

    esc1_rev.disabled = disable
    esc2_rev.disabled = disable
    esc3_rev.disabled = disable
    esc4_rev.disabled = disable
}


