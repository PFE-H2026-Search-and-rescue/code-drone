const SATELLITE_INFO_INDEX = 1
const plotDiv = document.getElementById('satellitePlot');
const enablePlotCheckBox = document.getElementById('enablePlotCheckBox');

var url_base = get_url_base();
var url_port = get_url_port();

const constellationColors = {
    'GPS': 'rgb(158,202,225)',   // Light blue
    'SBAS': 'rgb(255,165,0)',  // Orange
    'Galileo': 'rgb(144,238,144)', // Light green
    'BeilDou': 'rgb(255,99,71)',  // Tomato
    'QZSS': 'rgb(255,215,0)',   // Gold
    'GLONASS': 'rgb(173,216,230)', // Light blue (slightly different)
    'NavIC': 'rgb(255,192,203)'   // Pink
};

var gps_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/gps");
gps_ws.binaryType = "arraybuffer";

enablePlotCheckBox.addEventListener('change', function() {
    if (this.checked) {
        update_param_int("GPS_SAT_INFO", 1)
    } else {
        update_param_int("GPS_SAT_INFO", 0)
    }
    alert("Power cycle your VOXL or restart VOXL-PX4 for this to take effect")
});


function updatePlot(prns, snrs) {
    const satelliteData = [];

    for (let i = 0; i < prns.length; i++) {
        if (prns[i] != 0 && prns[i] != 255) {
            const [constellation, code] = translate_prn(prns[i]);
            satelliteData.push({ prn: prns[i], snr: snrs[i], constellation: constellation, code: code });
        }
    }
    satelliteData.sort((a, b) => a.prn - b.prn);

    const sortedCodes = satelliteData.map(data => data.code);
    const sortedSnrs = satelliteData.map(data => data.snr);

    const constellationNames = [...new Set(satelliteData.map(data => data.constellation))]; // Get unique constellation names
    const barColors = satelliteData.map(data => constellationColors[data.constellation] || 'gray'); // Default to gray if not found

    const trace = {
        x: sortedCodes,
        y: sortedSnrs,
        type: 'bar',
        name: 'Satellite SNR',
        marker: {
            color: barColors,
            opacity: 0.7,
            line: {
                color: 'rgb(8,48,107)',
                width: 1.5
            }
        },
        text: sortedSnrs.map(String),
        textposition: 'outside',
        showlegend: false
    };

    const legendTraces = constellationNames.map(constellation => ({
        x: [null],
        y: [null],
        mode: 'markers',
        marker: {
            color: constellationColors[constellation] || 'gray',
            size: 10
        },
        name: constellation,
        showlegend: true
    }));

    const layout = {
        title: 'Satellite Signal to Noise Ratio (SNR)',
        showlegend: true,
        xaxis: {
            title: 'Satellite Code',
            tickvals: sortedCodes,
            ticktext: sortedCodes,
            type: 'category'
        },
        yaxis: {
            title: 'SNR (dB-Hz)',
            range: [0, 55]
        },
        bargap: 0,
        bargroupgap: 0,
        legend: {
            itemsizing: 'constant',
            itemwidth: 30,
            itemheight: 20
        }
    };

    Plotly.newPlot(plotDiv, [trace, ...legendTraces], layout);
}

function onSatelliteInfo(satellite_info_msg) {
    updatePlot(satellite_info_msg.satellite_prn, satellite_info_msg.satellite_snr);
}

function onParamMsg(param_msg) {
    // Updates the gui to show that a parameter changed
  
    let param_string = ('' + param_msg.buffer)
    let cleaned_param_string = param_string.replace(/[^A-Za-z0-9_]/g, '') // removes anything that's not a A-Z, an underscore, or a digit 
  
    switch (cleaned_param_string) {
  
      case ("GPS_SAT_INFO"):
        enablePlotCheckBox.checked = (param_msg.param_val == 1)
        break
  
      default:
        break
    }
  }

gps_ws.onmessage = function (evt) {
    var msg = evt.data;

    var parser = new jParser(msg, {
        msg_type: {
            index: 'uint8',
        },
        satellite_info: {
            satellite_prn: ['array', 'uint8', 20],
            satellite_snr: ['array', 'uint8', 20]
        },
        param_msg: {
            buffer: ['array', 'char', 16],
            param_val: 'int32'
        }
    });

    var msg_type = parser.parse('msg_type');

    switch(msg_type.index) {

        case (SATELLITE_INFO_INDEX):
            var satellite_info_msg = parser.parse('satellite_info')
            onSatelliteInfo(satellite_info_msg)
            break
        case (PARAM_MSG_INDEX):
            var param_msg = parser.parse('param_msg')
            onParamMsg(param_msg)
            break

        default:
            break
    }
}

function get_initial_gps_settings() {
    request_get_param("GPS_SAT_INFO")
}

function update_param_int(param_id, val) {
    fetch('/_cmd/param_set_int?param_id=' + param_id + '&param_val=' + val).then(response => {
    })
}

function request_get_param(param_id) {
    fetch('/_cmd/param_request?param_id=' + param_id).then(response => {
    })
}

function translate_prn(satellite_num) {
    if (satellite_num >= 1 && satellite_num <= 32) {
        // GPS
        // G1 - G32
        return ['GPS', 'G' + satellite_num]
    }
    else if (satellite_num >= 120 && satellite_num <= 158) {
        // SBAS
        // S120 - S158
        return ['SBAS', 'S' + satellite_num]
    }
    else if (satellite_num >= 211 && satellite_num <= 246) {
        // Galileo
        // E1 - E36
        return ['Galileo', 'E' + (satellite_num - 210)]
    }
    else if (satellite_num >= 159 && satellite_num <= 163) {
        // BeilDou
        // B1 - B5
        return ['BeilDou', 'B' + (satellite_num - 158)]
    }
    else if (satellite_num >= 33 && satellite_num <= 64) {
        // BeilDou
        // B6 - B37
        return ['BeilDou', 'B' + (satellite_num - 27)]
    }
    else if (satellite_num >= 193 && satellite_num <= 202) {
        // QZSS
        // Q1 - Q10
        return ['QZSS', 'Q' + (satellite_num - 192)]
    }
    else if (satellite_num >= 65 && satellite_num <= 96) {
        // GLONASS
        // R1 - R32
        return ['GLONASS', 'R' + (satellite_num - 64)]
    }
    else if (satellite_num >= 247 && satellite_num <= 253) {
        // NavIC
        // N1 - N7
        return ['NavIC', 'N' + (satellite_num - 246)]
    } else {
        return ['', '']
    }
}