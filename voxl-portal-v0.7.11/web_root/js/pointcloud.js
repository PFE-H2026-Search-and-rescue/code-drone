import * as THREE from './3rd_party/three.module.js';
import { TrackballControls } from './3rd_party/TrackballControls.js';
import { FontLoader} from './3rd_party/FontLoader.js';
import { TextGeometry} from './3rd_party/TextGeometry.js';
import {OrbitControls} from "./3rd_party/OrbitControls.js";
import { OrbitControlsGizmo } from  "./3rd_party/OrbitControlsGizmo.js";

// global elements
let perspectiveCamera, controls, scene, renderer, points, ptc_name, pointcloud_ws, timerId;

let default_color = 0; // black, 1 for white if dark mode

let MAX_SOCKET_RETRIES = 5; 
let curr_retries = 0; // track this

// camera setup
const fov = 50;
const aspect = window.innerWidth / window.innerHeight;
const near = 0.25;
const far = 1000;
perspectiveCamera = new THREE.PerspectiveCamera( fov, aspect, near, far );
perspectiveCamera.position.z = -1;
perspectiveCamera.position.x = 0;
perspectiveCamera.position.y = -28;
perspectiveCamera.up.set(0, 0, -1);
perspectiveCamera.rotation.set(0, 0, 0);

// material
const black_material = new THREE.LineBasicMaterial( { color: 0x000000, linewidth: 1     } );
const thin_red_material = new THREE.LineBasicMaterial( {color: 0xff0000, linewidth: 1});

// scene setup
scene = new THREE.Scene();
scene.background = new THREE.Color( 0xcccccc );

var grid = new THREE.GridHelper(15, 25);
grid.geometry.rotateX( Math.PI / 2 );

scene.add(grid);

const tick_markers = [];

// add a vertical marker for z axis (height)
const z_marker_points = [];

z_marker_points.push( new THREE.Vector3( 0, 0, -5));
z_marker_points.push( new THREE.Vector3( 0, 0,  5));

const z_marker_geom = new THREE.BufferGeometry().setFromPoints( z_marker_points );
const z_marker = new THREE.Line( z_marker_geom, black_material);

// now add the labels to the marker, at 0.5m increments?
let label_material;
label_material = new THREE.MeshPhongMaterial(
    { color: 0xcccccc, specular: 0xffffff }
);

var font_loader = new FontLoader();
font_loader.load( 'fonts/helvetiker_regular.typeface.json', function ( font ) {
    for ( var i = 5.0; i > -5.5; i-=0.5){
        var label_text_geom = new TextGeometry( " " + i, {
            font: font,
            size: 0.075,
            height: 0,
            curveSegments: 12,
            bevelEnabled: false
        });

        var tick_marker = new THREE.Mesh( label_text_geom, label_material );

        tick_marker.position.set(0, 0, -i);
        tick_marker.up.set(0, 0, -1);
        tick_marker.lookAt(0, -20, -i);

        scene.add( tick_marker );
        tick_markers.push(tick_marker);
    }
});
scene.add(z_marker);

const light = new THREE.HemisphereLight();
scene.add( light );

// renderer setup
renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setPixelRatio( window.devicePixelRatio );
renderer.setSize( window.innerWidth, window.innerHeight );
var canvas = renderer.domElement;

document.body.appendChild( canvas );

window.addEventListener( 'resize', onWindowResize );

createControls( perspectiveCamera );

const insetWidth = 300, insetHeight = 300;
let container2 = document.getElementById( 'inset' );
container2.width = insetWidth;
container2.height = insetHeight;

const controlsGizmo = new OrbitControlsGizmo(controls, { size:  300, padding:  8 });
controlsGizmo.domElement.style.marginTop = 0 + "px";
container2.appendChild(controlsGizmo.domElement);

// scene
let scene2 = new THREE.Scene();
scene2.background = new THREE.Color( 0xcccccc );

// camera
let camera2 = new THREE.PerspectiveCamera( 50, insetWidth / insetHeight, 1, 1000 );
camera2.up = perspectiveCamera.up; // important!

// axes
let axes2 = new THREE.AxesHelper( 150 );
scene2.add( axes2 );

let conf_filter_value = 0;

animate();

let first_pts = true;

const urlParams = new URLSearchParams(window.location.search);
if(!urlParams.has('ptc')){
    console.log("No Pointcloud Name Specified");
} else {
    ptc_name = urlParams.get('ptc');
    console.log(ptc_name);
}

// color gradient for tof
function getrgGradientColor(value){
    const num_colors = 2;
    var colors = new Array();
    colors[0] = new Array(0.5,0,0);
    colors[1] = new Array(0,0.9,0);

    var idx1, idx2;        // |-- Our desired color will be between these two indexes in "color".
    var fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

    if(value <= 0) idx1 = idx2 = 0;
    else if(value >= 1) idx1 = idx2 = num_colors-1;
    else {
        value = value * (num_colors-1);
        idx1  = Math.floor(value);
        idx2  = idx1+1;
        fractBetween = value - idx1;
    }

    var red   = (colors[idx2][0] - colors[idx1][0])*fractBetween + colors[idx1][0];
    var green = (colors[idx2][1] - colors[idx1][1])*fractBetween + colors[idx1][1];
    var blue  = (colors[idx2][2] - colors[idx1][2])*fractBetween + colors[idx1][2];

    return {"r": red, "g": green, "b": blue};
}

function getintensGradientColor(value){
    const num_colors = 2;
    var colors = new Array();
    colors[0] = new Array(0.25,0.25,0.25);
    colors[1] = new Array(1,1,1);

    var idx1, idx2;        // |-- Our desired color will be between these two indexes in "color".
    var fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

    if(value <= 0) idx1 = idx2 = 0;
    else if(value >= 1) idx1 = idx2 = num_colors-1;
    else {
        value = value * (num_colors-1);
        idx1  = Math.floor(value);
        idx2  = idx1+1;
        fractBetween = value - idx1;
    }

    var red   = (colors[idx2][0] - colors[idx1][0])*fractBetween + colors[idx1][0];
    var green = (colors[idx2][1] - colors[idx1][1])*fractBetween + colors[idx1][1];
    var blue  = (colors[idx2][2] - colors[idx1][2])*fractBetween + colors[idx1][2];

    return {"r": red, "g": green, "b": blue};
}
//////////////////////////////////////////////////////////
// Websocket functions
//////////////////////////////////////////////////////////
var url_base = get_url_base();
var url_port = get_url_port();

function connect() {
    pointcloud_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/ptc/" + ptc_name);
    pointcloud_ws.binaryType = "arraybuffer";

    pointcloud_ws.onopen = function() {
        console.log("[INFO] Pointcloud websocket open");
        curr_retries = 0;
    };

    // TODO decide if it's a point cloud or tof2_data_t type more robustly than
    // by looking at the pipe name
    if(ptc_name.includes("_pc") || !ptc_name.includes("tof")){
        pointcloud_ws.onmessage = function (evt) {
            console.log("RECEIVED");
            var received_msg = evt.data;
            var parser = new jParser(received_msg, {
                pointcloud_metadata_t: {
                    magic_number: 'uint32',
                    timestamp_ns: ['array', 'uint32', 2],
                    n_points: 'uint32',
                    format: 'uint32',
                    id: 'uint32',
                    server_name: ['array', 'char', 32],
                    reserved: 'uint32'
                },
                pointcloud_xyz: {
                    x: 'float32',
                    y: 'float32',
                    z: 'float32',
                },
                pointcloud_xyzc: {
                    x: 'float32',
                    y: 'float32',
                    z: 'float32',
                    c: 'float32',
                },
                pointcloud_xyz_rgb: {
                    x: 'float32',
                    y: 'float32',
                    z: 'float32',
                    r: 'uint8',
                    g: 'uint8',
                    b: 'uint8',
                },
                pointcloud_xyzc_rgb: {
                    x: 'float32',
                    y: 'float32',
                    z: 'float32',
                    c: 'float32',
                    r: 'uint8',
                    g: 'uint8',
                    b: 'uint8',
                },
                pointcloud_xy: {
                    x: 'float32',
                    y: 'float32',
                },
                pointcloud_xyc: {
                    x: 'float32',
                    y: 'float32',
                    c: 'float32',
                }
            });

            var pointcloud_meta = parser.parse('pointcloud_metadata_t');
            if (pointcloud_meta.magic_number != 1448040524) return;

            // ptcloud attributes
            var positions = [];
            var colors = [];

            switch (pointcloud_meta.format){
                case 0:
                    for (var i = 0; i < pointcloud_meta.n_points; i++){
                        var curr_point = parser.parse('pointcloud_xyz');
                        positions.push(curr_point.x, curr_point.y, curr_point.z);
                        colors.push(default_color, default_color, default_color);
                    }
                    break;
                case 1:
                    for (var i = 0; i < pointcloud_meta.n_points; i++){
                        var curr_point = parser.parse('pointcloud_xyzc');
                        positions.push(curr_point.x, curr_point.y, curr_point.z);
                        var new_color = getrgGradientColor(curr_point.c/255.);
                        colors.push(new_color.r, new_color.g, new_color.b);
                    }
                    break;
                case 2:
                    for (var i = 0; i < pointcloud_meta.n_points; i++){
                        var curr_point = parser.parse('pointcloud_xyz_rgb');
                        positions.push(curr_point.x, curr_point.y, curr_point.z);
                        colors.push(curr_point.r, curr_point.g, curr_point.b);
                    }
                    break;
                case 3:
                    for (var i = 0; i < pointcloud_meta.n_points; i++){
                        var curr_point = parser.parse('pointcloud_xyzc_rgb');
                        positions.push(curr_point.x, curr_point.y, curr_point.z);
                        colors.push(curr_point.r, curr_point.g, curr_point.b);
                    }
                    break;
                case 4:
                    for (var i = 0; i < pointcloud_meta.n_points; i++){
                        var curr_point = parser.parse('pointcloud_xy');
                        positions.push(curr_point.x, curr_point.y, 0);
                        colors.push(default_color, default_color, default_color);
                    }
                    break;
                case 5:
                    for (var i = 0; i < pointcloud_meta.n_points; i++){
                        var curr_point = parser.parse('pointcloud_xyc');
                        positions.push(curr_point.x, curr_point.y, 0);
                        var new_color = getrgGradientColor(curr_point.c/255.);
                        colors.push(new_color.r, new_color.g, new_color.b);
                    }
                    break;
            }

            if (first_pts){
                first_pts = false;
                var pt_geometry = new THREE.BufferGeometry();
                pt_geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );
                pt_geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );

                var material = new THREE.PointsMaterial( { size: 0.1, vertexColors: true } );

                points = new THREE.Points( pt_geometry, material );
                keepAlive();
            }
            else {
                points.geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );
                points.geometry.attributes.color.needsUpdate = true;

                points.geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );
                points.geometry.buffersNeedUpdate = true;
                points.geometry.attributes.position.needsUpdate = true;
            }
            scene.add( points );
        }
    }
    else {
        let tof_mode = 1; // start with intensity coloring, mode0 is confidence coloring
        const opt_div = document.getElementById('options_div');

        if (!opt_div.hasChildNodes()){
            var conf = document.createElement("button");
            var intens = document.createElement("button");

            var conf_slider = document.createElement("input");
            var conf_slider_value_div = document.createElement("div");
            var conf_slider_value = document.createTextNode("0");
            conf_slider_value_div.style.textAlign = "center";
            conf_slider_value_div.appendChild(conf_slider_value)

            conf_slider.id = "confidence_slider"
            conf_slider.type = "range"
            conf_slider.min = 0
            conf_slider.max = 254
            conf_slider.value = 0
            conf_slider.class = "w3-show confidence_slider"

            conf.className = " w3-button w3-block w3-red";
            intens.className = " w3-button w3-block w3-green";

            conf.innerText = "Confidence";
            intens.innerText = "Intensity";

            conf.addEventListener("click", () => {
                conf.classList.replace("w3-red", "w3-green");
                intens.classList.replace("w3-green", "w3-red");
                tof_mode = 0;
            });

            intens.addEventListener("click", () => {
                intens.classList.replace("w3-red", "w3-green");
                conf.classList.replace("w3-green", "w3-red");
                tof_mode = 1;
            });

            conf_slider.addEventListener("change", () => {
                conf_filter_value = conf_slider.value;
                conf_slider_value.nodeValue = conf_slider.value
            })

            opt_div.appendChild(conf);
            opt_div.appendChild(intens);
            opt_div.appendChild(conf_slider);
            opt_div.appendChild(conf_slider_value_div);
        }

        const MPA_MAX_TOF2_SIZE = 240*180;

        pointcloud_ws.onmessage = function (evt) {
            var received_msg = evt.data;
            var parser = new jParser(received_msg, {
                tof2_data_t: {
                    magic_number: 'uint32',
                    timestamp_ns: ['array', 'uint32', 2],
                    width: 'int16',
                    height: 'int16',
                    reserved1: 'int16',
                    reserved2: 'int16',
                    points: ['array', 'float32', MPA_MAX_TOF2_SIZE * 3],
                    noises: ['array', 'float32', MPA_MAX_TOF2_SIZE],
                    grayValues: ['array', 'uint8', MPA_MAX_TOF2_SIZE],
                    confidences: ['array', 'uint8', MPA_MAX_TOF2_SIZE]
                }
            });

            var tof_data = parser.parse('tof2_data_t');

            // old magic number was 1448040524
            // this is for new tof2_data_t type
            if (tof_data.magic_number != 1448040525) return;

            var n_pts = tof_data.width * tof_data.height;

            // ptcloud attributes
            var positions = [];
            var colors = [];

            switch (tof_mode){
                case 0:
                    for (var i = 0; i < n_pts; i++){
                        if(tof_data.confidences[i] < conf_filter_value)
                        {
                            continue;
                        }

                        positions.push(tof_data.points[i*3], tof_data.points[(i*3) + 1], tof_data.points[(i* 3) + 2]);
                        var new_color = getrgGradientColor(tof_data.confidences[i]/255.);
                        colors.push(new_color.r, new_color.g, new_color.b);
                    }

                    break;
                case 1:
                    for (var i = 0; i < n_pts; i++){

                        if(tof_data.confidences[i] < conf_filter_value)
                        {
                            continue;
                        }

                        positions.push(tof_data.points[i*3], tof_data.points[(i*3) + 1], tof_data.points[(i* 3) + 2]);
                        var new_color = getintensGradientColor(tof_data.grayValues[i]/255.);
                        colors.push(new_color.r, new_color.g, new_color.b);                    
                    }
                    break;
            }


            if (first_pts){
                first_pts = false;
                var pt_geometry = new THREE.BufferGeometry();
                pt_geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );
                pt_geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );

                var material = new THREE.PointsMaterial( { size: 0.1, vertexColors: true } );

                points = new THREE.Points( pt_geometry, material );
                keepAlive();
            }
            else {
                points.geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );
                points.geometry.attributes.color.needsUpdate = true;

                points.geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );
                points.geometry.buffersNeedUpdate = true;
                points.geometry.attributes.position.needsUpdate = true;
            }
            scene.add( points );
        }

    }

    pointcloud_ws.onclose = function(e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        curr_retries++;
        if (curr_retries <= MAX_SOCKET_RETRIES)
            setTimeout(function() {
                connect();
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    pointcloud_ws.onerror = function(err) {
      console.error('Socket encountered error: ', err.message, 'Closing socket');
      pointcloud_ws.close();
    };
  }

connect();

$(window).on('beforeunload', function(){
    pointcloud_ws.close();
});

function keepAlive(timeout = 5000) {
    if (pointcloud_ws.readyState == pointcloud_ws.OPEN) {
        pointcloud_ws.send('');
    }
    timerId = setTimeout(keepAlive, timeout);
}


//////////////////////////////////////////////////////////
// Handle all of our buttons
//////////////////////////////////////////////////////////
const home = document.getElementById('home');

const stream_add_btn = document.getElementById('stream_add');
const stream0 = document.getElementById('stream0');
const stream_rem_btn = document.getElementById('stream_remove');


// always checking if home is clicked, if so, close ws
home.addEventListener("click", () => {
    pointcloud_ws.close();
});

document.getElementById('dark_mode').classList.remove("w3-hide");

mode_btn.addEventListener("click", () => {
    if (mode_btn.classList.contains("w3-black")){
        var x = document.getElementById("button_text");
        x.textContent = "Dark Mode";
        mode_btn.className = "w3-button w3-block w3-white";
        scene.background = new THREE.Color( 0x000000 );
        scene2.background = new THREE.Color( 0x000000 );
        default_color = 1;
        localStorage.setItem("dark_mode","true");
        tick_markers.forEach((tick_marker) => {
            tick_marker.material.color.set(0xcccccc);
        })
        z_marker.material.color.set(0xcccccc);
        label_material.material.color.set(0x000000);
    }
    else {
        mode_btn.className = "w3-button w3-block w3-black";
        var x = document.getElementById("button_text");
        x.textContent = "Light Mode";
        scene.background = new THREE.Color( 0xcccccc );
        scene2.background = new THREE.Color( 0xcccccc );
        default_color = 0;
        localStorage.setItem("dark_mode","false");
        tick_markers.forEach((tick_marker) => {
            tick_marker.material.color.set(0x000000);
        })
        z_marker.material.color.set(0x000000);
        label_material.material.color.set(0x000000);
    }
})

mode_btn.click();

/*
if (document.body.style.background = "black") {
    document.getElementById("button_text").textContent = "Dark Mode";
}
else {
    document.getElementById("button_text").textContent = "Light Mode";
}

var dark_mode_status = localStorage.getItem("dark_mode");
if (dark_mode_status == "true"){
    // we were in dark mode on the previous page, so mimic thats
    mode_btn.click();
}
*/

function createControls( camera ) {
    controls = new OrbitControls( camera, renderer.domElement );
    controls.zoomSpeed = 1.2;
    controls.panSpeed = 0.8;
    //perspectiveCamera.position.z = 20;

    controls.rotateSpeed = 1.0;

    controls.keys = [ 'KeyA', 'KeyS', 'KeyD' ];
}

function onWindowResize() {

    const aspect = window.innerWidth / window.innerHeight;

    perspectiveCamera.aspect = aspect;
    perspectiveCamera.updateProjectionMatrix();

    renderer.setSize( window.innerWidth, window.innerHeight );

}

function animate() {

    requestAnimationFrame( animate );

    controls.update();

    //copy position of the camera into inset
    camera2.position.copy( perspectiveCamera.position );
    camera2.position.sub( controls.target );
    camera2.position.setLength( 300 );
    camera2.lookAt( scene2.position );

    renderer.render( scene, perspectiveCamera );
}

function closeCameraAcc(){

    //Close and remove the green coloring of the dropdown
    var x = document.getElementById("streamAcc");
    x.className = x.className.replace(" w3-show", "w3-hide");

    //Remove all the children, they should get repopulated to ensure no faulty cameras
    /*
    var eles = document.getElementsByClassName("cam-item");
    while(eles[0]) {
      eles[0].parentNode.removeChild(eles[0]);
    }
    */

}

stream_rem_btn.addEventListener("click", () => {
    stream0.src = "";
    stream0.className = "w3-hide";
    var div = document.getElementById('stream_div');
    div.style = "";
    stream_rem_btn.className="w3-hide";
    var cam_name_h3 = document.getElementById("cam_name");
    cam_name_h3.style.display = "none";
})

let already_called = false;
stream_add_btn.addEventListener("click", () => {
    var x = document.getElementById("streamAcc");

    if (x.classList.contains("w3-hide")) {
        if (already_called == false) {
            updateCameraList();
            already_called = true;
        }
        x.classList.replace("w3-hide", "w3-show");
    } else {
      closeCameraAcc();
      x.classList.replace("w3-show", "w3-hide");

    }
})

function updateCameraList(){
    fetch('/_cmd/list_cameras').then(function(response) {
      return response.text().then(function(text) {
        var y = document.getElementById("dropdown_vertical_streamAcc");
        var cams = text.split(" ");

        for(var i = 0; i < cams.length; i++){
          if(cams[i].length < 1) continue;

          var ele = document.createElement("a");
          ele.setAttribute("id",       "btn_"+cams[i]);
          ele.setAttribute("pipe",     cams[i]);
          ele.addEventListener("click", function(e) {
            var imgString = "/video_raw/"+this.getAttribute("pipe");
            stream0.src = imgString;
            stream0.className = "";
            stream_rem_btn.className = "w3-button w3-red";
            var cam_name_button = document.getElementById("cam_name");
            cam_name_button.style.display = "";
            cam_name_button.textContent = this.textContent;
            //closeCameraAcc();

            const move_or_resize = function(e) {
                const height = this.offsetHeight;
                const width = this.offsetWidth;

                const top = this.offsetTop;
                const left = this.offsetLeft;

                const pos_y = top + height - e.pageY;
                const pos_x = left + width - e.pageX;

                const drag = (e) => {
                    this.style.top = e.pageY + pos_y - height + "px";
                    this.style.left = e.pageX + pos_x - width + "px";
                }

                const stop_drag = (e) => {
                    document.removeEventListener("mousemove", drag);
                    document.removeEventListener("mouseup", stop_drag);
                    e.preventDefault();
                }
                
                
                if (pos_x > 12 && pos_y > 12) {
                    console.log("DRAGGING");
                    document.addEventListener("mousemove", drag);
                    document.addEventListener("mouseup", stop_drag);
                    e.preventDefault();
                }
                else {
                    //curr_str_div.style.position = "absolute"
                }
            }

            var stream_div = document.getElementById("stream_div");
            stream_div.addEventListener("mousedown", move_or_resize);
        })

          ele.classList.add("cam-item");
          ele.classList.add("w3-bar-item");
          ele.classList.add("w3-button");
          ele.classList.add("w3-green");

          var nameList = cams[i].split("_");
          for(var j = 0; j < nameList.length; j++){
            if(nameList[j].length < 1) continue;
            nameList[j] = nameList[j][0].toUpperCase() + nameList[j].substring(1);
          }
          ele.textContent = nameList.join(' ');
          y.appendChild(ele);
        }

        //y.className += " w3-show";
      });
    });
}
