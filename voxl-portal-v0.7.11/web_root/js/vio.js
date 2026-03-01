import * as THREE from './3rd_party/three.module.js';
import { OrbitControls } from './3rd_party/OrbitControls.js';
import { FontLoader} from './3rd_party/FontLoader.js';
import { TextGeometry} from './3rd_party/TextGeometry.js';
import { OrbitControlsGizmo } from  "./3rd_party/OrbitControlsGizmo.js";

let MAX_SOCKET_CONNECTIONS = 10;

// global elements
let perspectiveCamera, controls, scene, renderer, timerId, mesh_ws, mesh_curr_retries;

var vio_colors = [/*Green*/ '#4CAF50', /*Cyan*/ '#00ffff', /*Orange*/ '#ffa500', /*Purple*/ '#800080',
                    /*Blue*/ '#4a63c4', /*Gold*/ '#ffd700']

let vio_ws_array = [];

let default_color = 0; // black, 1 for white if dark mode

let MAX_SOCKET_RETRIES = 5;

// camera setup
const fov = 50;
const aspect = window.innerWidth/4 * 3 / window.innerHeight;
const near = 0.25;
const far = 1000;
perspectiveCamera = new THREE.PerspectiveCamera( fov, aspect, near, far );
perspectiveCamera.position.z = -1;
perspectiveCamera.position.x = 0;
perspectiveCamera.position.y = -20;
perspectiveCamera.up.set(0, 0, -1);
perspectiveCamera.rotation.set(0, 0, 0);

// materials
const blue_material = new THREE.LineBasicMaterial( { color: 0x0000ff, linewidth: 5     } );
const green_material = new THREE.LineBasicMaterial( { color: 0x00ff00, linewidth: 5    } );
const red_material = new THREE.LineBasicMaterial( { color: 0xff0000, linewidth: 5      } );
const black_material = new THREE.LineBasicMaterial( { color: 0x000000, linewidth: 1     } );
const thin_red_material = new THREE.LineBasicMaterial( {color: 0xff0000, linewidth: 1});

//dark mode button
const dark_mode = document.getElementById("mode_btn");

// scene setup
scene = new THREE.Scene();
scene.background = new THREE.Color( 0xcccccc );

var grid = new THREE.GridHelper(16, 36);
/*grid.geometry.rotateX( Math.PI );*/
/*grid.geometry.rotateX(Math.PI / 2);*/
// /grid.geometry.rotateZ(Math.PI);
//grid.geometry.rotateX(Math.PI / 2);
//grid.geometry.rotateZ(Math.PI / 2);
///grid.geometry.rotateX(Math.PI / 2);
grid.geometry.rotateX(Math.PI / 2);

console.log(grid.material.color.getHexString());

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

var stream_div = document.getElementById('stream_div');
stream_div.style.width = window.innerWidth / 4 + 'px';
stream_div.style.height = window.innerHeight + 'px';

// renderer setup
renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setPixelRatio( window.devicePixelRatio );
renderer.setSize( window.innerWidth/4 * 3, window.innerHeight );
var canvas = renderer.domElement;
canvas.style.float = 'right';

document.body.appendChild( canvas );
/*
var canvas_con = document.getElementById("canvas_container");
console.log(canvas_con);
canvas_con.appendChild(canvas);
*/

window.addEventListener( 'resize', onWindowResize )

createControls( perspectiveCamera );

const insetWidth = 300, insetHeight = 300;
let container2 = document.getElementById( 'inset' );
container2.width = insetWidth;
container2.height = insetHeight;

// renderer
/*
let renderer2 = new THREE.WebGLRenderer( { antialias: true } );
renderer2.setClearColor( 0x000000, 0 );
renderer2.setSize( insetWidth, insetHeight );
container2.appendChild( renderer2.domElement );
*/
const controlsGizmo = new OrbitControlsGizmo(controls, { size:  300, padding:  8 });
container2.appendChild(controlsGizmo.domElement);

// scene
/*
let scene2 = new THREE.Scene();
scene2.background = new THREE.Color( 0xcccccc );

// camera
let camera2 = new THREE.PerspectiveCamera( 50, insetWidth / insetHeight, 1, 1000 );
camera2.up = perspectiveCamera.up; // important!

// axes
let axes2 = new THREE.AxesHelper( 150 );
//axes2.geometry.position = new THREE.Vector3(0, Math.PI / 2, Math.PI / 2);
scene2.add( axes2 );
*/

animate();

//////////////////////////////////////////////////////////
// Websocket functions
//////////////////////////////////////////////////////////
var url_base = get_url_base();
var url_port = get_url_port();

function connect(next_ws) {

    next_ws.color = new THREE.Color(vio_colors[next_ws.c_index]);

    console.log("TOPIC: " + next_ws.topic);
    next_ws.ws_handle = new WebSocket("ws://" + url_base + ":" + url_port + "/vio/" + next_ws.topic);
    next_ws.ws_handle.binaryType = "arraybuffer";

    next_ws.ws_handle.onopen = function() {
        console.log("[INFO] Vio websocket open");
        next_ws.curr_retries = 0;
    };

    next_ws.num_packets = 0;
    next_ws.ws_handle.onmessage = function (evt) {
        next_ws.num_packets++;
        var received_msg = evt.data;
        //console.log(evt.data.byteLength);
        var parser = new jParser(received_msg, {
            vio_data_t: {
                magic_number: 'uint32',
                quality: 'int32',
                timestamp_ns: ['array', 'uint32', 2],
                T_imu_wrt_vio: ['array', 'float32', 3],
                R_imu_wrt_vio: ['array', 'float32', 9],
                pose_covariance: ['array', 'float32', 21],
                vel_imu_wrt_vio: ['array', 'float32', 3],
                velocity_covariance: ['array', 'float32', 21],
                imu_angular_vel: ['array', 'float32', 3],
                gravity_vector: ['array', 'float32', 3],
                T_cam_wrt_imu: ['array', 'float32', 3],
                R_cam_to_imu: ['array', 'float32', 9],
                error_code: 'uint32',
                n_feature_points: 'uint16',
                state: 'uint8',
                reserved: 'uint8'
            },
            vio_feature_t: {
                id: 'uint32',
                pix_loc: ['array', 'float32', 2],
                tsf: ['array', 'float32', 3],
                p_tsf: ['array', 'float32', 9],   // use feature cov for color, dif shapes per source
                depth: 'float32',
                depth_error_stddev: 'float32',
                point_quality:  'uint32'
            },
            ext_vio_data_t: {
                v: 'vio_data_t',
                last_cam_frame_id: 'int32',
                last_cam_timestamp_ns: ['array', 'uint32', 2],
                imu_cam_time_shift_s: 'float32',
                gravity_covariance: ['array', 'float32', 9],
                gyro_bias: ['array', 'float32', 3],
                accl_bias: ['array', 'float32', 3],
                n_total_features: 'uint32',
                features: ['array', 'vio_feature_t', 64]
            },
            pose_vel_6dof_t: {
                magic_number: 'uint32',
                timestamp_ns: ['array', 'uint32', 2],
                T_child_wrt_parent: ['array', 'float32', 3],
                R_child_to_parent: ['array', 'float32', 9],
                v_child_wrt_parent: ['array', 'float32', 3],
                w_child_wrt_child: ['array', 'float32', 3]
            },
        });

        if (received_msg.byteLength == 5268) {
            var do_features_only = false;
            next_ws.vio_feature_points = [];
            var vio_data = parser.parse('ext_vio_data_t');
            //console.log(vio_data);

            //const checked = check_data(vio_data)
            //console.log(checked);

            // debug
            // console.log(vio_data.v.R_cam_to_imu);
            // console.log(vio_data.v.T_cam_wrt_imu);

            var curr_pt = new THREE.Vector3(vio_data.v.T_imu_wrt_vio[0], vio_data.v.T_imu_wrt_vio[1], vio_data.v.T_imu_wrt_vio[2]);
            if (next_ws.previous_point != null){
                if (curr_pt.distanceTo(next_ws.previous_point) < .05){
                    do_features_only = true;
                }
            }

            for (var i = 0; i < 32; i++){
                next_ws.vio_feature_points.push(vio_data.features[i].tsf[0], vio_data.features[i].tsf[1], vio_data.features[i].tsf[2]);
            }

            scene.remove(next_ws.scene_pose_x);
            scene.remove(next_ws.scene_pose_y);
            scene.remove(next_ws.scene_pose_z);

            scene.remove(next_ws.cam0_wrt_vio_x);
            scene.remove(next_ws.cam0_wrt_vio_y);
            scene.remove(next_ws.cam0_wrt_vio_z);

            if (!do_features_only){
                next_ws.previous_point = curr_pt;
                next_ws.vio_raw_points.push(curr_pt);
            }

            const p_x = [];
            const p_y = [];
            const p_z = [];

            p_x.push( new THREE.Vector3( 0,       0,       0));
            p_y.push( new THREE.Vector3( 0,       0,       0));
            p_z.push( new THREE.Vector3( 0,       0,       0));
            p_x.push( new THREE.Vector3( 0+0.175, 0,       0));
            p_y.push( new THREE.Vector3( 0,       0+0.175, 0));
            p_z.push( new THREE.Vector3( 0,       0,       0+0.175));

            const geometry_x = new THREE.BufferGeometry().setFromPoints( p_x );
            const geometry_y = new THREE.BufferGeometry().setFromPoints( p_y );
            const geometry_z = new THREE.BufferGeometry().setFromPoints( p_z );

            const geometry_x_ = geometry_x.clone();
            const geometry_y_ = geometry_y.clone();
            const geometry_z_ = geometry_z.clone();

            var m = new THREE.Matrix4();
            m.set(  vio_data.v.R_imu_wrt_vio[0], vio_data.v.R_imu_wrt_vio[1], vio_data.v.R_imu_wrt_vio[2], vio_data.v.T_imu_wrt_vio[0],
                vio_data.v.R_imu_wrt_vio[3], vio_data.v.R_imu_wrt_vio[4], vio_data.v.R_imu_wrt_vio[5], vio_data.v.T_imu_wrt_vio[1],
                vio_data.v.R_imu_wrt_vio[6], vio_data.v.R_imu_wrt_vio[7], vio_data.v.R_imu_wrt_vio[8], vio_data.v.T_imu_wrt_vio[2],
                0,                         0,                         0,                         1);

            geometry_x.applyMatrix4(m);
            geometry_y.applyMatrix4(m);
            geometry_z.applyMatrix4(m);

            next_ws.scene_pose_x = new THREE.Line( geometry_x, red_material );
            next_ws.scene_pose_y = new THREE.Line( geometry_y, green_material );
            next_ws.scene_pose_z = new THREE.Line( geometry_z, blue_material );

            var m_ = new THREE.Matrix4();
            m_.set(  vio_data.v.R_cam_to_imu[0], vio_data.v.R_cam_to_imu[1], vio_data.v.R_cam_to_imu[2], vio_data.v.T_cam_wrt_imu[0],
                    vio_data.v.R_cam_to_imu[3], vio_data.v.R_cam_to_imu[4], vio_data.v.R_cam_to_imu[5], vio_data.v.T_cam_wrt_imu[1],
                    vio_data.v.R_cam_to_imu[6], vio_data.v.R_cam_to_imu[7], vio_data.v.R_cam_to_imu[8], vio_data.v.T_cam_wrt_imu[2],
                    0,                         0,                         0,                         1);

            // cam to imu
            geometry_x_.applyMatrix4(m_);
            geometry_y_.applyMatrix4(m_);
            geometry_z_.applyMatrix4(m_);

            // imu to vio
            geometry_x_.applyMatrix4(m);
            geometry_y_.applyMatrix4(m);
            geometry_z_.applyMatrix4(m);

            next_ws.cam0_wrt_vio_x = new THREE.Line( geometry_x_, red_material );
            next_ws.cam0_wrt_vio_y = new THREE.Line( geometry_y_, green_material );
            next_ws.cam0_wrt_vio_z = new THREE.Line( geometry_z_, blue_material );

            //scene.add(next_ws.cam0_wrt_vio_x);
            //scene.add(next_ws.cam0_wrt_vio_y);
            //scene.add(next_ws.cam0_wrt_vio_z);
            scene.add(next_ws.scene_pose_x);
            scene.add(next_ws.scene_pose_y);
            scene.add(next_ws.scene_pose_z);

            if (next_ws.num_packets == 1){
                var loader = new FontLoader();
                loader.load( 'fonts/helvetiker_regular.typeface.json', function ( font ) {

                    var imu_text_geom = new TextGeometry(next_ws.topic, {
                    font: font,
                    size: 0.1,
                    height: 0,
                    curveSegments: 12,
                    bevelEnabled: false
                    });

                    var imu_text_material = new THREE.MeshPhongMaterial(
                    { color: next_ws.color, specular: 0xffffff }
                    );

                    next_ws.imu_wrt_vio_label = new THREE.Mesh( imu_text_geom, imu_text_material );

                    
                    var cam_text_geom = new TextGeometry(next_ws.topic, {
                        font: font,
                        size: 0.1,
                        height: 0,
                        curveSegments: 12,
                        bevelEnabled: false
                    });
                    

                    var cam_text_material = new THREE.MeshPhongMaterial(
                    { color: next_ws.color, specular: 0xffffff }
                    );

                    next_ws.cam0_wrt_vio_label = new THREE.Mesh( cam_text_geom, cam_text_material );

                    next_ws.imu_wrt_vio_label.position.set(curr_pt.x, curr_pt.y, curr_pt.z);
                    next_ws.cam0_wrt_vio_label.position.set(geometry_y_.attributes.position.array[0], geometry_y_.attributes.position.array[1]+0.1, geometry_y_.attributes.position.array[2]);
                    next_ws.imu_wrt_vio_label.lookAt( perspectiveCamera.position );
                    next_ws.cam0_wrt_vio_label.lookAt( perspectiveCamera.position );

                    scene.add(next_ws.imu_wrt_vio_label);
                    //scene.add(  next_ws.cam0_wrt_vio_label );
                });
            }
            else if (next_ws.imu_wrt_vio_label != null) {
                next_ws.imu_wrt_vio_label.position.set(curr_pt.x, curr_pt.y, curr_pt.z);
                next_ws.cam0_wrt_vio_label.position.set(geometry_y_.attributes.position.array[0], geometry_y_.attributes.position.array[1]+0.1, geometry_y_.attributes.position.array[2]);
                next_ws.imu_wrt_vio_label.geometry.buffersNeedUpdate = true;
                next_ws.imu_wrt_vio_label.geometry.attributes.position.needsUpdate = true;
                next_ws.cam0_wrt_vio_label.geometry.buffersNeedUpdate = true;
                next_ws.cam0_wrt_vio_label.geometry.attributes.position.needsUpdate = true;
                next_ws.imu_wrt_vio_label.quaternion.copy(perspectiveCamera.quaternion);
                next_ws.cam0_wrt_vio_label.quaternion.copy(perspectiveCamera.quaternion);
            }

            if (!do_features_only){
                if (next_ws.vio_path == null){
                    var temp_geometry = new THREE.BufferGeometry().setFromPoints(next_ws.vio_raw_points);
                    // NOTE linewidth does not set in many browsers, would need to use a custom material and more js modules
                    //console.log(next_ws.color);
                    var line_material = new THREE.LineBasicMaterial({color: next_ws.color, linewidth: 4.5});

                    next_ws.vio_path = new THREE.Line( temp_geometry, line_material );
                }
                else {
                    next_ws.vio_path.geometry.setFromPoints(next_ws.vio_raw_points);
                    next_ws.vio_path.geometry.buffersNeedUpdate = true;
                    next_ws.vio_path.geometry.attributes.position.needsUpdate = true;
                }
                scene.add(next_ws.vio_path);
            }

            if (next_ws.vio_pointcloud == null){
                var pt_geometry = new THREE.BufferGeometry();
                pt_geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( next_ws.vio_feature_points, 3 ) );

                var material = new THREE.PointsMaterial( { size: 0.1, color: next_ws.color, vertexColors: false } );

                next_ws.vio_pointcloud = new THREE.Points( pt_geometry, material );
            }
            else {
                next_ws.vio_pointcloud.geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( next_ws.vio_feature_points, 3 ) );
                next_ws.vio_pointcloud.geometry.buffersNeedUpdate = true;
                next_ws.vio_pointcloud.geometry.attributes.position.needsUpdate = true;
            }

            scene.add(next_ws.vio_pointcloud);
    
        }
        else if (received_msg.byteLength == 324) {
            var vio_data = parser.parse('vio_data_t');

            var curr_pt = new THREE.Vector3(vio_data.T_imu_wrt_vio[0], vio_data.T_imu_wrt_vio[1], vio_data.T_imu_wrt_vio[2])

            scene.remove(next_ws.scene_pose_x);
            scene.remove(next_ws.scene_pose_y);
            scene.remove(next_ws.scene_pose_z);

            scene.remove(next_ws.cam0_wrt_vio_x);
            scene.remove(next_ws.cam0_wrt_vio_y);
            scene.remove(next_ws.cam0_wrt_vio_z);

            next_ws.previous_point = curr_pt;
            next_ws.vio_raw_points.push(curr_pt);
            
            const p_x = [];
            const p_y = [];
            const p_z = [];

            p_x.push( new THREE.Vector3( 0,       0,       0));
            p_y.push( new THREE.Vector3( 0,       0,       0));
            p_z.push( new THREE.Vector3( 0,       0,       0));
            p_x.push( new THREE.Vector3( 0+0.175, 0,       0));
            p_y.push( new THREE.Vector3( 0,       0+0.175, 0));
            p_z.push( new THREE.Vector3( 0,       0,       0+0.175));

            const geometry_x = new THREE.BufferGeometry().setFromPoints( p_x );
            const geometry_y = new THREE.BufferGeometry().setFromPoints( p_y );
            const geometry_z = new THREE.BufferGeometry().setFromPoints( p_z );

            const geometry_x_ = geometry_x.clone();
            const geometry_y_ = geometry_y.clone();
            const geometry_z_ = geometry_z.clone();

            var m = new THREE.Matrix4();
            m.set(  vio_data.R_imu_wrt_vio[0], vio_data.R_imu_wrt_vio[1], vio_data.R_imu_wrt_vio[2], vio_data.T_imu_wrt_vio[0],
                vio_data.R_imu_wrt_vio[3], vio_data.R_imu_wrt_vio[4], vio_data.R_imu_wrt_vio[5], vio_data.T_imu_wrt_vio[1],
                vio_data.R_imu_wrt_vio[6], vio_data.R_imu_wrt_vio[7], vio_data.R_imu_wrt_vio[8], vio_data.T_imu_wrt_vio[2],
                0,                         0,                         0,                         1);

            geometry_x.applyMatrix4(m);
            geometry_y.applyMatrix4(m);
            geometry_z.applyMatrix4(m);

            next_ws.scene_pose_x = new THREE.Line( geometry_x, red_material );
            next_ws.scene_pose_y = new THREE.Line( geometry_y, green_material );
            next_ws.scene_pose_z = new THREE.Line( geometry_z, blue_material );

            var m_ = new THREE.Matrix4();
            m_.set(  vio_data.R_cam_to_imu[0], vio_data.R_cam_to_imu[1], vio_data.R_cam_to_imu[2], vio_data.T_cam_wrt_imu[0],
                    vio_data.R_cam_to_imu[3], vio_data.R_cam_to_imu[4], vio_data.R_cam_to_imu[5], vio_data.T_cam_wrt_imu[1],
                    vio_data.R_cam_to_imu[6], vio_data.R_cam_to_imu[7], vio_data.R_cam_to_imu[8], vio_data.T_cam_wrt_imu[2],
                    0,                         0,                         0,                         1);

            // cam to imu
            geometry_x_.applyMatrix4(m_);
            geometry_y_.applyMatrix4(m_);
            geometry_z_.applyMatrix4(m_);

            // imu to vio
            geometry_x_.applyMatrix4(m);
            geometry_y_.applyMatrix4(m);
            geometry_z_.applyMatrix4(m);

            next_ws.cam0_wrt_vio_x = new THREE.Line( geometry_x_, red_material );
            next_ws.cam0_wrt_vio_y = new THREE.Line( geometry_y_, green_material );
            next_ws.cam0_wrt_vio_z = new THREE.Line( geometry_z_, blue_material );

            scene.add(next_ws.scene_pose_x);
            scene.add(next_ws.scene_pose_y);
            scene.add(next_ws.scene_pose_z);

            if (next_ws.num_packets == 1) {
                var loader = new FontLoader();
                loader.load( 'fonts/helvetiker_regular.typeface.json', function ( font ) {

                    var imu_text_geom = new TextGeometry( next_ws.topic, {
                    font: font,
                    size: 0.1,
                    height: 0,
                    curveSegments: 12,
                    bevelEnabled: false
                    });

                    var imu_text_material = new THREE.MeshPhongMaterial(
                    { color: next_ws.color, specular: 0xffffff }
                    );

                    next_ws.imu_wrt_vio_label = new THREE.Mesh( imu_text_geom, imu_text_material );

                    var cam_text_geom = new TextGeometry( "", {
                        font: font,
                        size: 0.1,
                        height: 0,
                        curveSegments: 12,
                        bevelEnabled: false
                    });
                    

                    var cam_text_material = new THREE.MeshPhongMaterial(
                    { color: next_ws.color, specular: 0xffffff }
                    );

                    next_ws.cam0_wrt_vio_label = new THREE.Mesh( cam_text_geom, cam_text_material );
                    
                    next_ws.imu_wrt_vio_label.position.set(curr_pt.x, curr_pt.y, curr_pt.z);
                    next_ws.imu_wrt_vio_label.lookAt( perspectiveCamera.position );
                    next_ws.cam0_wrt_vio_label.position.set(geometry_y_.attributes.position.array[0], geometry_y_.attributes.position.array[1]+0.1, geometry_y_.attributes.position.array[2]);
                    next_ws.cam0_wrt_vio_label.lookAt(perspectiveCamera.position);

                    scene.add(next_ws.imu_wrt_vio_label );
                });

            }
            else if (next_ws.imu_wrt_vio_label != null) {
                next_ws.imu_wrt_vio_label.position.set(curr_pt.x, curr_pt.y, curr_pt.z);
                next_ws.imu_wrt_vio_label.geometry.buffersNeedUpdate = true;
                next_ws.imu_wrt_vio_label.geometry.attributes.position.needsUpdate = true;
                next_ws.imu_wrt_vio_label.quaternion.copy(perspectiveCamera.quaternion);
                next_ws.cam0_wrt_vio_label.position.set(geometry_y_.attributes.position.array[0], geometry_y_.attributes.position.array[1]+0.1, geometry_y_.attributes.position.array[2]);
                next_ws.cam0_wrt_vio_label.geometry.buffersNeedUpdate = true;
                next_ws.cam0_wrt_vio_label.geometry.attributes.position.needsUpdate = true;
                next_ws.cam0_wrt_vio_label.quaternion.copy(perspectiveCamera.quaternion);

            }

            if (next_ws.vio_path == null){
                var temp_geometry = new THREE.BufferGeometry().setFromPoints(next_ws.vio_raw_points);
                // NOTE linewidth does not set in many browsers, would need to use a custom material and more js modules
                //console.log(next_ws.color);
                var line_material = new THREE.LineBasicMaterial({color: next_ws.color, linewidth: 4.5});

                next_ws.vio_path = new THREE.Line( temp_geometry, line_material );
            }
            else {
                next_ws.vio_path.geometry.setFromPoints(next_ws.vio_raw_points);
                next_ws.vio_path.geometry.buffersNeedUpdate = true;
                next_ws.vio_path.geometry.attributes.position.needsUpdate = true;
            }
            scene.add(next_ws.vio_path);
        }
        else if (received_msg.byteLength == 84) {
            var vio_data = parser.parse('pose_vel_6dof_t');
            //console.log(vio_data);

            var curr_pt = new THREE.Vector3(vio_data.T_child_wrt_parent[0], vio_data.T_child_wrt_parent[1], vio_data.T_child_wrt_parent[2]);
            
            scene.remove(next_ws.scene_pose_x);
            scene.remove(next_ws.scene_pose_y);
            scene.remove(next_ws.scene_pose_z);
            
            next_ws.previous_point = curr_pt;
            next_ws.vio_raw_points.push(curr_pt);

            const p_x = [];
            const p_y = [];
            const p_z = [];

            p_x.push( new THREE.Vector3( 0,       0,       0));
            p_y.push( new THREE.Vector3( 0,       0,       0));
            p_z.push( new THREE.Vector3( 0,       0,       0));
            p_x.push( new THREE.Vector3( 0+0.175, 0,       0));
            p_y.push( new THREE.Vector3( 0,       0+0.175, 0));
            p_z.push( new THREE.Vector3( 0,       0,       0+0.175));

            const geometry_x = new THREE.BufferGeometry().setFromPoints( p_x );
            const geometry_y = new THREE.BufferGeometry().setFromPoints( p_y );
            const geometry_z = new THREE.BufferGeometry().setFromPoints( p_z );

            //const geometry_x_ = geometry_x.clone();
            //const geometry_y_ = geometry_y.clone();
            //const geometry_z_ = geometry_z.clone();

            var m = new THREE.Matrix4();
            m.set(vio_data.R_child_to_parent[0], vio_data.R_child_to_parent[1], vio_data.R_child_to_parent[2], vio_data.T_child_wrt_parent[0], 
                  vio_data.R_child_to_parent[3], vio_data.R_child_to_parent[4], vio_data.R_child_to_parent[5], vio_data.T_child_wrt_parent[1], 
                  vio_data.R_child_to_parent[6], vio_data.R_child_to_parent[7], vio_data.R_child_to_parent[8], vio_data.T_child_wrt_parent[2], 
                  0, 0, 0, 1);

            geometry_x.applyMatrix4(m);
            geometry_y.applyMatrix4(m);
            geometry_z.applyMatrix4(m);

            next_ws.scene_pose_x = new THREE.Line( geometry_x, red_material );
            next_ws.scene_pose_y = new THREE.Line( geometry_y, green_material );
            next_ws.scene_pose_z = new THREE.Line( geometry_z, blue_material );

            scene.add(next_ws.scene_pose_x);
            scene.add(next_ws.scene_pose_y);
            scene.add(next_ws.scene_pose_z);

            if (next_ws.num_packets == 1) {
                console.log("6dof name change");
                var loader = new FontLoader();
                loader.load( 'fonts/helvetiker_regular.typeface.json', function ( font ) {
                    var child_to_parent = new TextGeometry(next_ws.topic, {
                        font: font,
                        size: 0.1,
                        height: 0,
                        curveSegments: 12,
                        bevelEnabled: false,
                    });

                    var child_to_parent_text_material = new THREE.MeshPhongMaterial({color : next_ws.color, specular: 0xffffff});

                    next_ws.child_wrt_parent_label = new THREE.Mesh(child_to_parent, child_to_parent_text_material);
                    next_ws.child_wrt_parent_label.position.set(curr_pt.x, curr_pt.y, curr_pt.z);
                    next_ws.child_wrt_parent_label.lookAt(perspectiveCamera.position);

                    scene.add(next_ws.child_wrt_parent_label);
                });
            }
            else if (next_ws.child_wrt_parent_label != null) {
                next_ws.child_wrt_parent_label.position.set(curr_pt.x, curr_pt.y, curr_pt.z);
                next_ws.child_wrt_parent_label.geometry.buffersNeedUpdate = true;
                next_ws.child_wrt_parent_label.geometry.attributes.position.needsUpdate = true;
                next_ws.child_wrt_parent_label.quaternion.copy(perspectiveCamera.quaternion);
            }

            if (next_ws.vio_path == null){
                var temp_geometry = new THREE.BufferGeometry().setFromPoints(next_ws.vio_raw_points);
                // NOTE linewidth does not set in many browsers, would need to use a custom material and more js modules
                //console.log(next_ws.color);
                var line_material = new THREE.LineBasicMaterial({color: next_ws.color, linewidth: 4.5});

                next_ws.vio_path = new THREE.Line( temp_geometry, line_material );
            }
            else {
                next_ws.vio_path.geometry.setFromPoints(next_ws.vio_raw_points);
                next_ws.vio_path.geometry.buffersNeedUpdate = true;
                next_ws.vio_path.geometry.attributes.position.needsUpdate = true;
            }
            scene.add(next_ws.vio_path);
        }
    } 

    next_ws.ws_handle.onclose = function(e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        next_ws.curr_retries++;
        if (next_ws.curr_retries <= MAX_SOCKET_RETRIES)
            setTimeout(function() {
                connect(next_ws);
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    next_ws.ws_handle.onerror = function(err) {
      console.error('Socket encountered error: ', err.message, 'Closing socket');
      next_ws.ws_handle.close();
    };

    vio_ws_array.push(next_ws);
  }

updateVioTopicList();
keepAlive();

function addTopic(topic_name, c_index){
    //updateVioTopicList();
    var next_ws = {
        'topic' : topic_name,
        'c_index' : c_index,
        'ws_handle' : null,
        'scene_pose_x' : null,
        'scene_pose_y' : null,
        'scene_pose_z' : null,
        'cam0_wrt_vio_x' : null,
        'cam0_wrt_vio_y' : null,
        'cam0_wrt_vio_z' : null,
        'cam1_wrt_vio_x' : null,
        'cam1_wrt_vio_y' : null,
        'cam1_wrt_vio_z' : null,
        'imu_wrt_vio_label' : null,
        'cam0_wrt_vio_label' : null,
        'cam1_wrt_vio_label' : null,
        'child_wrt_parent_label' : null, 
        'vio_path' : null,
        'previous_point' : null,
        'vio_raw_points' : [],
        'vio_feature_points' : [],
        'vio_pointcloud' : null,
        'curr_retries' : 0,
        'color' : 0x0000ff,
        'num_packets': null,
    }
    connect(next_ws);
}

function clearTopic(topic_name){
    for (var i = 0; i < vio_ws_array.length; i++){
        if (vio_ws_array[i].topic == topic_name){
            console.log("CLOSING: " + vio_ws_array[i].topic);

            
            scene.remove(vio_ws_array[i].scene_pose_x);
            scene.remove(vio_ws_array[i].scene_pose_y);
            scene.remove(vio_ws_array[i].scene_pose_z);
            scene.remove(vio_ws_array[i].cam0_wrt_vio_x);
            scene.remove(vio_ws_array[i].cam0_wrt_vio_y);
            scene.remove(vio_ws_array[i].cam0_wrt_vio_z);
            scene.remove(vio_ws_array[i].cam1_wrt_vio_x);
            scene.remove(vio_ws_array[i].cam1_wrt_vio_y);
            scene.remove(vio_ws_array[i].cam1_wrt_vio_z);
            scene.remove(vio_ws_array[i].imu_wrt_vio_label);
            scene.remove(vio_ws_array[i].cam0_wrt_vio_label);
            scene.remove(vio_ws_array[i].cam1_wrt_vio_label);
            scene.remove(vio_ws_array[i].child_wrt_parent_label);

            scene.remove(vio_ws_array[i].vio_path);
            scene.remove(vio_ws_array[i].vio_pointcloud);
            
            //console.log(scene.children);
            /*
            for (var key in vio_ws_array[i]) {
                if (vio_ws_array[i][key] != null && vio_ws_array[i][key].parent != null) {
                    vio_ws_array[i][key].removeFromParent();
                    scene.remove(vio_ws_array[i][key])
                }
            }
            console.log(scene.children);
            */
            
            /*
            console.log("PARENT: " + vio_ws_array[i].cam0_wrt_vio_label.parent);

            vio_ws_array[i].imu_wrt_vio_label.removeFromParent();
            vio_ws_array[i].cam0_wrt_vio_label.removeFromParent();
            vio_ws_array[i].cam1_wrt_vio_label.removeFromParent();

            console.log("PARENT: " + vio_ws_array[i].imu_wrt_vio_label.parent);
            */

            vio_ws_array[i].vio_path = null;
            vio_ws_array[i].vio_raw_points = [];
            vio_ws_array[i].imu_wrt_vio_label = null;
            vio_ws_array[i].cam0_wrt_vio_label = null;
            vio_ws_array[i].cam1_wrt_vio_label = null;
            vio_ws_array[i].child_wrt_parent_label = null;

            vio_ws_array[i].vio_raw_points = [];
            vio_ws_array[i].vio_feature_points = [];

            vio_ws_array[i].ws_handle.onclose = null;
            vio_ws_array[i].ws_handle.close();
            vio_ws_array.splice(i,1);
            return;
        }
    }
}

function keepAlive(timeout = 5000) {
    for (var i = 0; i < vio_ws_array.length; i++){
        if (vio_ws_array[i].ws_handle.readyState == vio_ws_array[i].ws_handle.OPEN) {
            vio_ws_array[i].ws_handle.send('');
        }
    }
    timerId = setTimeout(keepAlive, timeout);
}

//////////////////////////////////////////////////////////
// Handle all of our buttons
//////////////////////////////////////////////////////////
const home = document.getElementById('home');
const stream_add_btn = document.getElementById('stream_add');
const load_map_btn = document.getElementById('load_action');
const load_form = document.getElementById('load_form');
const load_file_path = document.getElementById('load_file_name');
const load_form_sub_btn = document.getElementById('load_sub_action');
const load_back_btn = document.getElementById("load_back");
const clear_traj_btn = document.getElementById("clear_traj");
const mode_btn = document.getElementById("mode_btn");
const reset_qvio_btn = document.getElementById("reset_qvio");
const reset_ov_btn = document.getElementById("reset_ov");

clear_traj_btn.addEventListener("click", () => {
    for (var i = 0; i < vio_ws_array.length; i++){
        scene.remove(vio_ws_array[i].vio_path);
        vio_ws_array[i].vio_path = null;
        vio_ws_array[i].vio_raw_points = [];
    }
})

load_map_btn.addEventListener("click", () => {
    load_map_btn.classList.replace("w3-show", "w3-hide");
    load_form.style.display = 'block';
})

load_back_btn.addEventListener("click", () => {
    load_map_btn.classList.replace("w3-hide", "w3-show");
    load_form.style.display = 'none';
})

load_form_sub_btn.addEventListener("click", () => {
    var file_path = load_file_path.value;
    file_path = file_path.replace(/\s+/g, '');
    file_path = file_path.replaceAll('/', '%2F');

    // console.log(file_path);

    fetch("/log/" + file_path).then(function(response) {
      console.log("fetched");
    })

    load_map_btn.classList.replace("w3-hide", "w3-show");
    load_form.style.display = 'none';

})

mode_btn.addEventListener("click", () => {
    if (mode_btn.classList.contains("w3-black")){
        mode_btn.className = "w3-button w3-block w3-white";
        scene.background = new THREE.Color( 0x000000 );
        //scene2.background = new THREE.Color( 0x000000 );
        stream_div.style.background = "#000000";
        default_color = 1;
        tick_markers.forEach((tick_marker) => {
            tick_marker.material.color.set(0xcccccc);
        })
        z_marker.material.color.set(0xcccccc);
        label_material.material.color.set(0x000000);
    }
    else {
        mode_btn.className = "w3-button w3-block w3-black";
        scene.background = new THREE.Color( 0xcccccc );
        //scene2.background = new THREE.Color( 0xcccccc );
        stream_div.style.background = "#cccccc";
        default_color = 0;
        tick_markers.forEach((tick_marker) => {
            tick_marker.material.color.set(0x000000);
        })
        z_marker.material.color.set(0x000000);
        label_material.material.color.set(0x000000);
    }
})

mode_btn.click();

reset_qvio_btn.addEventListener("click", () => {
    var reset_qvio_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/reset_qvio/");
    reset_qvio_ws.onopen = function() {
        console.log("Reset QVIO");
    }
})

reset_ov_btn.addEventListener("click", function() {
    var reset_ov_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/reset_ov/");
    reset_ov_ws.onopen = function() {
        console.log("Reset OV");
    }
})

function createControls( camera ) {
    controls = new OrbitControls(camera, canvas);

    controls.zoomSpeed = 1.2;
    controls.panSpeed = 0.8;
    controls.rotateSpeed = 1;
}

function onWindowResize() {

    const aspect = window.innerWidth * 3 / 4 / window.innerHeight;

    perspectiveCamera.aspect = aspect;
    perspectiveCamera.updateProjectionMatrix();

    renderer.setSize( window.innerWidth * 3 / 4, window.innerHeight );

    //controls.handleResize();

    stream_div.style.width = window.innerWidth / 4 + 'px';
    stream_div.style.height = window.innerHeight + 'px';
}

function animate() {

    requestAnimationFrame( animate );

    controls.update();

    //copy position of the camera into inset
    /*
    camera2.position.copy( perspectiveCamera.position );
    camera2.position.sub( controls.target );
    camera2.position.setLength( 300 );
    camera2.lookAt( scene2.position );
    */

    renderer.render( scene, perspectiveCamera );
    //renderer2.render( scene2, camera2 );
}

function closeCameraAcc(){

    //Close and remove the green coloring of the dropdown
    var x = document.getElementById("streamAcc");
    x.className = x.className.replace("w3-show", "w3-hide");

    //Remove all the children, they should get repopulated to ensure no faulty cameras
    //var eles = document.getElementsByClassName("cam-item");
    //while(eles[0]) {
      //eles[0].parentNode.removeChild(eles[0]);
    //}

}

var pchild = document.getElementById("dropdown_vertical_streamAcc");
pchild.style.maxWidth = window.innerWidth + "px";

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
          if(!cams[i].includes("overlay")) continue;
          
          var ele = document.createElement("a");
          ele.style.flexWrap = "wrap";
          ele.setAttribute("id",       "btn_"+cams[i]);
          ele.setAttribute("pipe",     cams[i]);
          ele.addEventListener("click", function(e) {
            var imgString = "/video_raw/"+this.getAttribute("pipe");
            // now instead of just assigning properties to this guy, we need to create the whole thing
            var curr_str_div = document.createElement("div");
            curr_str_div.style.width = "265px";
            curr_str_div.style.height = "237px";
            curr_str_div.classList.add("resizable");
            
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
                
                
                if (pos_x > 0 && pos_y > 0) {
                    console.log("DRAGGING");
                    document.addEventListener("mousemove", drag);
                    document.addEventListener("mouseup", stop_drag);
                    e.preventDefault();
                }
                else {
                    //curr_str_div.style.position = "absolute"
                }
            }

            curr_str_div.addEventListener("mousedown", move_or_resize);

            var curr_str_img = document.createElement("img");

            curr_str_img.setAttribute("src", imgString);
            curr_str_img.className = "";

            var curr_str_rem_btn = document.createElement("button");
            curr_str_rem_btn.classList.add("w3-red", "w3-button");
            curr_str_rem_btn.style.position = "absolute";
            curr_str_rem_btn.style.top = '5px';
            curr_str_rem_btn.style.right = '5px';
            curr_str_rem_btn.style.opacity = '40%';
            curr_str_rem_btn.innerHTML = "x";
            curr_str_rem_btn.setAttribute("cam_name", this.getAttribute("pipe"));

            curr_str_rem_btn.addEventListener("click", () => {
                stream_div.removeChild(curr_str_div);
                curr_str_img.src = "";
                const url = window.location.href;
                const url_partial = url.split("/");
                //var close_cam_ws = new WebSocket("ws://" + url_partial[2] + ":80/vio_close/" + curr_str_rem_btn.getAttribute("cam_name"));
                //close_cam_ws.onopen = function() {
                    //console.log("Attempting to close: " + curr_str_rem_btn.getAttribute("cam_name"));
                //}
            })

            var curr_str_cam_name = document.createElement("h3");
            curr_str_cam_name.textContent = this.textContent;
            curr_str_cam_name.style.color = "white";
            curr_str_cam_name.style.textAlign = "center";

            curr_str_div.append(curr_str_cam_name);
            curr_str_div.appendChild(curr_str_img);
            curr_str_div.appendChild(curr_str_rem_btn);

            document.getElementById("stream_div").appendChild(curr_str_div);
            //closeCameraAcc();
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

function updateVioTopicList(){
    const url_1 = window.location.href;
    const url_partial_1 = url_1.split("/");
    var extract_features_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/extract/");
    extract_features_ws.onopen = function() {
        console.log("CALLING VOXL-LIST-PIPES");
    }
    var options_div = document.getElementById("options_div");
    var c_index = 0;
    extract_features_ws.onmessage = function(event) {
        console.log(event.data);
        var ele = document.createElement("button");
        ele.setAttribute("pipe", event.data);
        ele.setAttribute("color", vio_colors[c_index]);
        ele.setAttribute("c_index", c_index);


        ele.addEventListener("click", function(e) {
            for (var a = 0; a < options_div.childNodes.length; a++){
                options_div.childNodes[a].disabled = true;
            }
            if (this.classList.contains("w3-red")){
                this.classList.remove("w3-red");
                this.style.background = this.getAttribute("color");
                addTopic(this.getAttribute("pipe"), this.getAttribute("c_index"));
            }
            else {
                this.classList.add("w3-red");
                clearTopic(this.getAttribute("pipe"))
            }
            setTimeout(function() {
                for (var a = 0; a < options_div.childNodes.length; a++){
                    options_div.childNodes[a].disabled = false;
                }
            }, 250);

        });

        c_index+=1;

        ele.classList.add("w3-button");
        ele.classList.add("w3-red");
        ele.classList.add("w3-block");

        ele.classList.add("w3-padding-large");

        var msg = event.data;
        msg = msg[0].toUpperCase() + msg.substring(1);
        //console.log(msg);
        ele.textContent = msg;
        //console.log(ele.textContent)
        options_div.appendChild(ele);
        if (event.data.includes("qvio_extended")) {
            ele.click();
        }
        if (event.data.includes("ov_extended")) {
            ele.click();
        }
    }
}

document.getElementById('dark_mode').classList.remove("w3-hide");
