// import * as THREE from 'three';
// import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
// import { FlyControls } from './3rd_party/FlyControls.js';
// import { FontLoader} from './3rd_party/FontLoader.js';
// import { TextGeometry} from './3rd_party/TextGeometry.js';
// import { TransformControls } from './3rd_party/TransformControls.js';
// import {
//     init as initRecastNavigation,
//     NavMeshQuery,
// } from '@recast-navigation/core';
// import { generateSoloNavMesh } from '@recast-navigation/generators';
// import {
//     DebugDrawer,
//     getPositionsAndIndices,
// } from '@recast-navigation/three';

import * as THREE from './3rd_party/pfe_mapper/three.module.js';
import { OrbitControls } from './3rd_party/pfe_mapper/OrbitControls.js';
import { FlyControls } from './3rd_party/FlyControls.js';
import { FontLoader} from './3rd_party/FontLoader.js';
import { TextGeometry} from './3rd_party/TextGeometry.js';
import { TransformControls } from './3rd_party/TransformControls.js';
import {
    init as initRecastNavigation,
} from './3rd_party/pfe_mapper/recast-navigation-core.js';
import { PFE_Pathfinder } from './pfe_mapper.js';



const pointcloud_format = {
    FLOAT_XYZ: 0,
    FLOAT_XYZC: 1,
    FLOAT_XYZRGB: 2,
    FLOAT_XYZCRGB: 3,
    FLOAT_XY: 4,
    FLOAT_XYC: 5
};

// Create enum for handling controls
const ControlsType = Object.freeze({
    Orbit: 0,
    Fly: 1
})

//---------PFE--------
await initRecastNavigation();
//--------------------

//////////////////////////////////////////////////////////
// Scene Setup
//////////////////////////////////////////////////////////
// Default up needs to align with NED of quadcopter
THREE.Object3D.DefaultUp = new THREE.Vector3(0,0,-1);
let clock = new THREE.Clock();
let prev_control;
let fpv_mode = false;

// this needs to be here, scene setup references this guy to know what to show
const two_d = document.getElementById('2d');

// globals for threejs elements
let perspectiveCamera, controls, scene, renderer;

// materials
const black_material = new THREE.LineBasicMaterial( { color: 0x000000, linewidth: 1     } );

// scene setup
scene = new THREE.Scene();
scene.background = new THREE.Color(0xcccccc);

var grid = new THREE.GridHelper(25, 50);
grid.geometry.rotateX(Math.PI / 2);

// scene.add(grid);

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
scene.add(light);

// renderer setup
const height_offset = document.getElementById("header").getBoundingClientRect().bottom;
renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight - height_offset);
var canvas = renderer.domElement;

document.body.appendChild(renderer.domElement);

window.addEventListener('resize', onWindowResize);


// camera setup
const fov = 50;
const aspect = window.innerWidth / window.innerHeight;
const near = 0.1;
const far = 500;
perspectiveCamera = new THREE.PerspectiveCamera(fov, aspect, near, far);
//perspectiveCamera.position.z = -1;
//perspectiveCamera.position.x = 0;
//perspectiveCamera.position.y = -20;

createControls(ControlsType.Orbit);
controls.enableRotate = false; // for costmap view

let transform_helper = new TransformControls(perspectiveCamera, renderer.domElement);
transform_helper.addEventListener( 'change', render );
transform_helper.addEventListener( 'dragging-changed', function ( event ) {
    controls.enabled = ! event.value;
} );
transform_helper.setSize(0.5);

const insetWidth = 300, insetHeight = 300;
let container2 = document.getElementById('inset');
container2.width = insetWidth;
container2.height = insetHeight;

//-------PFE-----------
const pfe_get_path_btn = document.getElementById('get_path_btn');
const pfe_send_path_btn = document.getElementById('send_path_btn');
const pfe_add_calibration_point_btn = document.getElementById('add_calibration_point_btn');
const pfe_calibrate_btn = document.getElementById('calibrate_btn');

const pfe_pathfinder = new PFE_Pathfinder(window.innerHeight - height_offset, window.innerWidth, height_offset, scene, perspectiveCamera);

window.addEventListener('click', (e) => {
    pfe_pathfinder.windowClick_eventListener(e);
})

var intervalId = window.setInterval(async function(){
    pfe_pathfinder.update_robot_position();
}, 2000);

pfe_send_path_btn.addEventListener("click", () => {
    pfe_pathfinder.send_robot_path();
})

pfe_add_calibration_point_btn.addEventListener("click", () => {
    pfe_pathfinder.add_robot_calibration_point();
})

pfe_calibrate_btn.addEventListener("click", () => {
    pfe_pathfinder.send_robot_path();
})
//----------------------

animate();

//////////////////////////////////////////////////////////
// General Helpers
//////////////////////////////////////////////////////////
function getGradientColor(value) {
    const num_colors = 2;
    var colors = new Array();
    colors[0] = new Array(1, 0, 0);
    colors[1] = new Array(33 / 255, 150 / 255, 243 / 255);


    var idx1, idx2;        // |-- Our desired color will be between these two indexes in "color".
    var fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

    if (value <= 0) idx1 = idx2 = 0;
    else if (value >= 1) idx1 = idx2 = num_colors - 1;
    else {
        value = value * (num_colors - 1);
        idx1 = Math.floor(value);
        idx2 = idx1 + 1;
        fractBetween = value - idx1;
    }

    var red = (colors[idx2][0] - colors[idx1][0]) * fractBetween + colors[idx1][0];
    var green = (colors[idx2][1] - colors[idx1][1]) * fractBetween + colors[idx1][1];
    var blue = (colors[idx2][2] - colors[idx1][2]) * fractBetween + colors[idx1][2];

    return { "r": red, "g": green, "b": blue };
}

//////////////////////////////////////////////////////////
// Websocket functions
//////////////////////////////////////////////////////////
var url = window.location.href;
var url_partial = url.split("/");

let curr_retries = [0, 0, 0, 0, 0];
let MAX_SOCKET_RETRIES = 5;

// global websocket vars
let costmap_ws, mesh_ws, plan_ws, ptcloud_ws, pose_ws;

// global rendered objects
let scene_costmap, scene_mesh, scene_pose_x, scene_pose_y, scene_pose_z,
    scene_ali_ptc0, scene_ali_ptc1, scene_ali_ptc2, scene_ali_ptc3,
    scene_ali_ptc4, scene_ali_ptc5, scene_ali_ptc6, scene_ali_ptc7, plan_pt;

let scene_qvio_group;

let aligned_pointclouds = [scene_ali_ptc0, scene_ali_ptc1, scene_ali_ptc2, scene_ali_ptc3,
    scene_ali_ptc4, scene_ali_ptc5, scene_ali_ptc6, scene_ali_ptc7];

let aligned_colors = ['#000000', '#FF0000', '#FFA500', '#FFFF00', '#008000', '#0000FF', '#4B0082', '#EE82EE'];

let scene_plans = {};

// necessary flag specifying if we need to determine location of click on canvas
var click_plan = false;

var url_base = get_url_base();
var url_port = get_url_port();


//--------PFE--------
function remove_scene_mesh(){
    scene.remove(scene_mesh);
    scene_mesh = null;
}

function add_scene_mesh(){
    scene.add(scene_mesh);
    pfe_pathfinder.add_scene_mesh(scene_mesh);
}

//-------------------

function connect_cmap() {
    costmap_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/costmap");
    costmap_ws.binaryType = "arraybuffer";

    costmap_ws.onopen = function () {
        console.log("[INFO] Costmap websocket open");
        curr_retries[0] = 0;
    };

    costmap_ws.onmessage = function (evt) {
        const in_2d_view = two_d.classList.contains("w3-blue-grey");
        const in_3d_view = esdf_btn.classList.contains("w3-green") && three_d.classList.contains("w3-blue-grey");

        if (in_2d_view || in_3d_view) {
            var received_msg = evt.data;
            var [ptcloud_meta, ptcloud_data] = parsePointCloud(received_msg);

            // ptcloud attributes
            const distances = [];
            const good_dists = [];
            const colors = [];
            const positions = [];

            const voxel_size = ptcloud_data.points[0];
            const max_dist = ptcloud_data.points[1];

            for (var i = 1; i < ptcloud_meta.n_points; i++) {

                if (in_3d_view)
                {
                    // Ignore points that are too large
                    if (ptcloud_data.confidences[i] >= max_dist)
                        continue;

                    const x = ptcloud_data.points[3 * i];
                    const y = ptcloud_data.points[3 * i + 1];
                    const z = ptcloud_data.points[3 * i + 2];

                    distances.push(ptcloud_data.confidences[i]);
                    good_dists.push(ptcloud_data.confidences[i]);
                    positions.push(x, y, z);
                }
                else
                {
                    if (ptcloud_data.confidences[i] >= max_dist)
                        ptcloud_data.confidences[i] = 10;
                    else
                        good_dists.push(ptcloud_data.confidences[i]);

                    const x = ptcloud_data.points[3 * i];
                    const y = ptcloud_data.points[3 * i + 1];
                    const z = 0;

                    distances.push(ptcloud_data.confidences[i]);
                    positions.push(x, y, z);
                }
            }

            var dist_min = Math.min.apply(Math, good_dists);
            var dist_max = Math.max.apply(Math, good_dists);

            for (var i = 0; i < distances.length - 1; i++) {
                var norm_d;
                if (distances[i] != 10) norm_d = (distances[i] - dist_min) / (dist_max - dist_min);
                else norm_d = 1;
                var new_color = getGradientColor(norm_d);
                colors.push(new_color.r, new_color.g, new_color.b);
            }
            colors.push(0, 1, 0);

            if (scene_costmap == null) {
                const pt_geometry = new THREE.BufferGeometry();
                pt_geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
                pt_geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

                const material = new THREE.PointsMaterial({ size: voxel_size * 2, vertexColors: true });

                scene_costmap = new THREE.Points(pt_geometry, material);
            }

            else {
                const old_positions = scene_costmap.geometry.attributes.position.array;
                const conc_positions = positions.concat(old_positions);

                const old_colors = scene_costmap.geometry.attributes.color.array;
                const conc_colors = colors.concat(old_colors);

                scene_costmap.geometry.setAttribute('position', new THREE.Float32BufferAttribute(conc_positions, 3));

                scene_costmap.geometry.setAttribute('color', new THREE.Float32BufferAttribute(conc_colors, 3));
            }

            // required after the first render
            scene_costmap.geometry.attributes.position.needsUpdate = true;
            scene_costmap.geometry.attributes.color.needsUpdate = true;

            // scene.add(scene_costmap);
        }
        else {
            scene.remove(scene_costmap);
            scene_costmap = null;
        }
    }

    costmap_ws.onclose = function (e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        curr_retries[0]++;
        if (curr_retries[0] <= MAX_SOCKET_RETRIES)
            setTimeout(function () {
                connect_cmap();
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    costmap_ws.onerror = function (err) {
        console.error('Socket encountered error: ', err.message, 'Closing socket');
        costmap_ws.close();
    };
}

function connect_mesh() {
    mesh_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/mesh");
    mesh_ws.binaryType = "arraybuffer";

    mesh_ws.onopen = function () {
        console.log("[INFO] Mesh websocket open");
        curr_retries[1] = 0;
    };

    mesh_ws.onmessage = function (evt) {
        if (three_d.classList.contains("w3-blue-grey")) {
            var received_msg = evt.data;
            var parser = new jParser(received_msg, {
                mesh_metadata_t: {
                    magic_number: 'uint32',
                    timestamp_ns: ['array', 'uint32', 2],
                    size_bytes: ['array', 'uint32', 2],
                    num_vertices: 'uint32',
                    num_indices: 'uint32'
                },
                mesh_vertex_t: {
                    x: 'float32',
                    y: 'float32',
                    z: 'float32',
                    r: 'uint8',
                    g: 'uint8',
                    b: 'uint8'
                },
                mesh_index_t: {
                    indices: ['array', 'uint32', 3]
                }
            });

            var mesh_metadata = parser.parse('mesh_metadata_t');

            // mesh attributes
            const m_vertices = [];
            const m_colors = [];
            const m_indices = [];
            const m_normals = [];

            for (var k = 0; k < mesh_metadata.num_vertices; k++) {
                var mesh_vertex = parser.parse('mesh_vertex_t');

                m_vertices.push(mesh_vertex.x, mesh_vertex.y, mesh_vertex.z);
                m_colors.push(mesh_vertex.r / 255.0, mesh_vertex.g / 255.0, mesh_vertex.b / 255.0);

                m_normals.push(0,0,1);
            }

            for (var k = 0; k < mesh_metadata.num_indices; k++)
            {
                var mesh_index = parser.parse('mesh_index_t');
                m_indices.push(mesh_index.indices[0]);
                m_indices.push(mesh_index.indices[1]);
                m_indices.push(mesh_index.indices[2]);
            }

            const NumComponents = 3;

            if (scene_mesh == null) {
                const m_geometry = new THREE.BufferGeometry();
                m_geometry.setIndex(m_indices);
                m_geometry.setAttribute('position', new THREE.Float32BufferAttribute(m_vertices, NumComponents));
                m_geometry.setAttribute('normal', new THREE.Float32BufferAttribute(m_normals, NumComponents));
                m_geometry.setAttribute('color', new THREE.Float32BufferAttribute(m_colors, NumComponents));

                const front_material = new THREE.MeshPhongMaterial({
                    side: THREE.FrontSide,
                    vertexColors: true
                })

                const back_material = new THREE.MeshPhongMaterial({
                    side: THREE.BackSide,
                    vertexColors: true,
                    transparent: true,
                    opacity: 0.2,
                    depthWrite: false
                });

                const front_mesh = new THREE.Mesh(m_geometry, front_material);
                const back_mesh = new THREE.Mesh(m_geometry, back_material);

                front_mesh.name = "front_mesh";
                back_mesh.name = "back_mesh";

                scene_mesh = new THREE.Group()
                scene_mesh.add(front_mesh);
                scene_mesh.add(back_mesh);
                scene_mesh.rotateX(Math.PI / 2);

                add_scene_mesh(scene_mesh);
            }
            else {
                const front_mesh = scene_mesh.getObjectByName("front_mesh");
                const back_mesh = scene_mesh.getObjectByName("back_mesh");

                front_mesh.geometry.setAttribute('position', new THREE.Float32BufferAttribute(m_vertices, NumComponents));
                front_mesh.geometry.setAttribute('normal', new THREE.Float32BufferAttribute(m_normals, NumComponents));
                front_mesh.geometry.setAttribute('color', new THREE.Float32BufferAttribute(m_colors, NumComponents));
                front_mesh.geometry.setIndex(m_indices);

                back_mesh.geometry.setAttribute('position', new THREE.Float32BufferAttribute(m_vertices, NumComponents));
                back_mesh.geometry.setAttribute('normal', new THREE.Float32BufferAttribute(m_normals, NumComponents));
                back_mesh.geometry.setAttribute('color', new THREE.Float32BufferAttribute(m_colors, NumComponents));
                back_mesh.geometry.setIndex(m_indices);
            }
        }
        else {
            remove_scene_mesh();
        }
    }

    mesh_ws.onclose = function (e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        curr_retries[1]++;
        if (curr_retries[1] <= MAX_SOCKET_RETRIES)
            setTimeout(function () {
                connect_mesh();
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    mesh_ws.onerror = function (err) {
        console.error('Socket encountered error: ', err.message, 'Closing socket');
        mesh_ws.close();
    };
}

function connect_pose() {
    pose_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/pose");
    pose_ws.binaryType = "arraybuffer";

    pose_ws.onopen = function () {
        console.log("[INFO] Pose websocket open");
        curr_retries[2] = 0;
    };

    pose_ws.onmessage = function (evt) {
        if(scene_qvio_group != null){
            scene.remove(scene_qvio_group);
        }

        if (pose_btn.classList.contains("w3-green") && three_d.classList.contains("w3-blue-grey")) {
            var received_msg = evt.data;
            var parser = new jParser(received_msg, {
                pose_6dof_t: {
                    magic_number: 'uint32',
                    timestamp_ns: 'uint64',
                    T_child_wrt_parent: ['array', 'float32', 3],
                    R_child_to_parent: ['array', 'float32', 9],
                    v_child_wrt_parent: ['array', 'float32', 3],
                    w_child_wrt_child: ['array', 'float32', 3]
                }
            });

            var pose = parser.parse('pose_6dof_t');

            const blue_material = new THREE.LineBasicMaterial({ color: 0x0000ff, linewidth: 5 });
            const green_material = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 5 });
            const red_material = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 5 });

            const p_x = [];
            const p_y = [];
            const p_z = [];

            p_x.push(new THREE.Vector3(0, 0, 0));
            p_y.push(new THREE.Vector3(0, 0, 0));
            p_z.push(new THREE.Vector3(0, 0, 0));
            p_x.push(new THREE.Vector3(0 + 0.125, 0, 0));
            p_y.push(new THREE.Vector3(0, 0 + 0.125, 0));
            p_z.push(new THREE.Vector3(0, 0, 0 + 0.125));

            const geometry_x = new THREE.BufferGeometry().setFromPoints(p_x);
            const geometry_y = new THREE.BufferGeometry().setFromPoints(p_y);
            const geometry_z = new THREE.BufferGeometry().setFromPoints(p_z);

            var m = new THREE.Matrix4();
            m.set(pose.R_child_to_parent[0], pose.R_child_to_parent[1], pose.R_child_to_parent[2], pose.T_child_wrt_parent[0],
                pose.R_child_to_parent[3], pose.R_child_to_parent[4], pose.R_child_to_parent[5], pose.T_child_wrt_parent[1],
                pose.R_child_to_parent[6], pose.R_child_to_parent[7], pose.R_child_to_parent[8], pose.T_child_wrt_parent[2],
                0, 0, 0, 1);

            geometry_x.applyMatrix4(m);
            geometry_y.applyMatrix4(m);
            geometry_z.applyMatrix4(m);

            scene_pose_x = new THREE.Line(geometry_x, red_material);
            scene_pose_y = new THREE.Line(geometry_y, green_material);
            scene_pose_z = new THREE.Line(geometry_z, blue_material);

            scene_qvio_group = new THREE.Group()
            scene_qvio_group.add(scene_pose_x);
            scene_qvio_group.add(scene_pose_y);
            scene_qvio_group.add(scene_pose_z);
            
            scene_qvio_group.rotateX(Math.PI / 2);
            scene.add(scene_qvio_group);

            if (fpv_mode)
            {
                const x_axis = new THREE.Vector3(1, 0, 0)
                const z_axis = new THREE.Vector3(0, 0, -1)
                x_axis.applyMatrix4(m);
                z_axis.applyMatrix4(m);
                perspectiveCamera.position.set(pose.T_child_wrt_parent[0], pose.T_child_wrt_parent[1], pose.T_child_wrt_parent[2]);
                perspectiveCamera.lookAt(x_axis);
            }
        }
    }

    pose_ws.onclose = function (e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        curr_retries[2]++;
        if (curr_retries[2] <= MAX_SOCKET_RETRIES)
            setTimeout(function () {
                connect_pose();
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    pose_ws.onerror = function (err) {
        console.error('Socket encountered error: ', err.message, 'Closing socket');
        pose_ws.close();
    };
}

function connect_plan() {
    plan_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/plan");
    plan_ws.binaryType = "arraybuffer";

    plan_ws.onopen = function () {
        console.log("[INFO] Plan websocket open");
        curr_retries[3] = 0;
        resetRightButtons();
    };

    const pathVisFormat = {
        LINE: 0,
        TRAJECTORY: 1,
        TREE: 2,
        POINTS: 3
    }

    plan_ws.onmessage = function (evt) {
        var received_msg = evt.data;

        var parser = new jParser(received_msg, {
            path_vis_meta_t: {
                magic_number: 'uint32',
                n_points: 'uint32',
                format: 'uint32',
                name: ['string', 32]
            },
            path_vis_t: {
                x: 'float32',
                y: 'float32',
                z: 'float32',
                r: 'uint8',
                g: 'uint8',
                b: 'uint8'
            }
        });

        var path_meta = parser.parse('path_vis_meta_t');

        // Cleanup previous scene plans
        if (path_meta.name in scene_plans) {
            scene.remove(scene_plans[path_meta.name]);
            scene_plans[path_meta.name].geometry.dispose();
            scene_plans[path_meta.name].material.dispose();
        }

        // Only show controls for local planner if we received the global path
        if(path_meta.name.includes("Global"))
        {
            showPathOptions();
        }

        // Get the path data
        const points = [];
        const colors = [];

        for (var i = 0; i < path_meta.n_points; i++) {
            var point = parser.parse('path_vis_t');
            points.push(new THREE.Vector3(point.x, point.y, point.z));
            colors.push(point.r, point.g, point.b);
        }

        // Draw according to the format
        if (path_meta.format == pathVisFormat.LINE) {
            const pt_geometry = new THREE.BufferGeometry().setFromPoints(points);
            pt_geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
            const line_material = new THREE.LineBasicMaterial({ vertexColors: THREE.VertexColors, linewidth: 4, opacity: 0.5, transparent: true });
            scene_plans[path_meta.name] = new THREE.Line(pt_geometry, line_material);
            scene.add(scene_plans[path_meta.name]);
        }
        else if (path_meta.format == pathVisFormat.TRAJECTORY) {
            const pt_curve = new THREE.CatmullRomCurve3(points);
            var curvy_points = pt_curve.getPoints(path_meta.n_points);
            var pt_geometry = new THREE.BufferGeometry().setFromPoints(curvy_points);
            pt_geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
            var line_material = new THREE.LineBasicMaterial({ vertexColors: THREE.VertexColors, linewidth: 4, opacity: 0.5, transparent: true });
            scene_plans[path_meta.name] = new THREE.Line(pt_geometry, line_material);
            scene.add(scene_plans[path_meta.name]);
        }
        else if (path_meta.format == pathVisFormat.TREE) {
            const pt_geometry = new THREE.BufferGeometry().setFromPoints(points);
            pt_geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
            const line_material = new THREE.LineBasicMaterial({ vertexColors: THREE.VertexColors, linewidth: 4, opacity: 0.5, transparent: true });
            scene_plans[path_meta.name] = new THREE.LineSegments(pt_geometry, line_material);
            scene.add(scene_plans[path_meta.name]);
        }
        else if (path_meta.format == pathVisFormat.POINTS) {
            const pt_geometry = new THREE.BufferGeometry().setFromPoints(points);
            pt_geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

            const material = new THREE.PointsMaterial({ size: 0.1, color: 0xff0000, vertexColors: true });
            scene_plans[path_meta.name] = new THREE.Points(pt_geometry, material);
            scene.add(scene_plans[path_meta.name]);
        }
    }

    plan_ws.onclose = function (e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        curr_retries[3]++;
        if (curr_retries[3] <= MAX_SOCKET_RETRIES)
            setTimeout(function () {
                connect_plan();
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    plan_ws.onerror = function (err) {
        console.error('Socket encountered error: ', err.message, 'Closing socket');
        plan_ws.close();
    };
}

function connect_ptc() {
    var ptcloud_ws = new WebSocket("ws://" + url_base + ":" + url_port + "/aligned_ptcloud");
    ptcloud_ws.binaryType = "arraybuffer";

    ptcloud_ws.onopen = function () {
        console.log("[INFO] Pointcloud websocket open");
        curr_retries[4] = 0;
    };

    ptcloud_ws.onmessage = function (evt) {
        if (ptcloud_btn.classList.contains("w3-green") && three_d.classList.contains("w3-blue-grey")) {
            var received_msg = evt.data;

            const [ptcloud_meta, ptcloud_data] = parsePointCloud(received_msg);

            let aligned_index = ptcloud_meta.id;

            if(aligned_pointclouds[aligned_index] == null) {
                var pt_geometry = new THREE.BufferGeometry();
                pt_geometry.setAttribute('position', new THREE.Float32BufferAttribute(ptcloud_data.points, 3));

                var material = new THREE.PointsMaterial({ size: 0.1, color: new THREE.Color(aligned_colors[aligned_index]), vertexColors: false });

                aligned_pointclouds[aligned_index] = new THREE.Points(pt_geometry, material);
            }
            else {
                aligned_pointclouds[aligned_index].geometry.setAttribute('position', new THREE.Float32BufferAttribute(ptcloud_data.points, 3));
            }

            // required after the first render
            aligned_pointclouds[aligned_index].geometry.buffersNeedUpdate = true;
            aligned_pointclouds[aligned_index].geometry.attributes.position.needsUpdate = true;

            // scene.add(aligned_pointclouds[aligned_index]);
        }
        else {
            for (var i = 0; i < aligned_pointclouds.length; i++){
                scene.remove(aligned_pointclouds[i]);
                aligned_pointclouds[i] = null;
            }
        }
    }

    ptcloud_ws.onclose = function (e) {
        console.log('[INFO] Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        curr_retries[4]++;
        if (curr_retries[4] <= MAX_SOCKET_RETRIES)
            setTimeout(function () {
                connect_ptc();
            }, 1000);
        else console.log("[INFO] Giving up");
    };

    ptcloud_ws.onerror = function (err) {
        console.error('Socket encountered error: ', err.message, 'Closing socket');
        ptcloud_ws.close();
    };
}

connect_cmap();
connect_mesh();
connect_plan();
connect_pose();
connect_ptc();

//////////////////////////////////////////////////////////
// Handle all of our buttons
//////////////////////////////////////////////////////////
const home = document.getElementById('home');

const pose_btn = document.getElementById('pose_btn');
const ptcloud_btn = document.getElementById('ptcloud_btn');
const esdf_btn = document.getElementById('esdf_btn');

const plan_act_btn = document.getElementById('plan_action');
const plan_point_btn = document.getElementById('plan_point_action');
const plan_point_go = document.getElementById('plan_point_go');
const plan_point_back = document.getElementById('plan_point_back');

const load_map_btn = document.getElementById('load_action');
const load_form = document.getElementById('load_form');
const load_file_path = document.getElementById('load_file_name');
const load_form_sub_btn = document.getElementById('load_sub_action');
const load_back_btn = document.getElementById("load_back");

const save_map_btn = document.getElementById('save_action')
const save_form = document.getElementById('save_form');
const save_file_path = document.getElementById('save_file_name');
const save_form_sub_btn = document.getElementById('save_sub_action');
const save_back_btn = document.getElementById('save_back');

const save_form_ply = document.getElementById("mesh_ply_radio");
const save_form_obj = document.getElementById("mesh_obj_radio");
const save_form_gltf = document.getElementById("mesh_gltf_radio");

const download_map_btn = document.getElementById('download_action')
const download_form = document.getElementById('download_form');
const download_form_sub_btn = document.getElementById('download_sub_action');
const download_back_btn = document.getElementById('download_back');

const download_form_ply = document.getElementById("dl_mesh_ply_radio");
const download_form_obj = document.getElementById("dl_mesh_obj_radio");
const download_form_gltf = document.getElementById("dl_mesh_gltf_radio");

const clear_map_btn = document.getElementById('clear_action');
const reset_vio_btn = document.getElementById('reset_action');
const clear_paths_btn = document.getElementById('clear_paths');





const cmap_slider = document.getElementById('2d_slider');
cmap_slider.setAttribute("style", "width: " + Math.ceil(window.innerHeight / 2) + "px");
const slider_label = document.getElementById('slider_label');

const three_d = document.getElementById('3d');

const path_go = document.getElementById('path_go');
const warning = document.getElementById('warning');
const path_abort = document.getElementById('path_abort');
// const path_pause = document.getElementById('path_pause');
// const path_store = document.getElementById('path_store');
// const path_resume = document.getElementById('path_resume');

const stream_add_btn = document.getElementById('stream_add');
const stream0 = document.getElementById('stream0');
const stream_rem_btn = document.getElementById('stream_remove');

const warn_close = document.getElementById('warning_close');

const orbit_controls_opt = document.getElementById("orbit_controls");
const fly_controls_opt = document.getElementById("fly_controls");
const fpv_mode_opt = document.getElementById("fpv_mode");

orbit_controls_opt.addEventListener("click", () => {
    createControls(ControlsType.Orbit);
    fpv_mode = false;
});

fly_controls_opt.addEventListener("click", () => {
    createControls(ControlsType.Fly);
    fpv_mode = false;
});

fpv_mode_opt.addEventListener("click", () => {
    fpv_mode = ! fpv_mode
});

function showPathOptions() {
    path_go.classList.replace("w3-hide", "w3-show");
    warning.classList.replace("w3-hide", "w3-show");
    // path_store.classList.replace("w3-hide", "w3-show");
}

function hidePathOptions() {
    path_go.classList.replace("w3-show", "w3-hide");
    warning.classList.replace("w3-show", "w3-hide");
    // path_store.classList.replace("w3-show", "w3-hide");
    // path_pause.classList.replace("w3-show", "w3-hide");
    // path_resume.classList.replace("w3-show", "w3-hide");
    path_abort.classList.replace("w3-show", "w3-hide");

    // Remove all plans from scene
    for (let plan in scene_plans) {
        scene.remove(scene_plans[plan]);
    }

    scene.remove(plan_pt);

    scene.remove(transform_helper);
    transform_helper.detach(plan_pt);
}

function resetRightButtons() {
    load_map_btn.classList.replace("w3-hide", "w3-show");
    save_map_btn.classList.replace("w3-hide", "w3-show");
    download_map_btn.classList.replace("w3-hide", "w3-show");
    clear_map_btn.classList.replace("w3-hide", "w3-show");
    reset_vio_btn.classList.replace("w3-hide", "w3-show");
    pfe_get_path_btn.classList.replace("w3-hide", "w3-show");
    pfe_send_path_btn.classList.replace("w3-hide", "w3-show");
    pfe_add_calibration_point_btn.classList.replace("w3-hide", "w3-show");
    pfe_calibrate_btn.classList.replace("w3-hide", "w3-show");

    save_form.style.display = 'none';
    load_form.style.display = 'none';
    download_form.style.display = 'none';

    plan_point_back.classList.replace("w3-show", "w3-hide");
    plan_point_go.classList.replace("w3-show", "w3-hide");

    if (three_d.classList.contains("w3-blue-grey")) {
        plan_point_btn.classList.replace("w3-hide", "w3-show");
        plan_act_btn.classList.replace("w3-hide", "w3-show");
        ptcloud_btn.classList.replace("w3-hide", "w3-show");
        esdf_btn.classList.replace("w3-hide", "w3-show");
        pose_btn.classList.replace("w3-hide", "w3-show");
        clear_paths_btn.classList.replace("w3-hide", "w3-show");
        cmap_slider.classList.replace("w3-show", "w3-hide");
        slider_label.classList.replace("w3-show", "w3-hide");

    }
    else {
        plan_point_btn.classList.replace("w3-show", "w3-hide");
        plan_act_btn.classList.replace("w3-show", "w3-hide");
        ptcloud_btn.classList.replace("w3-show", "w3-hide");
        esdf_btn.classList.replace("w3-show", "w3-hide");
        pose_btn.classList.replace("w3-show", "w3-hide");
        clear_paths_btn.classList.replace("w3-show", "w3-hide");
        cmap_slider.classList.replace("w3-hide", "w3-show");
        slider_label.classList.replace("w3-hide", "w3-show");
    }
}

function hideRightButtons() {
    clear_paths_btn.classList.replace("w3-show", "w3-hide");

    plan_point_btn.classList.replace("w3-show", "w3-hide");
    load_map_btn.classList.replace("w3-show", "w3-hide");
    save_map_btn.classList.replace("w3-show", "w3-hide");
    plan_act_btn.classList.replace("w3-show", "w3-hide");
    clear_map_btn.classList.replace("w3-show", "w3-hide");
    reset_vio_btn.classList.replace("w3-show", "w3-hide");
    download_map_btn.classList.replace("w3-show", "w3-hide");
    pfe_get_path_btn.classList.replace("w3-show", "w3-hide");
    pfe_send_path_btn.classList.replace("w3-show", "w3-hide");
    pfe_add_calibration_point_btn.classList.replace("w3-show", "w3-hide");
    pfe_calibrate_btn.classList.replace("w3-show", "w3-hide");

    save_form.style.display = 'none';
    load_form.style.display = 'none';
    download_form.style.display = 'none';

    plan_point_back.classList.replace("w3-show", "w3-hide");
    plan_point_go.classList.replace("w3-show", "w3-hide");

    ptcloud_btn.classList.replace("w3-show", "w3-hide");
    esdf_btn.classList.replace("w3-show", "w3-hide");
    pose_btn.classList.replace("w3-show", "w3-hide");
}

// always checking if home is clicked, if so, close ws
home.addEventListener("click", () => {
    mesh_ws.close();
    pose_ws.close();
    plan_ws.close();
    ptcloud_ws.close();
    costmap_ws.close();
});

warn_close.addEventListener("click", () => {
    warning.classList.replace("w3-show", "w3-hide");
});

two_d.addEventListener("click", () => {
    fpv_mode = false;
    resetCamera();
    two_d.classList.replace("w3-dark-grey", "w3-blue-grey");
    three_d.classList.replace("w3-blue-grey", "w3-dark-grey");
    cmap_slider.setAttribute("style", "width: " + Math.ceil(window.innerHeight / 2) + "px");

    if (prev_control == ControlsType.Orbit) {
        controls.reset();
        controls.enableRotate = false;
    }
    else if (prev_control == ControlsType.Fly) {
        createControls(ControlsType.Fly);
        controls.rollSpeed = 0;
    }

    hidePathOptions();
    if (plan_ws.readyState == WebSocket.OPEN) resetRightButtons();
})

cmap_slider.addEventListener("change", () => {
    if (mesh_ws.readyState == WebSocket.OPEN) {
        mesh_ws.send("slice_level:" + cmap_slider.value);
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
    console.log(cmap_slider.value);
})

three_d.addEventListener("click", () => {
    fpv_mode = false;
    resetCamera();
    three_d.classList.replace("w3-dark-grey", "w3-blue-grey");
    two_d.classList.replace("w3-blue-grey", "w3-dark-grey");

    if (prev_control == ControlsType.Orbit) {
        controls.enableRotate = true;
    }
    else if (prev_control == ControlsType.Fly) {
        controls.rollSpeed = 0.3;
    }
    if (plan_ws.readyState == WebSocket.OPEN) resetRightButtons();
})

mode_btn.addEventListener("click", () => {
    if (mode_btn.classList.contains("w3-black")) {
        var x = document.getElementById("button_text");
        x.textContent = "Dark Mode";
        mode_btn.classList.replace("w3-black", "w3-white");
        scene.background = new THREE.Color(0x000000);
        localStorage.setItem("dark_mode","true");
        tick_markers.forEach((tick_marker) => {
            tick_marker.material.color.set(0xcccccc);
        })
        z_marker.material.color.set(0xcccccc);
        label_material.material.color.set(0x000000);
    }
    else {
        var x = document.getElementById("button_text");
        x.textContent = "Light Mode";
        mode_btn.classList.replace("w3-white", "w3-black");
        scene.background = new THREE.Color(0xcccccc);
        localStorage.setItem("dark_mode","false");
        tick_markers.forEach((tick_marker) => {
            tick_marker.material.color.set(0x000000);
        })
        z_marker.material.color.set(0x000000);
        label_material.material.color.set(0x000000);
    }
})

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

path_go.addEventListener("click", () => {
    if (plan_ws.readyState == WebSocket.OPEN) {
        plan_ws.send("follow_path");
        path_go.classList.replace("w3-show", "w3-hide");
        // path_store.classList.replace("w3-show", "w3-hide");
        warning.classList.replace("w3-show", "w3-hide");
        path_abort.classList.replace("w3-hide", "w3-show");
        // path_pause.classList.replace("w3-hide", "w3-show");
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
})

path_abort.addEventListener("click", () => {
    if (plan_ws.readyState == WebSocket.OPEN) {
        plan_ws.send("stop_following");
        hidePathOptions();
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
})

// path_store.addEventListener("click", () => {
//     if (plan_ws.readyState == WebSocket.OPEN){
//         plan_ws.send("store_path");
//         path_store.classList.replace("w3-show", "w3-hide");
//     }
//     else {
//         console.log("uh oh. Plan websocket is closed...");
//     }
// })


// path_resume.addEventListener("click", () => {
//     if (plan_ws.readyState == WebSocket.OPEN){
//         plan_ws.send("resume_path");
//         path_resume.classList.replace("w3-show", "w3-hide");
//         path_pause.classList.replace("w3-hide", "w3-show");
//     }
//     else {
//         console.log("uh oh. Plan websocket is closed...");
//     }
// })

// path_pause.addEventListener("click", () => {
//     if (plan_ws.readyState == WebSocket.OPEN){
//         plan_ws.send("pause_path");
//         path_pause.classList.replace("w3-show", "w3-hide");
//         path_resume.classList.replace("w3-hide", "w3-show");
//     }
//     else {
//         console.log("uh oh. Plan websocket is closed...");
//     }
// })

clear_paths_btn.addEventListener("click", () => {
    hidePathOptions();
});

pose_btn.addEventListener("click", () => {
    if (pose_btn.classList.contains("w3-red")) {
        pose_btn.classList.replace("w3-red", "w3-green");
    }
    else {
        pose_btn.classList.replace("w3-green", "w3-red");
    }
});

ptcloud_btn.addEventListener("click", () => {
    if (ptcloud_btn.classList.contains("w3-red")) {
        ptcloud_btn.classList.replace("w3-red", "w3-green");
    }
    else {
        ptcloud_btn.classList.replace("w3-green", "w3-red");
    }
});

esdf_btn.addEventListener("click", () => {
    if (esdf_btn.classList.contains("w3-red")) {
        esdf_btn.classList.replace("w3-red", "w3-green");
        cmap_slider.classList.replace("w3-hide", "w3-show");
        slider_label.classList.replace("w3-hide", "w3-show");
    }
    else {
        esdf_btn.classList.replace("w3-green", "w3-red");
        cmap_slider.classList.replace("w3-show", "w3-hide");
        slider_label.classList.replace("w3-show", "w3-hide");
    }
});

plan_act_btn.addEventListener("click", () => {
    if (plan_ws.readyState == WebSocket.OPEN) {
        plan_ws.send("plan_home");

        // Remove all plans from scene
        for (let plan in scene_plans) {
            scene.remove(scene_plans[plan]);
        }
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
})

plan_point_btn.addEventListener("click", () => {
    scene.remove(plan_pt);

    scene.remove(transform_helper);
    transform_helper.detach(plan_pt);

    click_plan = true;
    hideRightButtons();
    plan_point_back.classList.replace("w3-hide", "w3-show");
    plan_point_go.classList.replace("w3-hide", "w3-show");
})

plan_point_go.addEventListener("click", () => {
    scene.remove(transform_helper);
    transform_helper.detach(plan_pt);

    const pos = plan_pt.geometry.attributes.position.array;
    const msg = "plan_to: " + (pos[0] + plan_pt.position.x) + "," + (pos[1] + plan_pt.position.y) + "," + (pos[2] + plan_pt.position.z);
    console.log(msg);

    if (plan_ws.readyState == WebSocket.OPEN) {
        plan_ws.send(msg);

        // Remove all plans from scene
        for (let plan in scene_plans) {
            scene.remove(scene_plans[plan]);
        }
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
        scene.remove(plan_pt);

        scene.remove(transform_helper);
        transform_helper.detach(plan_pt);
    }
    resetRightButtons();
})

plan_point_back.addEventListener("click", () => {
    scene.remove(plan_pt);

    scene.remove(transform_helper);
    transform_helper.detach(plan_pt);
    
    resetRightButtons();
})

canvas.onclick = function getClicked3DPoint(evt) {
    perspectiveCamera.updateMatrix();
    if (click_plan) {
        var vec = new THREE.Vector3(); // create once and reuse
        var pos = new THREE.Vector3(); // create once and reuse

        var rect = renderer.domElement.getBoundingClientRect();

        vec.set(
            ((evt.clientX - rect.left) / (rect.width - rect.left)) * 2 - 1,
            - ((evt.clientY - rect.top) / (rect.bottom - rect.top)) * 2 + 1,
            perspectiveCamera.position.z);

        vec.unproject(perspectiveCamera);

        vec.sub(perspectiveCamera.position).normalize();

        var distance = - perspectiveCamera.position.z / vec.z;

        pos.copy(perspectiveCamera.position).add(vec.multiplyScalar(distance));
        pos.z = 0.0;

        const plan_geom = new THREE.BufferGeometry();
        const col = [0, 0, 0];
        plan_geom.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
        plan_geom.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));

        plan_geom.computeBoundingSphere();

        const material = new THREE.PointsMaterial({ size: 0.2, vertexColors: true });

        plan_pt = new THREE.Points(plan_geom, material);
        scene.add(plan_pt);

        transform_helper.position.copy(plan_pt.position);
        transform_helper.attach(plan_pt);
        scene.add(transform_helper);
        // transform_helper.getObjectByName("gizmo").position.copy(pos);

        click_plan = false;
    }

};

load_map_btn.addEventListener("click", () => {
    hideRightButtons();
    load_form.style.display = 'block';
})

load_back_btn.addEventListener("click", () => {
    resetRightButtons();
})

save_map_btn.addEventListener("click", () => {
    hideRightButtons();
    save_form.style.display = 'block';
})

save_back_btn.addEventListener("click", () => {
    resetRightButtons();
})

download_map_btn.addEventListener("click", () => {
    hideRightButtons();
    download_form.style.display = 'block';
})

download_back_btn.addEventListener("click", () => {
    resetRightButtons();
})


load_form_sub_btn.addEventListener("click", () => {
    if (mesh_ws.readyState == WebSocket.OPEN) {
        var file_path = load_file_path.value;
        file_path = file_path.replace(/\s+/g, '');
        if (file_path.length == 0) {
            mesh_ws.send("load_map");
        }
        else {
            const msg = "load_map file: ";
            const conc_msg = msg.concat(file_path);
            mesh_ws.send(conc_msg);
        }
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
    resetRightButtons();
})

save_form_sub_btn.addEventListener("click", () => {
    if (mesh_ws.readyState == WebSocket.OPEN) {
        var file_path = save_file_path.value;
        file_path = file_path.replace(/\s+/g, '');
        
        var msg = "save_map";
        if (save_form_ply.checked){
            msg = msg.concat(":ply");
        }
        else if (save_form_obj.checked){
            msg = msg.concat(":obj");
        }
        else if (save_form_gltf.checked){
            msg = msg.concat(":gltf");
        }

        if (file_path.length == 0) {
            mesh_ws.send(msg);
        }
        else {
            msg = msg.concat(" file: ");
            const conc_msg = msg.concat(file_path);
            mesh_ws.send(conc_msg);
        }
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
    resetRightButtons();
})

download_form_sub_btn.addEventListener("click", () => {
    if (download_form_ply.checked){
        fetch("/mesh_api/ply").then(function (response) {
            return response.text().then(function (text) {
                var down_t = document.createElement('a');
                down_t.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
                down_t.setAttribute('download', "mesh.ply");
                down_t.click();
            });
        });
    }
    else if (download_form_obj.checked){
        fetch("/mesh_api/obj").then(function (response) {
            return response.text().then(function (text) {
                var down_t = document.createElement('a');
                down_t.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
                down_t.setAttribute('download', "mesh.obj");
                down_t.click();
            });
        });
    }
    else if (download_form_gltf.checked){
        fetch("/mesh_api/gltf").then(function (response) {
            return response.text().then(function (text) {
                var down_t = document.createElement('a');
                down_t.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
                down_t.setAttribute('download', "mesh.gltf");
                down_t.click();
            });
        });
    }
    resetRightButtons();
});

clear_map_btn.addEventListener("click", () => {
    if (mesh_ws.readyState == WebSocket.OPEN) {
        mesh_ws.send("clear_map");
        remove_scene_mesh();
        scene.remove(scene_costmap);
        scene_costmap = null;

        hidePathOptions();
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
})

reset_vio_btn.addEventListener("click", () => {
    if (mesh_ws.readyState == WebSocket.OPEN) {
        mesh_ws.send("reset_vio");
    }
    else {
        console.log("uh oh. Plan websocket is closed...");
    }
})


function createControls(controls_type) {
    if (prev_control == controls_type) return;
    else prev_control = controls_type;

    perspectiveCamera = new THREE.PerspectiveCamera(fov, aspect, near, far);
    perspectiveCamera.position.x = 0;
    perspectiveCamera.position.y = 0;
    perspectiveCamera.position.z = -25;
    perspectiveCamera.rotation.set(0, 0, 0);

    if (controls_type == ControlsType.Orbit) {
        console.log("ORBITING");
        controls = new OrbitControls(perspectiveCamera, canvas);
        controls.maxDistance = far;

        controls.zoomSpeed = 1.2;
        controls.panSpeed = 0.5;
        controls.rotateSpeed = 0.25;
        if (two_d.classList.contains("w3-blue-grey")) controls.noRotate = true;
    }
    else if (controls_type == ControlsType.Fly) {
        console.log("FLYING");
        controls = new FlyControls(perspectiveCamera, canvas);

        controls.movementSpeed = 12;
        controls.domElement = canvas;
        controls.rollSpeed = 0.3;
        controls.autoForward = false;
        controls.dragToLook = false;
        if (two_d.classList.contains("w3-blue-grey")) controls.rollSpeed = 0;
    }
    // perspectiveCamera.up.set(0, 0, -1);
    perspectiveCamera.lookAt(0, 0, 0);

    controls.addEventListener('change',function() {
        console.log(perspectiveCamera.position.x, perspectiveCamera.position.y, perspectiveCamera.position.z);
    })
}

function resetCamera()
{
    perspectiveCamera.position.x = 0;
    perspectiveCamera.position.y = 0;
    perspectiveCamera.position.z = -25;
    perspectiveCamera.rotation.set(0, 0, 0);
    // perspectiveCamera.up.set(0, 0, -1);
    perspectiveCamera.lookAt(0, 0, 0);
}

function onWindowResize() {
    pfe_pathfinder.windowResize(window.innerHeight - height_offset, window.innerWidth)

    const aspect = window.innerWidth / window.innerHeight;   

    perspectiveCamera.aspect = aspect;
    perspectiveCamera.updateProjectionMatrix();

    renderer.setSize(window.innerWidth, window.innerHeight - height_offset);

    if (controls.handleResize) controls.handleResize();

}

function animate() {

    requestAnimationFrame(animate);
    
    renderer.render(scene, perspectiveCamera);

    // Dont update controls if in fpv mode
    if (!fpv_mode)
        controls.update(clock.getDelta());
}

function render() {
    renderer.render( scene, perspectiveCamera );
}

function closeCameraAcc() {

    //Close and remove the green coloring of the dropdown
    var x = document.getElementById("streamAcc");
    x.className = x.className.replace(" w3-show", "");

    //Remove all the children, they should get repopulated to ensure no faulty cameras
    /*
    var eles = document.getElementsByClassName("cam-item");
    while (eles[0]) {
        eles[0].parentNode.removeChild(eles[0]);
    }
    */

}

stream_rem_btn.addEventListener("click", () => {
    stream0.src = "";
    stream0.className = "w3-hide";
    var div = document.getElementById('stream_div');
    div.style = "";
    stream_rem_btn.className = "w3-hide";
    var cam_name_h3 = document.getElementById("cam_name");
    cam_name_h3.style.display = "none";
    /*
    var x = document.getElementById("streamAcc");
    x.classList.replace("w3-show", "w3-hide");
    */
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

function updateCameraList() {
    fetch('/_cmd/list_cameras').then(function (response) {
        return response.text().then(function (text) {
            var y = document.getElementById("dropdown_vertical_streamAcc");
            var cams = text.split(" ");

            for (var i = 0; i < cams.length; i++) {
                if (cams[i].length < 1) continue;

                var ele = document.createElement("a");
                ele.setAttribute("id", "btn_" + cams[i]);
                ele.setAttribute("pipe", cams[i]);
                ele.addEventListener("click", function (e) {
                    var imgString = "/video_raw/" + this.getAttribute("pipe");
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
                for (var j = 0; j < nameList.length; j++) {
                    if (nameList[j].length < 1) continue;
                    nameList[j] = nameList[j][0].toUpperCase() + nameList[j].substring(1);
                }
                ele.textContent = nameList.join(' ');
                y.appendChild(ele);
            }

            //y.className += " w3-show";
        });
    });

}

function parsePointCloud(received_msg) {
    var parser = new jParser(received_msg, {
        ptcloud_metadata_t: {
            magic_number: 'uint32',
            timestamp_ns: ['array', 'uint32', 2],
            n_points: 'uint32',
            format: 'uint32',
            id: 'uint32',
            server_name: ['array', 'char', 32],
            reserved: 'uint32'
        },
        point_xyz: {
            x: 'float32',
            y: 'float32',
            z: 'float32'
        },
        point_xyz_c: {
            x: 'float32',
            y: 'float32',
            z: 'float32',
            c: 'float32'
        },
        point_xyz_rgb: {
            x: 'float32',
            y: 'float32',
            z: 'float32',
            r: 'uint8',
            g: 'uint8',
            b: 'uint8'
        },
        point_xyz_c_rgb: {
            x: 'float32',
            y: 'float32',
            z: 'float32',
            c: 'float32',
            r: 'uint8',
            g: 'uint8',
            b: 'uint8'
        },
        point_xy: {
            x: 'float32',
            y: 'float32'
        },
        point_xyc: {
            x: 'float32',
            y: 'float32',
            c: 'float32'
        }
    });

    let ptcloud_meta = parser.parse('ptcloud_metadata_t');
    let ptcloud_data = {
        points: [],
        colors: [],
        confidences: []
    };

    switch (ptcloud_meta.format) {
        case pointcloud_format.FLOAT_XYZ:
            var data = Array.from({ length: ptcloud_meta.n_points }, (_, i) => parser.parse("point_xyz"));
            ptcloud_data.points = data.map(p => [p.x, p.y, p.z]).flat();
            break;

        case pointcloud_format.FLOAT_XYZC:
            var data = Array.from({ length: ptcloud_meta.n_points }, (_, i) => parser.parse("point_xyz_c"));
            ptcloud_data.points = data.map(p => [p.x, p.y, p.z]).flat();
            ptcloud_data.confidences = data.map(p => p.c);
            break;

        case pointcloud_format.FLOAT_XYZRGB:
            var data = Array.from({ length: ptcloud_meta.n_points }, (_, i) => parser.parse("point_xyz_rgb"));
            ptcloud_data.points = data.map(p => [p.x, p.y, p.z]).flat();
            ptcloud_data.colors = data.map(p => [p.r, p.g, p.b]).flat();
            break;

        case pointcloud_format.FLOAT_XYZCRGB:
            var data = Array.from({ length: ptcloud_meta.n_points }, (_, i) => parser.parse("point_xyz_c_rgb"));
            ptcloud_data.points = data.map(p => [p.x, p.y, p.z]).flat();
            ptcloud_data.colors = data.map(p => [p.r, p.g, p.b]).flat();
            ptcloud_data.confidences = data.map(p => p.c);
            break;

        case pointcloud_format.FLOAT_XY:
            var data = Array.from({ length: ptcloud_meta.n_points }, (_, i) => parser.parse("point_xy"));
            ptcloud_data.points = data.map(p => [p.x, p.y]).flat();
            break;

        case pointcloud_format.FLOAT_XYC:
            var data = Array.from({ length: ptcloud_meta.n_points }, (_, i) => parser.parse("point_xyc"));
            ptcloud_data.points = data.map(p => [p.x, p.y]).flat();
            ptcloud_data.confidences = data.map(p => p.c);
            break;
    }

    return [ptcloud_meta, ptcloud_data];
}