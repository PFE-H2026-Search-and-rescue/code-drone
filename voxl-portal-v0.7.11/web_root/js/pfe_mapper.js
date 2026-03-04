import * as THREE from './3rd_party/pfe_mapper/three.module.js';
import { generateSoloNavMesh } from './3rd_party/pfe_mapper/recast-navigation-generators.js';
import {
    DebugDrawer,
    getPositionsAndIndices,
} from './3rd_party/pfe_mapper/recast-navigation-three.js';
import {
    NavMeshQuery,
} from './3rd_party/pfe_mapper/recast-navigation-core.js';



class PFE_Pathfinder {
  constructor(height, width, height_offset, scene, perspectiveCamera) {
    this.height = height;
    this.width = width;
    this.height_offset = height_offset;
    this.scene = scene;
    this.scene_mesh = null;
    this.perspectiveCamera = perspectiveCamera;

    this.debugDrawer = null
    this.startMarker = null;
    this.endMarker = null;
    this.pathLine = null;
    this.robotPosition = {x: 0, y:0, z:0}

    this.robot_pose_x = null;
    this.robot_pose_y = null;
    this.robot_pose_y = null;
  }

  add_scene_mesh(scene_mesh){
    this.scene_mesh = scene_mesh;
  }

  remove_scene_mesh(){
    this.scene_mesh = null;
    reset_navmesh();
  }

  windowResize(height, width){
    this.height = height;
    this.width = width;
  }

  reset_navmesh(){
    if(this.debugDrawer != null){
        this.scene.remove(this.debugDrawer);
        this.debugDrawer == null;
    }

    if(this.startMarker != null){
        this.scene.remove(this.startMarker);
        this.startMarker == null;
    }
    if(this.endMarker != null){
        this.scene.remove(this.endMarker);
        this.endMarker == null;
    }
    if(this.pathLine != null){
        this.scene.remove(this.pathLine);
        this.pathLine == null;
    }
  }

  update_robot_position(){
    this.robotPosition.x += 0.2;
    this.robotPosition.y = 0;
    this.robotPosition.z = 0;

    //TODO : Faire en sorte qu'il le pogne du serveur

    if(this.robot_pose_x != null) this.scene.remove(this.robot_pose_x);
    if(this.robot_pose_y != null) this.scene.remove(this.robot_pose_y);
    if(this.robot_pose_z != null) this.scene.remove(this.robot_pose_z);
    

    const blue_material = new THREE.LineBasicMaterial({ color: 0x0000ff, linewidth: 5 });
    const green_material = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 5 });
    const red_material = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 5 });

    const p_x = [];
    const p_y = [];
    const p_z = [];

    p_x.push(new THREE.Vector3(this.robotPosition.x , this.robotPosition.y, this.robotPosition.z ));
    p_y.push(new THREE.Vector3(this.robotPosition.x , this.robotPosition.y, this.robotPosition.z ));
    p_z.push(new THREE.Vector3(this.robotPosition.x , this.robotPosition.y, this.robotPosition.z ));
    p_x.push(new THREE.Vector3(this.robotPosition.x  + 0.125, this.robotPosition.y, this.robotPosition.z ));
    p_y.push(new THREE.Vector3(this.robotPosition.x , this.robotPosition.y + 0.125, this.robotPosition.z ));
    p_z.push(new THREE.Vector3(this.robotPosition.x , this.robotPosition.y, this.robotPosition.z  + 0.125));

    const geometry_x = new THREE.BufferGeometry().setFromPoints(p_x);
    const geometry_y = new THREE.BufferGeometry().setFromPoints(p_y);
    const geometry_z = new THREE.BufferGeometry().setFromPoints(p_z);

    this.robot_pose_x = new THREE.Line(geometry_x, red_material);
    this.robot_pose_y = new THREE.Line(geometry_y, green_material);
    this.robot_pose_z = new THREE.Line(geometry_z, blue_material);

    this.scene.add(this.robot_pose_x);
    this.scene.add(this.robot_pose_y);
    this.scene.add(this.robot_pose_z);
  }

  windowClick_eventListener(e){
    if(this.scene_mesh == null){
        return;
    }

    let intersects = []
    const mouse = new THREE.Vector2()
    const raycaster = new THREE.Raycaster()

    mouse.set((e.clientX / this.width) * 2 - 1, -((e.clientY - this.height_offset) / this.height) * 2 + 1)
    raycaster.setFromCamera(mouse, this.perspectiveCamera)
    intersects = raycaster.intersectObjects(this.scene.children, true)

    console.log("Clicked")
    if (intersects.length > 0){
        let found = null;
        for (let index = 0; index < intersects.length; index++) {
            const element = intersects[index];
            
            if(element.object.name == "front_mesh"){
                found = {x: element.point.x, y:element.point.y, z: element.point.z};
                break;
            }
        }
        if(found == null) return;
        console.log(found);

        const navmesh = this._generate_navmesh();
       
        // compute a path

        const start = this.robotPosition;
        const end = found;
        this._get_path(navmesh, start, end);
        
    }
  }

  _generate_navmesh(){
    const [positions, indices] = getPositionsAndIndices([
        this.scene_mesh.getObjectByName("front_mesh")
    ]);

    // generate a solo navmesh
    const cs = 0.05;
    const ch = 0.05;
    const walkableRadius = 0.2;
    const { success, navMesh } = generateSoloNavMesh(positions, indices, {
        cs,
        ch,
        walkableRadius: Math.round(walkableRadius / ch),
    });

    this.reset_navmesh();

    // debug draw the navmesh
    
    this.debugDrawer = new DebugDrawer();
    this.debugDrawer.drawNavMesh(navMesh);
    this.scene.add(this.debugDrawer);
    return navMesh
  }

  _get_path(navMesh, start_point, end_point){
        const navMeshQuery = new NavMeshQuery(navMesh);
        const {point : start} = navMeshQuery.findClosestPoint(start_point);
        const {point : end} = navMeshQuery.findClosestPoint(end_point);
        console.log(end_point);
        console.log(end);
        const { path } = navMeshQuery.computePath(start, end);
        console.log(path.toString())
        // draw the path start

        this.startMarker = new THREE.Mesh(
            new THREE.BoxGeometry(0.1, 0.1, 0.1),
            new THREE.MeshBasicMaterial({ color: 'blue' })
        );
        this.startMarker.position.set(start.x, start.y + 0.1, start.z);
        this.scene.add(this.startMarker);

        // draw the path end
        this.endMarker = new THREE.Mesh(
            new THREE.BoxGeometry(0.1, 0.1, 0.1),
            new THREE.MeshBasicMaterial({ color: 'green' })
        );
        this.endMarker.position.set(end.x, end.y + 0.1, end.z);
        this.scene.add(this.endMarker);

        // draw the path line
        this.pathLine = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(
            path.map(({ x, y, z }) => new THREE.Vector3(x, y, z))
            ),
            new THREE.LineBasicMaterial({ color: 'blue' })
        );
        this.pathLine.position.y += 0.1;
        this.scene.add(this.pathLine);
    }
}
export { PFE_Pathfinder };