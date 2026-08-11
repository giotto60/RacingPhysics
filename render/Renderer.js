/**
 * Renderer.js — Three.js Scene, Camera, Lighting, and Car Visual Mesh setup.
 * Cute, minimalistic GT Sport Coupe with responsive camera.
 */
import * as THREE from 'three';

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;

    // Renderer setup
    this.threeRenderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: 'high-performance'
    });
    this.threeRenderer.setSize(window.innerWidth, window.innerHeight);
    this.threeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.threeRenderer.shadowMap.enabled = true;
    this.threeRenderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.threeRenderer.toneMapping = THREE.ACESFilmicToneMapping;

    // Scene with bright, clean background
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x161e33);
    this.scene.fog = new THREE.FogExp2(0x161e33, 0.005);

    // Camera
    this.camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 500);
    this.cameraTarget = new THREE.Vector3(0, 0.8, 0);
    this.cameraPos = new THREE.Vector3(0, 3.2, -7.0);
    this.camera.position.copy(this.cameraPos);
    this.camera.lookAt(this.cameraTarget);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.5);
    dirLight.position.set(20, 40, 20);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 150;
    dirLight.shadow.camera.left = -40;
    dirLight.shadow.camera.right = 40;
    dirLight.shadow.camera.top = 40;
    dirLight.shadow.camera.bottom = -40;
    this.scene.add(dirLight);

    // Ground Plane with Grid
    this.createGround();

    // Car Visual Mesh (Brick on Wheels)
    this.createCarMesh();

    // Window resize listener
    window.addEventListener('resize', () => this.onWindowResize());
  }

  createGround() {
    const planeGeo = new THREE.PlaneGeometry(600, 600);
    const planeMat = new THREE.MeshStandardMaterial({
      color: 0x1f273d,
      roughness: 0.7,
      metalness: 0.1
    });
    const ground = new THREE.Mesh(planeGeo, planeMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(600, 300, 0x60a5fa, 0x334155);
    grid.position.y = 0.01;
    this.scene.add(grid);
  }

  createCarMesh() {
    this.carGroup = new THREE.Group();

    // Body (Low-slung Sports Coupe)
    const bodyGeo = new THREE.BoxGeometry(1.6, 0.50, 3.2);
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x2563eb, // Vibrant Royal Blue
      roughness: 0.2,
      metalness: 0.5
    });
    this.bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
    this.bodyMesh.position.y = 0.35;
    this.bodyMesh.castShadow = true;
    this.bodyMesh.receiveShadow = true;
    this.carGroup.add(this.bodyMesh);

    // Aerodynamic Canopy Roof
    const roofGeo = new THREE.BoxGeometry(1.3, 0.40, 1.6);
    const roofMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.1,
      metalness: 0.9
    });
    const roofMesh = new THREE.Mesh(roofGeo, roofMat);
    roofMesh.position.set(0, 0.75, -0.1);
    roofMesh.castShadow = true;
    this.carGroup.add(roofMesh);

    // Headlights
    const lightGeo = new THREE.BoxGeometry(0.35, 0.12, 0.1);
    const hlMat = new THREE.MeshBasicMaterial({ color: 0x93c5fd });
    const hlLeft = new THREE.Mesh(lightGeo, hlMat);
    hlLeft.position.set(-0.55, 0.35, 1.58);
    const hlRight = new THREE.Mesh(lightGeo, hlMat);
    hlRight.position.set(0.55, 0.35, 1.58);
    this.carGroup.add(hlLeft);
    this.carGroup.add(hlRight);

    // Taillights
    const tlMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const tlLeft = new THREE.Mesh(lightGeo, tlMat);
    tlLeft.position.set(-0.55, 0.35, -1.58);
    const tlRight = new THREE.Mesh(lightGeo, tlMat);
    tlRight.position.set(0.55, 0.35, -1.58);
    this.carGroup.add(tlLeft);
    this.carGroup.add(tlRight);

    // Wheels (4 cylinders)
    this.wheelMeshes = [];
    const wheelGeo = new THREE.CylinderGeometry(0.32, 0.32, 0.25, 24);
    wheelGeo.rotateZ(Math.PI / 2);

    const tireMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.6
    });

    const rimGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.26, 12);
    rimGeo.rotateZ(Math.PI / 2);
    const rimMat = new THREE.MeshStandardMaterial({
      color: 0xf1f5f9,
      metalness: 0.8,
      roughness: 0.2
    });

    for (let i = 0; i < 4; i++) {
      const wheelGroup = new THREE.Group();
      const tire = new THREE.Mesh(wheelGeo, tireMat);
      tire.castShadow = true;
      const rim = new THREE.Mesh(rimGeo, rimMat);

      wheelGroup.add(tire);
      wheelGroup.add(rim);

      this.scene.add(wheelGroup);
      this.wheelMeshes.push(wheelGroup);
    }

    this.scene.add(this.carGroup);
  }

  update(vehicleState) {
    const { position, orientation, wheels } = vehicleState;

    this.carGroup.position.set(position.x, position.y, position.z);
    this.carGroup.quaternion.set(orientation.x, orientation.y, orientation.z, orientation.w);

    for (let i = 0; i < wheels.length; i++) {
      const w = wheels[i];
      const mesh = this.wheelMeshes[i];

      const attachLocal = new THREE.Vector3(
        w.localAttachPos.x,
        w.localAttachPos.y - w.suspensionLength,
        w.localAttachPos.z
      );
      attachLocal.applyQuaternion(this.carGroup.quaternion);
      mesh.position.copy(this.carGroup.position).add(attachLocal);

      mesh.quaternion.copy(this.carGroup.quaternion);

      const isFront = i < 2;
      if (isFront && w.steerAngle) {
        mesh.rotateY(w.steerAngle);
      }

      if (w.rotationAngle) {
        mesh.rotateX(w.rotationAngle);
      }
    }

    // Smooth Chase Camera
    const idealOffset = new THREE.Vector3(0, 3.0, -6.5);
    idealOffset.applyQuaternion(this.carGroup.quaternion);
    idealOffset.add(this.carGroup.position);

    const idealLookAt = new THREE.Vector3(0, 0.8, 2.5);
    idealLookAt.applyQuaternion(this.carGroup.quaternion);
    idealLookAt.add(this.carGroup.position);

    this.cameraPos.lerp(idealOffset, 0.12);
    this.cameraTarget.lerp(idealLookAt, 0.18);

    this.camera.position.copy(this.cameraPos);
    this.camera.lookAt(this.cameraTarget);

    this.threeRenderer.render(this.scene, this.camera);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.threeRenderer.setSize(window.innerWidth, window.innerHeight);
  }
}
