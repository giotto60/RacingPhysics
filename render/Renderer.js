/**
 * Renderer.js — Three.js Scene, Camera, Lighting, and Car Visual Mesh setup.
 * Cute, minimalistic aesthetic with soft shadow grid and smooth chase camera.
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

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0c18);
    this.scene.fog = new THREE.FogExp2(0x0a0c18, 0.008);

    // Camera
    this.camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 500);
    this.cameraTarget = new THREE.Vector3();
    this.cameraPos = new THREE.Vector3(0, 5, -10);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xd0e0ff, 0.6);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
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
    // Large ground plane
    const planeGeo = new THREE.PlaneGeometry(500, 500);
    const planeMat = new THREE.MeshStandardMaterial({
      color: 0x141828,
      roughness: 0.8,
      metalness: 0.1
    });
    const ground = new THREE.Mesh(planeGeo, planeMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    // Grid helper for movement reference
    const grid = new THREE.GridHelper(500, 250, 0x4080ff, 0x203050);
    grid.position.y = 0.01; // Slightly above ground plane
    this.scene.add(grid);
  }

  createCarMesh() {
    this.carGroup = new THREE.Group();

    // Car Body (Stylized Cute Brick / Compact Coupe)
    const bodyGeo = new THREE.BoxGeometry(1.6, 0.7, 3.2);
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x3b82f6, // Vibrant Electric Blue
      roughness: 0.2,
      metalness: 0.5
    });
    this.bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
    this.bodyMesh.position.y = 0.45;
    this.bodyMesh.castShadow = true;
    this.bodyMesh.receiveShadow = true;
    this.carGroup.add(this.bodyMesh);

    // Cabin / Roof (Cute minimalistic glass canopy)
    const roofGeo = new THREE.BoxGeometry(1.3, 0.5, 1.6);
    const roofMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.1,
      metalness: 0.9,
      transparent: true,
      opacity: 0.9
    });
    const roofMesh = new THREE.Mesh(roofGeo, roofMat);
    roofMesh.position.set(0, 0.95, -0.2);
    roofMesh.castShadow = true;
    this.carGroup.add(roofMesh);

    // Headlights
    const lightGeo = new THREE.BoxGeometry(0.3, 0.15, 0.1);
    const lightMat = new THREE.MeshBasicMaterial({ color: 0x93c5fd });
    const hlLeft = new THREE.Mesh(lightGeo, lightMat);
    hlLeft.position.set(-0.55, 0.45, 1.56);
    const hlRight = new THREE.Mesh(lightGeo, lightMat);
    hlRight.position.set(0.55, 0.45, 1.56);
    this.carGroup.add(hlLeft);
    this.carGroup.add(hlRight);

    // Taillights
    const tailMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const tlLeft = new THREE.Mesh(lightGeo, tailMat);
    tlLeft.position.set(-0.55, 0.45, -1.56);
    const tlRight = new THREE.Mesh(lightGeo, tailMat);
    tlRight.position.set(0.55, 0.45, -1.56);
    this.carGroup.add(tlLeft);
    this.carGroup.add(tlRight);

    // Wheels (4 cylinders)
    this.wheelMeshes = [];
    const wheelGeo = new THREE.CylinderGeometry(0.32, 0.32, 0.25, 24);
    wheelGeo.rotateZ(Math.PI / 2); // Align cylinder axis along X

    const wheelMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.5
    });

    const rimGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.26, 12);
    rimGeo.rotateZ(Math.PI / 2);
    const rimMat = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      metalness: 0.8,
      roughness: 0.2
    });

    for (let i = 0; i < 4; i++) {
      const wheelGroup = new THREE.Group();
      const tire = new THREE.Mesh(wheelGeo, wheelMat);
      tire.castShadow = true;
      const rim = new THREE.Mesh(rimGeo, rimMat);

      wheelGroup.add(tire);
      wheelGroup.add(rim);

      this.scene.add(wheelGroup);
      this.wheelMeshes.push(wheelGroup);
    }

    this.scene.add(this.carGroup);
  }

  /**
   * Update visual mesh positions from interpolated physics state.
   */
  update(vehicleState) {
    const { position, orientation, wheels } = vehicleState;

    // Chassis transform
    this.carGroup.position.set(position.x, position.y, position.z);
    this.carGroup.quaternion.set(orientation.x, orientation.y, orientation.z, orientation.w);

    // Wheels transforms
    for (let i = 0; i < wheels.length; i++) {
      const w = wheels[i];
      const mesh = this.wheelMeshes[i];

      // Compute world position of wheel from attachment point & suspension compression
      const attachLocal = new THREE.Vector3(w.localAttachPos.x, w.localAttachPos.y - w.suspensionLength, w.localAttachPos.z);
      attachLocal.applyQuaternion(this.carGroup.quaternion);
      mesh.position.copy(this.carGroup.position).add(attachLocal);

      // Wheel rotation: combine chassis yaw + steer angle + wheel roll spin
      mesh.quaternion.copy(this.carGroup.quaternion);

      // Apply steering (yaw)
      const isFront = i < 2;
      const bumpSteer = (w.config ? 0 : 0); // already handled in physics
      if (isFront) {
        mesh.rotateY(w.steerAngle || 0);
      }

      // Apply wheel spin (pitch)
      mesh.rotateX(w.rotationAngle || 0);
    }

    // Smooth Chase Camera
    const idealOffset = new THREE.Vector3(0, 3.5, -7.5);
    idealOffset.applyQuaternion(this.carGroup.quaternion);
    idealOffset.add(this.carGroup.position);

    const idealLookAt = new THREE.Vector3(0, 1.0, 3.0);
    idealLookAt.applyQuaternion(this.carGroup.quaternion);
    idealLookAt.add(this.carGroup.position);

    // Smooth interpolation (lerp)
    this.cameraPos.lerp(idealOffset, 0.1);
    this.cameraTarget.lerp(idealLookAt, 0.15);

    this.camera.position.copy(this.cameraPos);
    this.camera.lookAt(this.cameraTarget);

    // Render
    this.threeRenderer.render(this.scene, this.camera);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.threeRenderer.setSize(window.innerWidth, window.innerHeight);
  }
}
