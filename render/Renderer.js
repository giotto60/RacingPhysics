/**
 * Renderer.js — Three.js Scene with 3000m Ground Grid, Goodyear Tire Markings, and 3D Jump Ramps.
 */
import * as THREE from 'three';

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;

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

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x141c30);
    this.scene.fog = new THREE.FogExp2(0x141c30, 0.003);

    this.camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 1000);
    this.cameraTarget = new THREE.Vector3(0, 0.7, 0);
    this.cameraPos = new THREE.Vector3(0, 3.2, -6.5);
    this.camera.position.copy(this.cameraPos);
    this.camera.lookAt(this.cameraTarget);

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.9);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.6);
    dirLight.position.set(40, 80, 40);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 250;
    dirLight.shadow.camera.left = -80;
    dirLight.shadow.camera.right = 80;
    dirLight.shadow.camera.top = 80;
    dirLight.shadow.camera.bottom = -80;
    this.scene.add(dirLight);

    this.createGround();
    this.createJumpRamps();
    this.createCarMesh();

    window.addEventListener('resize', () => this.onWindowResize());
  }

  createGround() {
    // 3000m x 3000m Large Ground Plane
    const planeGeo = new THREE.PlaneGeometry(3000, 3000);
    const planeMat = new THREE.MeshStandardMaterial({
      color: 0x1a233a,
      roughness: 0.7,
      metalness: 0.1
    });
    const ground = new THREE.Mesh(planeGeo, planeMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(3000, 1500, 0x60a5fa, 0x2e3b59);
    grid.position.y = 0.01;
    this.scene.add(grid);
  }

  createJumpRamps() {
    const rampData = [
      { name: "Speed Ramp (10°)", x: 0, z: 60, width: 10, length: 12, height: 2.0 },
      { name: "Big Launch Ramp (20°)", x: -40, z: 120, width: 12, length: 16, height: 4.5 },
      { name: "Mega Kicker Ramp (30°)", x: 40, z: 180, width: 15, length: 20, height: 7.0 }
    ];

    const rampMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.5,
      metalness: 0.3
    });

    const stripeMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 }); // Caution Yellow

    rampData.forEach(r => {
      const group = new THREE.Group();

      // Wedge geometry for ramp
      const shape = new THREE.Shape();
      shape.moveTo(0, 0);
      shape.lineTo(r.length, r.height);
      shape.lineTo(r.length, 0);
      shape.closePath();

      const extrudeSettings = {
        steps: 1,
        depth: r.width,
        bevelEnabled: false
      };

      const rampGeo = new THREE.ExtrudeGeometry(shape, extrudeSettings);
      rampGeo.rotateY(-Math.PI / 2);
      rampGeo.translate(r.width * 0.5, 0, -r.length * 0.5);

      const rampMesh = new THREE.Mesh(rampGeo, rampMat);
      rampMesh.castShadow = true;
      rampMesh.receiveShadow = true;
      group.add(rampMesh);

      // Yellow caution stripes along ramp lip
      const lipGeo = new THREE.BoxGeometry(r.width, 0.15, 0.3);
      const lipMesh = new THREE.Mesh(lipGeo, stripeMat);
      lipMesh.position.set(0, r.height, r.length * 0.5 - 0.15);
      group.add(lipMesh);

      group.position.set(r.x, 0, r.z);
      this.scene.add(group);
    });
  }

  /**
   * Procedurally generate a 512x512 canvas texture with white "GOODYEAR" lettering.
   */
  createGoodyearTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 512;
    const ctx = canvas.getContext('2d');

    // Tire tread background
    ctx.fillStyle = '#181f2a';
    ctx.fillRect(0, 0, 512, 512);

    // Outer tire wall ring
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 12;
    ctx.beginPath();
    ctx.arc(256, 256, 230, 0, Math.PI * 2);
    ctx.stroke();

    // White rim accent line
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.arc(256, 256, 140, 0, Math.PI * 2);
    ctx.stroke();

    // Goodyear Text along top and bottom arc
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 34px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    // Top text: "GOODYEAR"
    ctx.save();
    ctx.translate(256, 256);
    ctx.fillText('GOODYEAR', 0, -185);
    ctx.restore();

    // Bottom text: "EAGLE F1"
    ctx.save();
    ctx.translate(256, 256);
    ctx.rotate(Math.PI);
    ctx.fillText('EAGLE F1', 0, -185);
    ctx.restore();

    // 5-Spoke Alloy Pattern in center
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 16;
    for (let i = 0; i < 5; i++) {
      const angle = (i * Math.PI * 2) / 5;
      ctx.beginPath();
      ctx.moveTo(256, 256);
      ctx.lineTo(256 + Math.cos(angle) * 130, 256 + Math.sin(angle) * 130);
      ctx.stroke();
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.anisotropy = 8;
    return texture;
  }

  createCarMesh() {
    this.carGroup = new THREE.Group();

    // Central Chassis Tub
    const bodyGeo = new THREE.BoxGeometry(1.2, 0.42, 3.0);
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x2563eb,
      roughness: 0.2,
      metalness: 0.5
    });
    this.bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
    this.bodyMesh.position.y = 0.28;
    this.bodyMesh.castShadow = true;
    this.bodyMesh.receiveShadow = true;
    this.carGroup.add(this.bodyMesh);

    // Aerodynamic Low Canopy Roof
    const roofGeo = new THREE.BoxGeometry(0.95, 0.35, 1.4);
    const roofMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.1,
      metalness: 0.9
    });
    const roofMesh = new THREE.Mesh(roofGeo, roofMat);
    roofMesh.position.set(0, 0.62, -0.1);
    roofMesh.castShadow = true;
    this.carGroup.add(roofMesh);

    // Axle Wishbones
    const axleGeo = new THREE.BoxGeometry(1.84, 0.08, 0.15);
    const axleMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8 });
    const frontAxle = new THREE.Mesh(axleGeo, axleMat);
    frontAxle.position.set(0, 0.12, 1.30);
    const rearAxle = new THREE.Mesh(axleGeo, axleMat);
    rearAxle.position.set(0, 0.12, -1.30);
    this.carGroup.add(frontAxle);
    this.carGroup.add(rearAxle);

    // Headlights
    const lightGeo = new THREE.BoxGeometry(0.28, 0.10, 0.1);
    const hlMat = new THREE.MeshBasicMaterial({ color: 0x93c5fd });
    const hlLeft = new THREE.Mesh(lightGeo, hlMat);
    hlLeft.position.set(-0.42, 0.28, 1.51);
    const hlRight = new THREE.Mesh(lightGeo, hlMat);
    hlRight.position.set(0.42, 0.28, 1.51);
    this.carGroup.add(hlLeft);
    this.carGroup.add(hlRight);

    // Taillights
    const tlMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const tlLeft = new THREE.Mesh(lightGeo, tlMat);
    tlLeft.position.set(-0.42, 0.28, -1.51);
    const tlRight = new THREE.Mesh(lightGeo, tlMat);
    tlRight.position.set(0.42, 0.28, -1.51);
    this.carGroup.add(tlLeft);
    this.carGroup.add(tlRight);

    // Wheels with Goodyear Sidewall Texture
    const goodyearTex = this.createGoodyearTexture();

    const tireFaceMat = new THREE.MeshStandardMaterial({
      map: goodyearTex,
      roughness: 0.4
    });
    const tireTreadMat = new THREE.MeshStandardMaterial({
      color: 0x181f2a,
      roughness: 0.8
    });

    // Materials array: 0: tread, 1: outer face, 2: inner face
    const wheelMaterials = [tireTreadMat, tireFaceMat, tireFaceMat];

    this.wheelMeshes = [];
    const wheelGeo = new THREE.CylinderGeometry(0.30, 0.30, 0.24, 24);
    wheelGeo.rotateZ(Math.PI / 2);

    for (let i = 0; i < 4; i++) {
      const tireMesh = new THREE.Mesh(wheelGeo, wheelMaterials);
      tireMesh.castShadow = true;
      this.scene.add(tireMesh);
      this.wheelMeshes.push(tireMesh);
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
      if (isFront && w.steerAngle !== undefined) {
        mesh.rotateY(-w.steerAngle);
      }

      if (w.rotationAngle) {
        mesh.rotateX(w.rotationAngle);
      }
    }

    // Smooth Chase Camera
    const idealOffset = new THREE.Vector3(0, 2.8, -6.0);
    idealOffset.applyQuaternion(this.carGroup.quaternion);
    idealOffset.add(this.carGroup.position);

    const idealLookAt = new THREE.Vector3(0, 0.7, 2.5);
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
