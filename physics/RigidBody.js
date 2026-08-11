/**
 * RigidBody.js — 3D Rigid body dynamics integrator with 8-corner chassis ground collision.
 */
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';
import { Mat3 } from './math/Mat3.js';

export class RigidBody {
  constructor(mass, inertiaDiag) {
    this.mass = mass;
    this.invMass = mass > 0 ? 1 / mass : 0;

    // Body-space diagonal inverse inertia tensor
    this.bodyInvInertia = new Vec3(
      inertiaDiag[0] > 0 ? 1 / inertiaDiag[0] : 0,
      inertiaDiag[1] > 0 ? 1 / inertiaDiag[1] : 0,
      inertiaDiag[2] > 0 ? 1 / inertiaDiag[2] : 0
    );

    // Dynamic state
    this.position        = new Vec3(0, 0.60, 0);
    this.velocity        = new Vec3();
    this.acceleration    = new Vec3();
    this.orientation     = new Quat();
    this.angularVelocity = new Vec3();

    // Accumulators
    this.forceAccumulator  = new Vec3();
    this.torqueAccumulator = new Vec3();

    // World inverse inertia matrix
    this.worldInvInertia = new Mat3();
    this.rotationMatrix  = new Mat3();

    // Previous state snapshot for render interpolation
    this.prevPosition    = new Vec3(0, 0.60, 0);
    this.prevOrientation = new Quat();

    // Chassis Box Bounding Box half-extents (width: 1.6m, height: 0.6m, length: 3.1m)
    this.boxHalfExtents = new Vec3(0.78, 0.32, 1.55);

    // Temp scratch vectors for zero allocation
    this.scratchR          = new Vec3();
    this.scratchTorque     = new Vec3();
    this.scratchAngAcc     = new Vec3();
    this.scratchCornerLocal= new Vec3();
    this.scratchCornerWorld= new Vec3();
    this.scratchCornerVel  = new Vec3();
    this.scratchCornerForce= new Vec3();
  }

  saveSnapshot() {
    this.prevPosition.copy(this.position);
    this.prevOrientation.copy(this.orientation);
  }

  clearAccumulators() {
    this.forceAccumulator.set(0, 0, 0);
    this.torqueAccumulator.set(0, 0, 0);
  }

  applyForceAtWorldPoint(force, point) {
    Vec3.add(this.forceAccumulator, force, this.forceAccumulator);
    Vec3.sub(point, this.position, this.scratchR);
    Vec3.cross(this.scratchR, force, this.scratchTorque);
    Vec3.add(this.torqueAccumulator, this.scratchTorque, this.torqueAccumulator);
  }

  applyForce(force) {
    Vec3.add(this.forceAccumulator, force, this.forceAccumulator);
  }

  updateWorldInertia() {
    this.rotationMatrix.setFromQuat(this.orientation);
    this.worldInvInertia.setWorldInverseInertia(this.rotationMatrix, this.bodyInvInertia);
  }

  /**
   * Check 8 corners of chassis bounding box against floor y = 0.
   * Prevents body or roof from clipping through ground when car tips over or rolls.
   */
  resolveChassisGroundCollision() {
    const hx = this.boxHalfExtents.x;
    const hy = this.boxHalfExtents.y;
    const hz = this.boxHalfExtents.z;

    const groundY = 0.05; // Ground collision threshold plane

    // 8 box corners in body space
    const signsX = [-1, 1, -1, 1, -1, 1, -1, 1];
    const signsY = [-1, -1, 1, 1, -1, -1, 1, 1];
    const signsZ = [-1, -1, -1, -1, 1, 1, 1, 1];

    for (let i = 0; i < 8; i++) {
      this.scratchCornerLocal.set(signsX[i] * hx, signsY[i] * hy, signsZ[i] * hz);

      // Transform corner to world space
      this.orientation.rotateVec3(this.scratchCornerLocal, this.scratchCornerWorld);
      Vec3.add(this.scratchCornerWorld, this.position, this.scratchCornerWorld);

      if (this.scratchCornerWorld.y < groundY) {
        const penetration = groundY - this.scratchCornerWorld.y;

        // Velocity of corner: V_corner = V_body + (omega × r)
        Vec3.sub(this.scratchCornerWorld, this.position, this.scratchR);
        Vec3.cross(this.angularVelocity, this.scratchR, this.scratchCornerVel);
        Vec3.add(this.scratchCornerVel, this.velocity, this.scratchCornerVel);

        // Penalty normal spring + damper force
        const kGround = 120000;
        const cGround = 6000;
        const normalForceMag = Math.max(0, penetration * kGround - this.scratchCornerVel.y * cGround);

        this.scratchCornerForce.set(0, normalForceMag, 0);

        // Ground friction at contact corner
        const frictionCoeff = 0.6;
        const vx = this.scratchCornerVel.x;
        const vz = this.scratchCornerVel.z;
        this.scratchCornerForce.x -= vx * normalForceMag * frictionCoeff * 0.05;
        this.scratchCornerForce.z -= vz * normalForceMag * frictionCoeff * 0.05;

        this.applyForceAtWorldPoint(this.scratchCornerForce, this.scratchCornerWorld);
      }
    }
  }

  /**
   * Semi-implicit Euler integration step.
   */
  integrate(dt) {
    // 1. Resolve chassis box collision vs floor before integrating forces
    this.resolveChassisGroundCollision();

    // 2. Linear velocity & acceleration
    this.acceleration.x = this.forceAccumulator.x * this.invMass;
    this.acceleration.y = this.forceAccumulator.y * this.invMass - 9.81;
    this.acceleration.z = this.forceAccumulator.z * this.invMass;

    Vec3.addScaled(this.velocity, this.acceleration, dt, this.velocity);
    Vec3.addScaled(this.position, this.velocity, dt, this.position);

    // Hard floor floor safety (center of mass never below 0.1m)
    if (this.position.y < 0.1) {
      this.position.y = 0.1;
      if (this.velocity.y < 0) this.velocity.y = 0;
    }

    // 3. Angular velocity & orientation
    this.updateWorldInertia();
    this.worldInvInertia.multiplyVec3(this.torqueAccumulator, this.scratchAngAcc);

    Vec3.addScaled(this.angularVelocity, this.scratchAngAcc, dt, this.angularVelocity);

    // Damping to prevent perpetual floating/wobble
    Vec3.scale(this.angularVelocity, Math.max(0, 1 - 0.8 * dt), this.angularVelocity);

    this.orientation.integrateAngularVelocity(this.angularVelocity, dt);
  }
}
