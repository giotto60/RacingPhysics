/**
 * RigidBody.js — 3D Rigid body dynamics with energy-absorbing impulse ground collision & tumbling friction.
 */
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';
import { Mat3 } from './math/Mat3.js';

export class RigidBody {
  constructor(mass, inertiaDiag) {
    this.mass = mass;
    this.invMass = mass > 0 ? 1 / mass : 0;

    this.bodyInvInertia = new Vec3(
      inertiaDiag[0] > 0 ? 1 / inertiaDiag[0] : 0,
      inertiaDiag[1] > 0 ? 1 / inertiaDiag[1] : 0,
      inertiaDiag[2] > 0 ? 1 / inertiaDiag[2] : 0
    );

    this.position        = new Vec3(0, 0.50, 0);
    this.velocity        = new Vec3();
    this.acceleration    = new Vec3();
    this.orientation     = new Quat();
    this.angularVelocity = new Vec3();

    this.forceAccumulator  = new Vec3();
    this.torqueAccumulator = new Vec3();

    this.worldInvInertia = new Mat3();
    this.rotationMatrix  = new Mat3();

    this.prevPosition    = new Vec3(0, 0.50, 0);
    this.prevOrientation = new Quat();

    // Chassis Bounding Box Half-extents
    this.boxHalfExtents = new Vec3(0.60, 0.25, 1.50);

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
   * Realistic Chassis Ground Contact & Tumbling Friction.
   * Energy-absorbing impulse model prevents artificial catapulting / mid-air launch.
   */
  resolveChassisGroundCollision() {
    const hx = this.boxHalfExtents.x;
    const hy = this.boxHalfExtents.y;
    const hz = this.boxHalfExtents.z;
    const groundY = 0.05;

    const signsX = [-1, 1, -1, 1, -1, 1, -1, 1];
    const signsY = [-1, -1, 1, 1, -1, -1, 1, 1];
    const signsZ = [-1, -1, -1, -1, 1, 1, 1, 1];

    let isChassisScraping = false;

    for (let i = 0; i < 8; i++) {
      this.scratchCornerLocal.set(signsX[i] * hx, signsY[i] * hy, signsZ[i] * hz);
      this.orientation.rotateVec3(this.scratchCornerLocal, this.scratchCornerWorld);
      Vec3.add(this.scratchCornerWorld, this.position, this.scratchCornerWorld);

      if (this.scratchCornerWorld.y < groundY) {
        isChassisScraping = true;
        const penetration = groundY - this.scratchCornerWorld.y;

        // V_corner = V_body + (omega × r)
        Vec3.sub(this.scratchCornerWorld, this.position, this.scratchR);
        Vec3.cross(this.angularVelocity, this.scratchR, this.scratchCornerVel);
        Vec3.add(this.scratchCornerVel, this.velocity, this.scratchCornerVel);

        // Highly damped contact force: restitution e = 0.1 absorbs crash energy
        const springK = 35000;
        const damperC = 3000;
        let normalForceMag = Math.max(0, penetration * springK - this.scratchCornerVel.y * damperC);

        // Cap max normal force to 1.5x gravity to eliminate explosive energy creation
        const maxForcePerCorner = (this.mass * 9.81 * 1.5) / 4;
        normalForceMag = Math.min(maxForcePerCorner, normalForceMag);

        this.scratchCornerForce.set(0, normalForceMag, 0);

        // Strong chassis friction when metal scrapes asphalt (dissipates sliding & tumbling)
        const frictionCoeff = 0.75;
        this.scratchCornerForce.x -= this.scratchCornerVel.x * frictionCoeff * 500;
        this.scratchCornerForce.z -= this.scratchCornerVel.z * frictionCoeff * 500;

        this.applyForceAtWorldPoint(this.scratchCornerForce, this.scratchCornerWorld);
      }
    }

    // Heavy angular velocity damping while chassis is scraping ground (realistic crash rest)
    if (isChassisScraping) {
      Vec3.scale(this.angularVelocity, 0.90, this.angularVelocity);
    }
  }

  integrate(dt) {
    this.resolveChassisGroundCollision();

    this.acceleration.x = this.forceAccumulator.x * this.invMass;
    this.acceleration.y = this.forceAccumulator.y * this.invMass - 9.81;
    this.acceleration.z = this.forceAccumulator.z * this.invMass;

    Vec3.addScaled(this.velocity, this.acceleration, dt, this.velocity);
    Vec3.addScaled(this.position, this.velocity, dt, this.position);

    // Floor floor safety
    if (this.position.y < 0.1) {
      this.position.y = 0.1;
      if (this.velocity.y < 0) this.velocity.y = 0;
    }

    this.updateWorldInertia();
    this.worldInvInertia.multiplyVec3(this.torqueAccumulator, this.scratchAngAcc);

    Vec3.addScaled(this.angularVelocity, this.scratchAngAcc, dt, this.angularVelocity);

    // Air & rotation damping
    Vec3.scale(this.angularVelocity, Math.max(0, 1 - 0.5 * dt), this.angularVelocity);

    this.orientation.integrateAngularVelocity(this.angularVelocity, dt);
  }
}
