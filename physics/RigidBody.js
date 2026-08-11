/**
 * RigidBody.js — 3D Rigid body dynamics with exact chassis bounding box ground contact.
 */
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';
import { Mat3 } from './math/Mat3.js';
import { TrackCollision } from './TrackCollision.js';

export class RigidBody {
  constructor(mass, inertiaDiag) {
    this.mass = mass;
    this.invMass = mass > 0 ? 1 / mass : 0;

    this.bodyInvInertia = new Vec3(
      inertiaDiag[0] > 0 ? 1 / inertiaDiag[0] : 0,
      inertiaDiag[1] > 0 ? 1 / inertiaDiag[1] : 0,
      inertiaDiag[2] > 0 ? 1 / inertiaDiag[2] : 0
    );

    this.position        = new Vec3(0, 0.56, 0);
    this.velocity        = new Vec3();
    this.acceleration    = new Vec3();
    this.orientation     = new Quat();
    this.angularVelocity = new Vec3();

    this.forceAccumulator  = new Vec3();
    this.torqueAccumulator = new Vec3();

    this.worldInvInertia = new Mat3();
    this.rotationMatrix  = new Mat3();

    this.prevPosition    = new Vec3(0, 0.56, 0);
    this.prevOrientation = new Quat();

    // Exact half-extents covering chassis body tub + canopy roof (width: 1.2m, height: 0.76m, length: 3.0m)
    this.boxHalfExtents = new Vec3(0.60, 0.38, 1.50);

    this.scratchR          = new Vec3();
    this.scratchTorque     = new Vec3();
    this.scratchAngAcc     = new Vec3();
    this.scratchCornerLocal= new Vec3();
    this.scratchCornerWorld= new Vec3();
    this.scratchCornerVel  = new Vec3();
    this.scratchCornerForce= new Vec3();
    this.scratchGroundNormal= new Vec3();
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
   * 8-Corner chassis bounding box ground contact.
   * Matches the physical roof and chassis geometry so no part of the car ever breaches the floor.
   */
  resolveChassisGroundCollision() {
    const hx = this.boxHalfExtents.x;
    const hy = this.boxHalfExtents.y;
    const hz = this.boxHalfExtents.z;

    const signsX = [-1, 1, -1, 1, -1, 1, -1, 1];
    const signsY = [-1, -1, 1, 1, -1, -1, 1, 1];
    const signsZ = [-1, -1, -1, -1, 1, 1, 1, 1];

    let isChassisScraping = false;

    for (let i = 0; i < 8; i++) {
      this.scratchCornerLocal.set(signsX[i] * hx, signsY[i] * hy, signsZ[i] * hz);
      this.orientation.rotateVec3(this.scratchCornerLocal, this.scratchCornerWorld);
      Vec3.add(this.scratchCornerWorld, this.position, this.scratchCornerWorld);

      const groundY = TrackCollision.getGroundHeightAndNormal(
        this.scratchCornerWorld.x,
        this.scratchCornerWorld.z,
        this.scratchGroundNormal
      ) + 0.02;

      if (this.scratchCornerWorld.y < groundY) {
        isChassisScraping = true;
        const penetration = groundY - this.scratchCornerWorld.y;

        Vec3.sub(this.scratchCornerWorld, this.position, this.scratchR);
        Vec3.cross(this.angularVelocity, this.scratchR, this.scratchCornerVel);
        Vec3.add(this.scratchCornerVel, this.velocity, this.scratchCornerVel);

        // Penalty spring-damper
        const springK = 60000;
        const damperC = 4000;
        const normalForceMag = Math.max(0, penetration * springK - this.scratchCornerVel.y * damperC);

        Vec3.scale(this.scratchGroundNormal, normalForceMag, this.scratchCornerForce);

        // Chassis metal friction on ground
        const frictionCoeff = 0.6;
        this.scratchCornerForce.x -= this.scratchCornerVel.x * frictionCoeff * 200;
        this.scratchCornerForce.z -= this.scratchCornerVel.z * frictionCoeff * 200;

        this.applyForceAtWorldPoint(this.scratchCornerForce, this.scratchCornerWorld);
      }
    }

    if (isChassisScraping) {
      Vec3.scale(this.angularVelocity, 0.92, this.angularVelocity);
    }
  }

  integrate(dt) {
    this.resolveChassisGroundCollision();

    this.acceleration.x = this.forceAccumulator.x * this.invMass;
    this.acceleration.y = this.forceAccumulator.y * this.invMass - 9.81;
    this.acceleration.z = this.forceAccumulator.z * this.invMass;

    Vec3.addScaled(this.velocity, this.acceleration, dt, this.velocity);
    Vec3.addScaled(this.position, this.velocity, dt, this.position);

    // Hard floor floor safety at half-height (0.38m)
    const minFloorY = TrackCollision.getGroundHeightAndNormal(this.position.x, this.position.z, this.scratchGroundNormal) + 0.38;
    if (this.position.y < minFloorY) {
      this.position.y = minFloorY;
      if (this.velocity.y < 0) this.velocity.y = 0;
    }

    this.updateWorldInertia();
    this.worldInvInertia.multiplyVec3(this.torqueAccumulator, this.scratchAngAcc);

    Vec3.addScaled(this.angularVelocity, this.scratchAngAcc, dt, this.angularVelocity);

    Vec3.scale(this.angularVelocity, Math.max(0, 1 - 0.4 * dt), this.angularVelocity);

    this.orientation.integrateAngularVelocity(this.angularVelocity, dt);
  }
}
