/**
 * RigidBody.js — 3D Rigid body dynamics integrator (Semi-Implicit Euler).
 *
 * All state variables and scratch spaces pre-allocated for zero-GC hot path.
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
    this.position        = new Vec3(0, 0.72, 0);
    this.velocity        = new Vec3();
    this.acceleration    = new Vec3();
    this.orientation     = new Quat(); // Identity (0,0,0,1)
    this.angularVelocity = new Vec3();

    // Accumulators
    this.forceAccumulator  = new Vec3();
    this.torqueAccumulator = new Vec3();

    // World inverse inertia matrix
    this.worldInvInertia = new Mat3();
    this.rotationMatrix  = new Mat3();

    // Previous state snapshot for render interpolation
    this.prevPosition    = new Vec3(0, 0.72, 0);
    this.prevOrientation = new Quat(); // Identity (0,0,0,1)

    // Temp scratch vectors
    this.scratchR     = new Vec3();
    this.scratchTorque= new Vec3();
    this.scratchAngAcc= new Vec3();
  }

  saveSnapshot() {
    this.prevPosition.copy(this.position);
    this.prevOrientation.copy(this.orientation);
  }

  clearAccumulators() {
    this.forceAccumulator.set(0, 0, 0);
    this.torqueAccumulator.set(0, 0, 0);
  }

  /**
   * Apply a force (N) in world-space at a specific world-space point.
   */
  applyForceAtWorldPoint(force, point) {
    Vec3.add(this.forceAccumulator, force, this.forceAccumulator);

    // Torque = (point - CoM) × force
    Vec3.sub(point, this.position, this.scratchR);
    Vec3.cross(this.scratchR, force, this.scratchTorque);
    Vec3.add(this.torqueAccumulator, this.scratchTorque, this.torqueAccumulator);
  }

  /**
   * Apply central force (no torque).
   */
  applyForce(force) {
    Vec3.add(this.forceAccumulator, force, this.forceAccumulator);
  }

  /**
   * Update world inverse inertia matrix based on current orientation.
   */
  updateWorldInertia() {
    this.rotationMatrix.setFromQuat(this.orientation);
    this.worldInvInertia.setWorldInverseInertia(this.rotationMatrix, this.bodyInvInertia);
  }

  /**
   * Semi-implicit Euler step:
   * 1. velocity += (force / mass + gravity) * dt
   * 2. position += velocity * dt
   * 3. angularVelocity += (I_inv * torque) * dt
   * 4. orientation += 0.5 * omega * orientation * dt
   */
  integrate(dt) {
    // 1. Linear velocity & acceleration
    this.acceleration.x = this.forceAccumulator.x * this.invMass;
    this.acceleration.y = this.forceAccumulator.y * this.invMass - 9.81; // Add gravity
    this.acceleration.z = this.forceAccumulator.z * this.invMass;

    Vec3.addScaled(this.velocity, this.acceleration, dt, this.velocity);
    Vec3.addScaled(this.position, this.velocity, dt, this.position);

    // Ground plane collision floor (prevents clipping below y=0.15)
    if (this.position.y < 0.15) {
      this.position.y = 0.15;
      if (this.velocity.y < 0) this.velocity.y = 0;
    }

    // 2. Angular velocity & orientation
    this.updateWorldInertia();
    this.worldInvInertia.multiplyVec3(this.torqueAccumulator, this.scratchAngAcc);

    Vec3.addScaled(this.angularVelocity, this.scratchAngAcc, dt, this.angularVelocity);

    // Apply angular velocity damping (air resistance / internal friction)
    Vec3.scale(this.angularVelocity, Math.max(0, 1 - 0.5 * dt), this.angularVelocity);

    // Integrate quaternion
    this.orientation.integrateAngularVelocity(this.angularVelocity, dt);
  }
}
