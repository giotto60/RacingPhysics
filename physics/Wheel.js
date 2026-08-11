/**
 * Wheel.js — Individual wheel dynamics & ground raycast contact.
 */
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';

export class Wheel {
  constructor(config) {
    this.id = config.id;
    this.isDriven = config.driven || false;
    this.radius = config.radius || 0.32;
    this.maxBrakeTorque = config.maxBrakeTorque || 2500;
    this.inertia = config.wheelInertia || 1.2;

    // Body-space attachment position
    this.localAttachPos = new Vec3(config.x, config.y, config.z);

    // State
    this.steerAngle = 0;       // User steering + bump steer (rad)
    this.suspensionLength = 0; // Current compressed length (m)
    this.prevSuspensionLength = 0;
    this.angularVelocity = 0;  // Wheel rotational speed (rad/s)
    this.rotationAngle = 0;    // Visual wheel spin angle (rad)
    this.isGrounded = false;

    // Calculated physics values
    this.slipRatio = 0;
    this.slipAngle = 0;
    this.Fx = 0; // Longitudinal tyre force (world)
    this.Fy = 0; // Lateral tyre force (world)
    this.Fz = 0; // Normal load (N)

    // Pre-allocated vectors for zero-allocation hot path
    this.worldAttachPos = new Vec3();
    this.worldRayDir    = new Vec3();
    this.contactPoint   = new Vec3();
    this.contactNormal  = new Vec3(0, 1, 0);
    this.wheelHeading   = new Vec3(0, 0, 1);
    this.wheelRight     = new Vec3(1, 0, 0);
    this.wheelVelocity  = new Vec3();
    this.totalForceWorld= new Vec3();

    // Visual transform (interpolated for rendering)
    this.visualPosition = new Vec3();
    this.visualRotation = new Quat();
  }

  /**
   * Perform ground raycast for wheel.
   * For the simple testing environment, ground plane is at y = 0.
   */
  updateGroundContact(chassisPos, chassisQuat, suspensionRestLength) {
    // Compute world attachment position
    chassisQuat.rotateVec3(this.localAttachPos, this.worldAttachPos);
    Vec3.add(this.worldAttachPos, chassisPos, this.worldAttachPos);

    // Raycast direction is chassis down vector
    const downLocal = new Vec3(0, -1, 0);
    chassisQuat.rotateVec3(downLocal, this.worldRayDir);

    // Flat ground collision check (y = 0 plane)
    // Ray: origin = worldAttachPos, dir = worldRayDir
    const maxDistance = suspensionRestLength + this.radius;

    if (this.worldRayDir.y < -1e-5) {
      // Intersection with y = 0
      const distToGround = (this.worldAttachPos.y - 0) / (-this.worldRayDir.y);

      if (distToGround >= 0 && distToGround <= maxDistance) {
        this.isGrounded = true;
        this.prevSuspensionLength = this.suspensionLength;
        this.suspensionLength = Math.max(0.05, distToGround - this.radius);

        // Contact point & normal
        Vec3.addScaled(this.worldAttachPos, this.worldRayDir, distToGround, this.contactPoint);
        this.contactNormal.set(0, 1, 0);
        return;
      }
    }

    // Wheel in air
    this.isGrounded = false;
    this.prevSuspensionLength = this.suspensionLength;
    this.suspensionLength = suspensionRestLength;
    this.Fz = 0;
    this.Fx = 0;
    this.Fy = 0;
  }

  /**
   * Calculate slip ratio and slip angle given wheel velocity at contact point.
   */
  computeSlip(chassisQuat, totalSteerAngle) {
    if (!this.isGrounded) {
      this.slipRatio = 0;
      this.slipAngle = 0;
      return;
    }

    // Combine chassis orientation with steering angle
    const steerQuat = new Quat().setFromAxisAngle(0, 1, 0, totalSteerAngle);
    // Local forward (0,0,1) and right (1,0,0) rotated by chassis then steer angle
    const localFwd = new Vec3(0, 0, 1);
    const localRight = new Vec3(1, 0, 0);

    steerQuat.rotateVec3(localFwd, localFwd);
    steerQuat.rotateVec3(localRight, localRight);

    chassisQuat.rotateVec3(localFwd, this.wheelHeading);
    chassisQuat.rotateVec3(localRight, this.wheelRight);

    // Velocities along wheel axes
    const vLong = Vec3.dot(this.wheelVelocity, this.wheelHeading);
    const vLat  = Vec3.dot(this.wheelVelocity, this.wheelRight);

    // Linear speed equivalent of wheel rotational speed
    const vWheel = this.angularVelocity * this.radius;

    // Longitudinal slip ratio kappa = (vWheel - vLong) / max(|vLong|, |vWheel|, epsilon)
    const denomRatio = Math.max(Math.abs(vLong), Math.abs(vWheel), 1.0);
    this.slipRatio = (vWheel - vLong) / denomRatio;

    // Lateral slip angle alpha = atan2(vLat, |vLong|)
    this.slipAngle = Math.atan2(vLat, Math.max(0.5, Math.abs(vLong)));
  }

  /**
   * Update wheel rotation speed (omega) based on drive torque, brake torque, and tyre force Fx.
   */
  integrateWheelSpin(driveTorque, brakeInput, dt) {
    const brakeTorque = brakeInput * this.maxBrakeTorque;

    // Net torque on wheel: DriveTorque - BrakeTorque - (Fx * radius)
    let netTorque = driveTorque;

    // Tyre reaction torque resisting spin
    if (this.isGrounded) {
      netTorque -= this.Fx * this.radius;
    }

    // Apply brake torque opposing spin
    if (brakeTorque > 0) {
      const brakeDir = Math.sign(this.angularVelocity);
      const mag = Math.min(Math.abs(this.angularVelocity / dt) * this.inertia, brakeTorque);
      netTorque -= brakeDir * mag;
    }

    // Angular acceleration alpha = Torque / Inertia
    const alpha = netTorque / this.inertia;
    this.angularVelocity += alpha * dt;

    // Spin angle for visual rendering
    this.rotationAngle += this.angularVelocity * dt;
  }
}
