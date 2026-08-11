/**
 * Wheel.js — Individual wheel dynamics & ground raycast contact with low-speed stability.
 */
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';

export class Wheel {
  constructor(config) {
    this.id = config.id;
    this.isDriven = config.driven || false;
    this.radius = config.radius || 0.32;
    this.maxBrakeTorque = config.maxBrakeTorque || 3200;
    this.inertia = config.wheelInertia || 1.2;

    // Body-space attachment position
    this.localAttachPos = new Vec3(config.x, config.y, config.z);

    // State
    this.steerAngle = 0;
    this.suspensionRestLength = config.restLength || 0.30;
    this.suspensionLength = this.suspensionRestLength;
    this.prevSuspensionLength = this.suspensionRestLength;
    this.angularVelocity = 0;
    this.rotationAngle = 0;
    this.isGrounded = false;

    // Calculated physics values
    this.slipRatio = 0;
    this.slipAngle = 0;
    this.Fx = 0;
    this.Fy = 0;
    this.Fz = 0;

    // Pre-allocated vectors for zero-allocation hot path
    this.worldAttachPos = new Vec3();
    this.worldRayDir    = new Vec3();
    this.contactPoint   = new Vec3();
    this.contactNormal  = new Vec3(0, 1, 0);
    this.wheelHeading   = new Vec3(0, 0, 1);
    this.wheelRight     = new Vec3(1, 0, 0);
    this.wheelVelocity  = new Vec3();
    this.totalForceWorld= new Vec3();

    this.visualPosition = new Vec3();
    this.visualRotation = new Quat();
  }

  reset() {
    this.steerAngle = 0;
    this.suspensionLength = this.suspensionRestLength;
    this.prevSuspensionLength = this.suspensionRestLength;
    this.angularVelocity = 0;
    this.rotationAngle = 0;
    this.isGrounded = false;
    this.slipRatio = 0;
    this.slipAngle = 0;
    this.Fx = 0;
    this.Fy = 0;
    this.Fz = 0;
  }

  updateGroundContact(chassisPos, chassisQuat) {
    chassisQuat.rotateVec3(this.localAttachPos, this.worldAttachPos);
    Vec3.add(this.worldAttachPos, chassisPos, this.worldAttachPos);

    const downLocal = new Vec3(0, -1, 0);
    chassisQuat.rotateVec3(downLocal, this.worldRayDir);

    const maxDistance = this.suspensionRestLength + this.radius;

    if (this.worldRayDir.y < -1e-5) {
      const distToGround = (this.worldAttachPos.y - 0) / (-this.worldRayDir.y);

      if (distToGround >= 0 && distToGround <= maxDistance) {
        this.isGrounded = true;
        this.prevSuspensionLength = this.suspensionLength;
        this.suspensionLength = Math.max(0.05, distToGround - this.radius);

        Vec3.addScaled(this.worldAttachPos, this.worldRayDir, distToGround, this.contactPoint);
        this.contactNormal.set(0, 1, 0);
        return;
      }
    }

    this.isGrounded = false;
    this.prevSuspensionLength = this.suspensionLength;
    this.suspensionLength = this.suspensionRestLength;
    this.Fz = 0;
    this.Fx = 0;
    this.Fy = 0;
  }

  computeSlip(chassisQuat, totalSteerAngle) {
    this.steerAngle = totalSteerAngle;

    if (!this.isGrounded) {
      this.slipRatio = 0;
      this.slipAngle = 0;
      return;
    }

    const steerQuat = new Quat().setFromAxisAngle(0, 1, 0, totalSteerAngle);
    const localFwd = new Vec3(0, 0, 1);
    const localRight = new Vec3(1, 0, 0);

    steerQuat.rotateVec3(localFwd, localFwd);
    steerQuat.rotateVec3(localRight, localRight);

    chassisQuat.rotateVec3(localFwd, this.wheelHeading);
    chassisQuat.rotateVec3(localRight, this.wheelRight);

    const vLong = Vec3.dot(this.wheelVelocity, this.wheelHeading);
    const vLat  = Vec3.dot(this.wheelVelocity, this.wheelRight);

    const vWheel = this.angularVelocity * this.radius;

    // Longitudinal slip ratio
    const denomRatio = Math.max(Math.abs(vLong), Math.abs(vWheel), 1.0);
    this.slipRatio = (vWheel - vLong) / denomRatio;

    // Lateral slip angle
    this.slipAngle = Math.atan2(vLat, Math.max(0.5, Math.abs(vLong)));

    // Low-speed scaling to prevent micro-chatter at rest
    const speedMag = Math.abs(vLong);
    if (speedMag < 1.0) {
      const lowSpeedFactor = speedMag; // Linear ramp down below 1 m/s
      this.slipRatio *= lowSpeedFactor;
      this.slipAngle *= lowSpeedFactor;
    }
  }

  integrateWheelSpin(driveTorque, brakeInput, dt) {
    const brakeTorque = brakeInput * this.maxBrakeTorque;

    let netTorque = driveTorque;

    if (this.isGrounded) {
      netTorque -= this.Fx * this.radius;
    }

    // Static / Dynamic Brake Torque
    if (brakeTorque > 0) {
      if (Math.abs(this.angularVelocity) < 0.5 && Math.abs(driveTorque) < 10) {
        // Full stop hold
        this.angularVelocity = 0;
        netTorque = 0;
      } else {
        const brakeDir = Math.sign(this.angularVelocity) || 1;
        const mag = Math.min(Math.abs(this.angularVelocity / dt) * this.inertia, brakeTorque);
        netTorque -= brakeDir * mag;
      }
    }

    const alpha = netTorque / this.inertia;
    this.angularVelocity += alpha * dt;

    this.rotationAngle += this.angularVelocity * dt;
  }
}
