/**
 * Wheel.js — Corrected lateral force directional vector & steering alignment.
 */
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';

export class Wheel {
  constructor(config) {
    this.id = config.id;
    this.isDriven = config.driven || false;
    this.radius = config.radius || 0.30;
    this.maxBrakeTorque = config.maxBrakeTorque || 3200;
    this.inertia = config.wheelInertia || 2.2;

    this.localAttachPos = new Vec3(config.x, config.y, config.z);

    this.steerAngle = 0;
    this.suspensionRestLength = config.restLength || 0.22;
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

    this.worldAttachPos = new Vec3();
    this.worldRayDir    = new Vec3();
    this.contactPoint   = new Vec3();
    this.contactNormal  = new Vec3(0, 1, 0);
    this.wheelHeading   = new Vec3(0, 0, 1);
    this.wheelRight     = new Vec3(1, 0, 0);
    this.wheelVelocity  = new Vec3();
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

    const maxDistance = this.suspensionRestLength + this.radius + 0.08;

    if (this.worldRayDir.y < -1e-5) {
      const distToGround = (this.worldAttachPos.y - 0) / (-this.worldRayDir.y);

      if (distToGround >= 0 && distToGround <= maxDistance) {
        this.isGrounded = true;
        this.prevSuspensionLength = this.suspensionLength;
        this.suspensionLength = Math.max(0.02, Math.min(this.suspensionRestLength, distToGround - this.radius));

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

    // Combine chassis orientation with steering angle
    // In our coordinate system, steering right (+angle) rotates wheel heading clockwise around Y
    const steerQuat = new Quat().setFromAxisAngle(0, 1, 0, -totalSteerAngle);
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

    // Lateral slip angle (angle between heading and velocity)
    this.slipAngle = Math.atan2(vLat, Math.max(0.5, Math.abs(vLong)));

    // Low-speed activity damping
    const totalActivity = Math.abs(vLong) + Math.abs(vWheel);
    if (totalActivity < 0.5) {
      const lowSpeedDamp = totalActivity / 0.5;
      this.slipRatio *= lowSpeedDamp;
      this.slipAngle *= lowSpeedDamp;
    }
  }

  integrateWheelSpin(driveTorque, brakeInput, dt) {
    const brakeTorque = brakeInput * this.maxBrakeTorque;

    let netTorque = driveTorque;

    if (this.isGrounded) {
      netTorque -= this.Fx * this.radius;
    }

    if (brakeTorque > 0) {
      if (Math.abs(this.angularVelocity) < 0.5 && Math.abs(driveTorque) < 10) {
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
