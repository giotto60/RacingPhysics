/**
 * Vehicle.js — Top-level vehicle assembly combining Chassis, Engine, Suspension, TyreModel, and Wheels.
 */
import { RigidBody } from './RigidBody.js';
import { Engine } from './Engine.js';
import { Suspension } from './Suspension.js';
import { TyreModel } from './TyreModel.js';
import { Wheel } from './Wheel.js';
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';

export class Vehicle {
  constructor(carConfig) {
    this.config = carConfig;

    // Sub-assemblies
    this.rigidBody  = new RigidBody(carConfig.mass, carConfig.inertiaDiag);
    this.engine     = new Engine(carConfig.engine);
    this.suspension = new Suspension(carConfig.suspension);
    this.tyreModel  = new TyreModel(carConfig.tyre.pacejka);

    // Wheels
    this.wheels = carConfig.wheelPositions.map(pos => new Wheel({
      ...pos,
      radius: carConfig.tyre.radius,
      wheelInertia: carConfig.tyre.wheelInertia,
      maxBrakeTorque: carConfig.tyre.maxBrakeTorque
    }));

    this.steeringAngle = 0;
    this.maxSteerAngle = 0.52; // ~30 degrees max steer angle

    // Vehicle dimensions for weight transfer
    this.wheelbase = Math.abs(carConfig.wheelPositions[0].z - carConfig.wheelPositions[2].z); // L
    this.trackWidth = Math.abs(carConfig.wheelPositions[0].x - carConfig.wheelPositions[1].x); // W
    this.cgHeight   = 0.45; // Height of center of mass above ground

    // Pre-allocated scratch vectors
    this.scratchTyreForceWorld = new Vec3();
    this.scratchTyreForceLocal = new Vec3();
    this.scratchResult = { Fx: 0, Fy: 0 };
  }

  /**
   * Reset vehicle to initial position on ground.
   */
  reset(posX = 0, posY = 0.5, posZ = 0) {
    this.rigidBody.position.set(posX, posY, posZ);
    this.rigidBody.velocity.set(0, 0, 0);
    this.rigidBody.acceleration.set(0, 0, 0);
    this.rigidBody.orientation.identity();
    this.rigidBody.angularVelocity.set(0, 0, 0);
    this.engine.currentGear = 1;
    this.engine.currentRPM = this.engine.idleRPM;

    for (const w of this.wheels) {
      w.angularVelocity = 0;
      w.rotationAngle = 0;
    }
  }

  /**
   * Step vehicle physics by timestep dt.
   * @param {number} dt Physics timestep (s)
   * @param {Object} inputs { throttle: 0..1, brake: 0..1, steer: -1..1 }
   */
  step(dt, inputs) {
    this.rigidBody.saveSnapshot();
    this.rigidBody.clearAccumulators();

    // 1. Steering input smoothing
    const targetSteer = inputs.steer * this.maxSteerAngle;
    this.steeringAngle += (targetSteer - this.steeringAngle) * Math.min(1.0, dt * 10);

    // 2. Raycast ground contacts for all wheels & calculate suspension forces
    let totalStaticFz = this.rigidBody.mass * 9.81 / 4;

    for (const wheel of this.wheels) {
      wheel.updateGroundContact(
        this.rigidBody.position,
        this.rigidBody.orientation,
        this.suspension.restLength
      );

      if (wheel.isGrounded) {
        const velComp = (wheel.prevSuspensionLength - wheel.suspensionLength) / dt;
        wheel.Fz = this.suspension.computeForce(wheel.suspensionLength, velComp);
      } else {
        wheel.Fz = 0;
      }
    }

    // 3. Dynamic Weight Transfer (Longitudinal + Lateral)
    // Convert world accelerations to local car body space
    const localAcc = new Vec3();
    this.rigidBody.orientation.inverseRotateVec3(this.rigidBody.acceleration, localAcc);

    // deltaFz_long = (m * accX * CoM_height) / Wheelbase
    // deltaFz_lat  = (m * accZ * CoM_height) / TrackWidth
    const deltaFzLong = (this.rigidBody.mass * localAcc.z * this.cgHeight) / this.wheelbase;
    const deltaFzLat  = (this.rigidBody.mass * localAcc.x * this.cgHeight) / this.trackWidth;

    // Apply weight transfer to corner loads
    // FL (front left)
    if (this.wheels[0].isGrounded) this.wheels[0].Fz += deltaFzLong - deltaFzLat;
    // FR (front right)
    if (this.wheels[1].isGrounded) this.wheels[1].Fz += deltaFzLong + deltaFzLat;
    // RL (rear left)
    if (this.wheels[2].isGrounded) this.wheels[2].Fz += -deltaFzLong - deltaFzLat;
    // RR (rear right)
    if (this.wheels[3].isGrounded) this.wheels[3].Fz += -deltaFzLong + deltaFzLat;

    // Clamp loads >= 0
    for (const w of this.wheels) w.Fz = Math.max(0, w.Fz);

    // 4. Update Engine & Gearbox
    const drivenWheels = this.wheels.filter(w => w.isDriven);
    const avgRadSec = drivenWheels.reduce((sum, w) => sum + w.angularVelocity, 0) / drivenWheels.length;

    this.engine.update(inputs.throttle, inputs.brake, avgRadSec, dt);

    const torquePerWheel = drivenWheels.length > 0 ? this.engine.outputTorque / drivenWheels.length : 0;

    // 5. Calculate Tyre Forces per Wheel
    for (let i = 0; i < this.wheels.length; i++) {
      const wheel = this.wheels[i];
      const isFront = i < 2;

      // Steering angle + bump steer angle offset
      const bumpSteer = this.suspension.getBumpSteerAngle(wheel.suspensionLength);
      const totalWheelSteer = (isFront ? this.steeringAngle : 0) + (isFront ? bumpSteer : -bumpSteer);

      // Compute wheel velocity at contact point
      // V_wheel = V_chassis + (omega_chassis × r_attach)
      Vec3.sub(wheel.worldAttachPos, this.rigidBody.position, this.scratchTyreForceWorld);
      Vec3.cross(this.rigidBody.angularVelocity, this.scratchTyreForceWorld, wheel.wheelVelocity);
      Vec3.add(wheel.wheelVelocity, this.rigidBody.velocity, wheel.wheelVelocity);

      // Compute slip ratio and slip angle
      wheel.computeSlip(this.rigidBody.orientation, totalWheelSteer);

      // Compute Pacejka tyre forces (Fx, Fy)
      this.tyreModel.computeForces(wheel.slipRatio, wheel.slipAngle, wheel.Fz, this.scratchResult);
      wheel.Fx = this.scratchResult.Fx;
      wheel.Fy = this.scratchResult.Fy;

      // Integrate wheel spin speed omega
      const wheelDriveTorque = wheel.isDriven ? torquePerWheel : 0;
      wheel.integrateWheelSpin(wheelDriveTorque, inputs.brake, dt);

      // Apply Tyre forces + Suspension normal force to RigidBody
      if (wheel.isGrounded) {
        // Local tyre force = (Fx * heading) + (Fy * right) + (Fz * normal)
        this.scratchTyreForceWorld.set(0, 0, 0);

        Vec3.addScaled(this.scratchTyreForceWorld, wheel.wheelHeading, wheel.Fx, this.scratchTyreForceWorld);
        Vec3.addScaled(this.scratchTyreForceWorld, wheel.wheelRight, wheel.Fy, this.scratchTyreForceWorld);
        Vec3.addScaled(this.scratchTyreForceWorld, wheel.contactNormal, wheel.Fz, this.scratchTyreForceWorld);

        this.rigidBody.applyForceAtWorldPoint(this.scratchTyreForceWorld, wheel.contactPoint);
      }
    }

    // 6. Integrate Chassis Rigid Body state
    this.rigidBody.integrate(dt);
  }
}
