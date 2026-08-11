/**
 * Vehicle.js — Top-level vehicle assembly with unconstrained physics & natural Anti-Roll Bars.
 */
import { RigidBody } from './RigidBody.js';
import { Engine } from './Engine.js';
import { Suspension } from './Suspension.js';
import { TyreModel } from './TyreModel.js';
import { Wheel } from './Wheel.js';
import { Vec3 } from './math/Vec3.js';

export class Vehicle {
  constructor(carConfig) {
    this.config = carConfig;

    this.rigidBody  = new RigidBody(carConfig.mass, carConfig.inertiaDiag);
    this.engine     = new Engine(carConfig.engine);
    this.suspension = new Suspension(carConfig.suspension);
    this.tyreModel  = new TyreModel(carConfig.tyre.pacejka);

    this.arbFront = carConfig.suspension.antiRollBarFront || 14000;
    this.arbRear  = carConfig.suspension.antiRollBarRear || 9000;

    this.wheels = carConfig.wheelPositions.map(pos => new Wheel({
      ...pos,
      radius: carConfig.tyre.radius,
      wheelInertia: carConfig.tyre.wheelInertia,
      maxBrakeTorque: carConfig.tyre.maxBrakeTorque,
      restLength: carConfig.suspension.restLength
    }));

    this.steeringAngle = 0;
    this.maxSteerAngle = 0.48; // ~27 degrees constant steering angle

    this.wheelbase = Math.abs(carConfig.wheelPositions[0].z - carConfig.wheelPositions[2].z);
    this.trackWidth = Math.abs(carConfig.wheelPositions[0].x - carConfig.wheelPositions[1].x);

    this.scratchTyreForceWorld = new Vec3();
    this.scratchResult = { Fx: 0, Fy: 0 };

    this.reset(0, 0.56, 0);
  }

  reset(posX = 0, posY = 0.56, posZ = 0) {
    this.rigidBody.position.set(posX, posY, posZ);
    this.rigidBody.velocity.set(0, 0, 0);
    this.rigidBody.acceleration.set(0, 0, 0);
    this.rigidBody.orientation.identity();
    this.rigidBody.angularVelocity.set(0, 0, 0);

    this.rigidBody.prevPosition.copy(this.rigidBody.position);
    this.rigidBody.prevOrientation.copy(this.rigidBody.orientation);

    this.engine.currentGear = 1;
    this.engine.currentRPM = this.engine.idleRPM;
    this.steeringAngle = 0;

    for (const w of this.wheels) {
      w.reset();
    }
  }

  step(dt, inputs) {
    this.rigidBody.saveSnapshot();
    this.rigidBody.clearAccumulators();

    const currentSpeedMs = this.rigidBody.velocity.length();
    const currentSpeedKmH = currentSpeedMs * 3.6;

    if (inputs.brake > 0.1 && inputs.throttle < 0.1 && currentSpeedKmH < 1.5 && this.engine.currentGear > 0) {
      Vec3.scale(this.rigidBody.velocity, 0.75, this.rigidBody.velocity);
      Vec3.scale(this.rigidBody.angularVelocity, 0.75, this.rigidBody.angularVelocity);
    }

    // 1. Steering input smoothing (unconstrained by artificial speed caps)
    const targetSteer = inputs.steer * this.maxSteerAngle;
    this.steeringAngle += (targetSteer - this.steeringAngle) * Math.min(1.0, dt * 10);

    // 2. Raycast ground contacts & compute spring/damper normal forces
    for (const wheel of this.wheels) {
      wheel.updateGroundContact(
        this.rigidBody.position,
        this.rigidBody.orientation
      );

      if (wheel.isGrounded) {
        const velComp = (wheel.prevSuspensionLength - wheel.suspensionLength) / dt;
        wheel.Fz = this.suspension.computeForce(wheel.suspensionLength, velComp);
      } else {
        wheel.Fz = 0;
      }
    }

    // 3. Anti-Roll Bar (ARB) Coupling
    if (this.wheels[0].isGrounded || this.wheels[1].isGrounded) {
      const compFL = this.suspension.restLength - this.wheels[0].suspensionLength;
      const compFR = this.suspension.restLength - this.wheels[1].suspensionLength;
      const arbForceFront = (compFL - compFR) * this.arbFront;
      this.wheels[0].Fz += arbForceFront;
      this.wheels[1].Fz -= arbForceFront;
    }

    if (this.wheels[2].isGrounded || this.wheels[3].isGrounded) {
      const compRL = this.suspension.restLength - this.wheels[2].suspensionLength;
      const compRR = this.suspension.restLength - this.wheels[3].suspensionLength;
      const arbForceRear = (compRL - compRR) * this.arbRear;
      this.wheels[2].Fz += arbForceRear;
      this.wheels[3].Fz -= arbForceRear;
    }

    for (const w of this.wheels) w.Fz = Math.max(0, w.Fz);

    // 4. Update Engine & Gearbox
    const drivenWheels = this.wheels.filter(w => w.isDriven);
    const avgRadSec = drivenWheels.reduce((sum, w) => sum + w.angularVelocity, 0) / drivenWheels.length;

    this.engine.update(inputs.throttle, inputs.brake, currentSpeedKmH, avgRadSec, dt);

    const torquePerWheel = drivenWheels.length > 0 ? this.engine.outputTorque / drivenWheels.length : 0;

    // 5. Calculate Tyre Forces per Wheel
    for (let i = 0; i < this.wheels.length; i++) {
      const wheel = this.wheels[i];
      const isFront = i < 2;

      const bumpSteer = this.suspension.getBumpSteerAngle(wheel.suspensionLength);
      const totalWheelSteer = (isFront ? this.steeringAngle : 0) + (isFront ? bumpSteer : -bumpSteer);

      Vec3.sub(wheel.worldAttachPos, this.rigidBody.position, this.scratchTyreForceWorld);
      Vec3.cross(this.rigidBody.angularVelocity, this.scratchTyreForceWorld, wheel.wheelVelocity);
      Vec3.add(wheel.wheelVelocity, this.rigidBody.velocity, wheel.wheelVelocity);

      wheel.computeSlip(this.rigidBody.orientation, totalWheelSteer);

      this.tyreModel.computeForces(wheel.slipRatio, wheel.slipAngle, wheel.Fz, this.scratchResult);
      wheel.Fx = this.scratchResult.Fx;
      wheel.Fy = this.scratchResult.Fy;

      const wheelDriveTorque = wheel.isDriven ? torquePerWheel : 0;
      const effectiveBrakeInput = this.engine.currentGear === -1 ? 0 : inputs.brake;
      wheel.integrateWheelSpin(wheelDriveTorque, effectiveBrakeInput, dt);

      if (wheel.isGrounded) {
        this.scratchTyreForceWorld.set(0, 0, 0);

        Vec3.addScaled(this.scratchTyreForceWorld, wheel.wheelHeading, wheel.Fx, this.scratchTyreForceWorld);
        Vec3.addScaled(this.scratchTyreForceWorld, wheel.wheelRight, wheel.Fy, this.scratchTyreForceWorld);
        Vec3.addScaled(this.scratchTyreForceWorld, wheel.contactNormal, wheel.Fz, this.scratchTyreForceWorld);

        this.rigidBody.applyForceAtWorldPoint(this.scratchTyreForceWorld, wheel.contactPoint);
      }
    }

    // 6. Integrate Chassis Rigid Body
    this.rigidBody.integrate(dt);
  }
}
