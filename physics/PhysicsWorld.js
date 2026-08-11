/**
 * PhysicsWorld.js — Fixed-timestep accumulator loop for 120Hz physics simulation.
 */
import { Vehicle } from './Vehicle.js';
import { Vec3 } from './math/Vec3.js';
import { Quat } from './math/Quat.js';

export class PhysicsWorld {
  constructor(carConfig) {
    this.fixedDt = 1 / 120; // 120 Hz fixed physics step
    this.accumulator = 0;

    this.vehicle = new Vehicle(carConfig);
    this.vehicle.reset(0, 0.72, 0);

    // Render interpolation objects
    this.interpPosition = new Vec3(0, 0.72, 0);
    this.interpOrientation = new Quat();
  }

  /**
   * Main update loop called every render frame with variable deltaTime.
   * Runs N fixed physics steps to stay in sync with real time.
   */
  update(frameDeltaTime, inputs) {
    // Prevent spiral of death on long frame lags (max 0.1s accum)
    const dt = Math.min(0.1, frameDeltaTime);
    this.accumulator += dt;

    while (this.accumulator >= this.fixedDt) {
      this.vehicle.step(this.fixedDt, inputs);
      this.accumulator -= this.fixedDt;
    }
  }

  /**
   * Get interpolated vehicle transform for smooth rendering at any frame rate.
   * alpha = remainder accumulator / fixedDt
   */
  getInterpolatedVehicleState() {
    const alpha = this.accumulator / this.fixedDt;
    const rb = this.vehicle.rigidBody;

    Vec3.lerp(rb.prevPosition, rb.position, alpha, this.interpPosition);
    Quat.slerp(rb.prevOrientation, rb.orientation, alpha, this.interpOrientation);

    return {
      position: this.interpPosition,
      orientation: this.interpOrientation,
      wheels: this.vehicle.wheels,
      speedKmH: rb.velocity.length() * 3.6,
      rpm: this.vehicle.engine.currentRPM,
      gear: this.vehicle.engine.currentGear
    };
  }
}
