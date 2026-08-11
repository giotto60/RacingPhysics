/**
 * Engine.js — Engine powertrain & gearbox model.
 *
 * Computes engine RPM, torque output based on throttle & RPM,
 * gear ratio, and automatic gear shifting logic.
 */
export class Engine {
  constructor(config) {
    this.idleRPM     = config.idleRPM || 900;
    this.maxRPM      = config.maxRPM || 7000;
    this.upshiftRPM  = config.upshiftRPM || 6500;
    this.downshiftRPM = config.downshiftRPM || 2500;

    this.torqueCurve  = config.torqueCurve || [[0, 100], [7000, 100]];
    this.gearRatios   = config.gearRatios || [3.5, 2.0, 1.4, 1.0, 0.8];
    this.finalDrive   = config.finalDrive || 3.7;
    this.reverseRatio = config.reverseRatio || 3.2;

    this.currentGear  = 1; // 1 to gearRatios.length, -1 for reverse, 0 for neutral
    this.currentRPM   = this.idleRPM;
    this.outputTorque = 0;

    this.shiftCooldown = 0; // seconds to prevent rapid gear hunting
  }

  /**
   * Sample engine torque (Nm) at given RPM from lookup table.
   */
  getTorqueAtRPM(rpm) {
    const curve = this.torqueCurve;
    if (rpm <= curve[0][0]) return curve[0][1];
    if (rpm >= curve[curve.length - 1][0]) return curve[curve.length - 1][1];

    for (let i = 0; i < curve.length - 1; i++) {
      const p1 = curve[i];
      const p2 = curve[i + 1];
      if (rpm >= p1[0] && rpm <= p2[0]) {
        const t = (rpm - p1[0]) / (p2[0] - p1[0]);
        return p1[1] + t * (p2[1] - p1[1]);
      }
    }
    return 100;
  }

  /**
   * Returns current gear ratio (including final drive).
   */
  getCurrentTotalRatio() {
    if (this.currentGear === 0) return 0;
    if (this.currentGear === -1) return -this.reverseRatio * this.finalDrive;
    return this.gearRatios[this.currentGear - 1] * this.finalDrive;
  }

  /**
   * Update engine state.
   * @param {number} throttle 0..1
   * @param {number} brake 0..1
   * @param {number} drivenWheelsAvgRadSec Average angular velocity (rad/s) of driven wheels
   * @param {number} dt Physics timestep (s)
   */
  update(throttle, brake, drivenWheelsAvgRadSec, dt) {
    if (this.shiftCooldown > 0) {
      this.shiftCooldown -= dt;
    }

    const totalRatio = this.getCurrentTotalRatio();

    if (this.currentGear === 0) {
      // Neutral
      this.currentRPM += (throttle * 4000 - (this.currentRPM - this.idleRPM) * 2) * dt;
    } else {
      // Calculate RPM from wheel speed
      const wheelRPM = (Math.abs(drivenWheelsAvgRadSec) * 60) / (2 * Math.PI);
      const targetRPM = wheelRPM * Math.abs(totalRatio);

      // Smooth transition to target RPM, accounting for engine flywheel inertia
      this.currentRPM += (targetRPM - this.currentRPM) * Math.min(1.0, dt * 15);
    }

    // Clamp RPM
    this.currentRPM = Math.max(this.idleRPM, Math.min(this.maxRPM, this.currentRPM));

    // Auto-gearbox shift logic
    if (this.shiftCooldown <= 0) {
      if (this.currentGear > 0) {
        if (this.currentRPM > this.upshiftRPM && this.currentGear < this.gearRatios.length) {
          this.currentGear++;
          this.shiftCooldown = 0.4;
        } else if (this.currentRPM < this.downshiftRPM && this.currentGear > 1) {
          this.currentGear--;
          this.shiftCooldown = 0.4;
        }
      }
    }

    // Engine torque calculation
    const rawTorque = this.getTorqueAtRPM(this.currentRPM);
    let engineTorque = rawTorque * throttle;

    // Rev limiter cut
    if (this.currentRPM >= this.maxRPM) {
      engineTorque *= 0.1;
    }

    // Engine brake when off throttle
    if (throttle < 0.05) {
      engineTorque = -30 * (this.currentRPM / this.maxRPM);
    }

    // Total axle torque passed to wheels
    this.outputTorque = engineTorque * totalRatio;
  }
}
