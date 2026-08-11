/**
 * Engine.js — Engine powertrain & gearbox model with seamless Forward / Reverse transitions.
 */
export class Engine {
  constructor(config) {
    this.idleRPM      = config.idleRPM || 900;
    this.maxRPM       = config.maxRPM || 7800;
    this.upshiftRPM   = config.upshiftRPM || 6600;
    this.downshiftRPM = config.downshiftRPM || 2600;

    this.torqueCurve  = config.torqueCurve || [[0, 140], [7800, 200]];
    this.gearRatios   = config.gearRatios || [3.6, 2.2, 1.5, 1.1, 0.85, 0.72];
    this.finalDrive   = config.finalDrive || 3.9;
    this.reverseRatio = config.reverseRatio || 3.3;

    this.currentGear  = 1;
    this.currentRPM   = this.idleRPM;
    this.outputTorque = 0;
    this.shiftCooldown = 0;
  }

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
    return 140;
  }

  getCurrentTotalRatio() {
    if (this.currentGear === 0) return 0;
    if (this.currentGear === -1) return -this.reverseRatio * this.finalDrive;
    return this.gearRatios[this.currentGear - 1] * this.finalDrive;
  }

  update(throttle, brake, speedKmH, drivenWheelsAvgRadSec, dt) {
    if (this.shiftCooldown > 0) {
      this.shiftCooldown -= dt;
    }

    // Seamless Forward / Reverse Gear Transitions
    if (this.shiftCooldown <= 0) {
      // In Forward gear: switch to Reverse if nearly stopped (or rolling back) and pressing Brake (S)
      if (this.currentGear > 0 && speedKmH < 3.0 && brake > 0.2) {
        this.currentGear = -1;
        this.shiftCooldown = 0.3;
      }
      // In Reverse gear: switch to Forward 1st gear if nearly stopped (or rolling forward) and pressing Throttle (W)
      else if (this.currentGear === -1 && speedKmH < 3.0 && throttle > 0.2) {
        this.currentGear = 1;
        this.shiftCooldown = 0.3;
      }
    }

    // Map pedal inputs according to current gear
    let effectiveThrottle = 0;
    if (this.currentGear > 0) {
      effectiveThrottle = throttle;
    } else if (this.currentGear === -1) {
      // In Reverse, Brake pedal acts as reverse throttle
      effectiveThrottle = brake;
    }

    const totalRatio = this.getCurrentTotalRatio();

    if (this.currentGear === 0) {
      this.currentRPM += (effectiveThrottle * 5000 - (this.currentRPM - this.idleRPM) * 3) * dt;
    } else {
      const wheelRPM = (Math.abs(drivenWheelsAvgRadSec) * 60) / (2 * Math.PI);
      const targetRPM = Math.max(this.idleRPM + effectiveThrottle * 1500, wheelRPM * Math.abs(totalRatio));
      this.currentRPM += (targetRPM - this.currentRPM) * Math.min(1.0, dt * 15);
    }

    this.currentRPM = Math.max(this.idleRPM, Math.min(this.maxRPM, this.currentRPM));

    // Auto upshift / downshift in forward gears
    if (this.shiftCooldown <= 0 && this.currentGear > 0) {
      if (this.currentRPM > this.upshiftRPM && this.currentGear < this.gearRatios.length) {
        this.currentGear++;
        this.shiftCooldown = 0.4;
      } else if (this.currentRPM < this.downshiftRPM && this.currentGear > 1) {
        this.currentGear--;
        this.shiftCooldown = 0.4;
      }
    }

    const rawTorque = this.getTorqueAtRPM(this.currentRPM);
    let engineTorque = rawTorque * effectiveThrottle;

    if (this.currentRPM >= this.maxRPM) {
      engineTorque *= 0.1;
    }

    if (effectiveThrottle < 0.05) {
      engineTorque = -40 * (this.currentRPM / this.maxRPM);
    }

    this.outputTorque = engineTorque * totalRatio;
  }
}
