/**
 * TyreModel.js — Pacejka Magic Formula Implementation
 *
 * Implements simplified Pacejka formula:
 * F = D * sin(C * atan(B * s - E * (B * s - atan(B * s))))
 *
 * Features:
 * - Load sensitivity: D scales dynamically with Fz
 * - Combined friction circle scaling (ellipse clamping)
 */
export class TyreModel {
  constructor(config) {
    this.longitudinal = config.longitudinal || { B: 12, C: 1.65, E: 0.3 };
    this.lateral      = config.lateral      || { B: 9,  C: 1.35, E: -0.4 };
    this.loadSens     = config.loadSensitivity || { a1: -0.00009, a2: 1.55 };
  }

  /**
   * Calculate peak friction coefficient / force scaling D based on normal load Fz (N).
   * Tyre load sensitivity: higher load = slightly lower effective friction coefficient (mu).
   */
  computePeakForce(Fz) {
    if (Fz <= 0) return 0;
    // D = a1 * Fz^2 + a2 * Fz
    const D = this.loadSens.a1 * Fz * Fz + this.loadSens.a2 * Fz;
    return Math.max(0, D);
  }

  /**
   * Pacejka formula evaluation.
   * s: slip value (slip ratio or slip angle in radians)
   * B, C, E: Pacejka curve parameters
   * D: Peak force
   */
  pacejka(s, B, C, D, E) {
    const Bs = B * s;
    const atanBs = Math.atan(Bs);
    return D * Math.sin(C * Math.atan(Bs - E * (Bs - atanBs)));
  }

  /**
   * Compute combined tyre forces.
   * slipRatio: longitudinal slip [-1, +inf]
   * slipAngle: lateral slip angle in radians
   * Fz: normal force in Newtons
   * outResult: { Fx, Fy }
   */
  computeForces(slipRatio, slipAngle, Fz, outResult) {
    if (Fz <= 0) {
      outResult.Fx = 0;
      outResult.Fy = 0;
      return outResult;
    }

    const D = this.computePeakForce(Fz);
    if (D <= 0) {
      outResult.Fx = 0;
      outResult.Fy = 0;
      return outResult;
    }

    // Pure longitudinal force
    const FxPure = this.pacejka(
      slipRatio,
      this.longitudinal.B,
      this.longitudinal.C,
      D,
      this.longitudinal.E
    );

    // Pure lateral force (slipAngle in rad)
    const FyPure = -this.pacejka(
      slipAngle,
      this.lateral.B,
      this.lateral.C,
      D,
      this.lateral.E
    );

    // Combined friction circle scaling (ellipse coupling)
    // Normalized slips
    const maxFx = D;
    const maxFy = D;

    const normFx = maxFx > 0 ? FxPure / maxFx : 0;
    const normFy = maxFy > 0 ? FyPure / maxFy : 0;

    const combined = Math.sqrt(normFx * normFx + normFy * normFy);

    if (combined > 1.0) {
      outResult.Fx = FxPure / combined;
      outResult.Fy = FyPure / combined;
    } else {
      outResult.Fx = FxPure;
      outResult.Fy = FyPure;
    }

    return outResult;
  }
}
