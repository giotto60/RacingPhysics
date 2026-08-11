/**
 * TrackCollision.js — Environment ground height & ramp geometry evaluation.
 * Supports flat ground floor (y = 0) and 3D jump ramps with analytical normals.
 */
import { Vec3 } from './math/Vec3.js';

export const RAMPS = [
  // 1. Speed Ramp (Small)
  {
    id: 'speedRamp',
    x: 0, z: 60,
    width: 10, length: 12, height: 2.0,
    angleDeg: 10
  },
  // 2. Big Launch Ramp (Medium)
  {
    id: 'bigRamp',
    x: -40, z: 120,
    width: 12, length: 16, height: 4.5,
    angleDeg: 20
  },
  // 3. Mega Kicker Ramp (Large)
  {
    id: 'megaRamp',
    x: 40, z: 180,
    width: 15, length: 20, height: 7.0,
    angleDeg: 30
  }
];

export class TrackCollision {
  /**
   * Evaluate ground height y and surface normal vector at given world (x, z).
   * @param {number} x World X
   * @param {number} z World Z
   * @param {Vec3} outNormal Surface normal written into outNormal
   * @returns {number} Ground height Y at (x, z)
   */
  static getGroundHeightAndNormal(x, z, outNormal) {
    outNormal.set(0, 1, 0); // Default flat ground normal
    let groundY = 0;

    for (const ramp of RAMPS) {
      const halfW = ramp.width * 0.5;
      const minZ = ramp.z - ramp.length * 0.5;
      const maxZ = ramp.z + ramp.length * 0.5;

      if (x >= ramp.x - halfW && x <= ramp.x + halfW && z >= minZ && z <= maxZ) {
        // Position along ramp length (0 at start, 1 at crest)
        const t = (z - minZ) / ramp.length;
        groundY = t * ramp.height;

        // Ramp incline angle
        const angleRad = Math.atan2(ramp.height, ramp.length);
        outNormal.x = 0;
        outNormal.y = Math.cos(angleRad);
        outNormal.z = -Math.sin(angleRad);
        break;
      }
    }

    return groundY;
  }
}
