/**
 * Mat3.js — 3x3 Matrix for rotation and moment of inertia.
 *
 * Stored in column-major order:
 * [ m0 m3 m6 ]
 * [ m1 m4 m7 ]
 * [ m2 m5 m8 ]
 */
import { Vec3 } from './Vec3.js';

export class Mat3 {
  constructor() {
    this.elements = new Float32Array([
      1, 0, 0,
      0, 1, 0,
      0, 0, 1
    ]);
  }

  identity() {
    const e = this.elements;
    e[0] = 1; e[3] = 0; e[6] = 0;
    e[1] = 0; e[4] = 1; e[7] = 0;
    e[2] = 0; e[5] = 0; e[8] = 1;
    return this;
  }

  copy(m) {
    this.elements.set(m.elements);
    return this;
  }

  /** Set matrix from rotation quaternion */
  setFromQuat(q) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    const x2 = x + x, y2 = y + y, z2 = z + z;
    const xx = x * x2, xy = x * y2, xz = x * z2;
    const yy = y * y2, yz = y * z2, zz = z * z2;
    const wx = w * x2, wy = w * y2, wz = w * z2;

    const e = this.elements;
    e[0] = 1 - (yy + zz);
    e[1] = xy + wz;
    e[2] = xz - wy;

    e[3] = xy - wz;
    e[4] = 1 - (xx + zz);
    e[5] = yz + wx;

    e[6] = xz + wy;
    e[7] = yz - wx;
    e[8] = 1 - (xx + yy);

    return this;
  }

  /**
   * Transforms vector v by matrix: out = M * v
   */
  multiplyVec3(v, out) {
    const x = v.x, y = v.y, z = v.z;
    const e = this.elements;
    out.x = e[0] * x + e[3] * y + e[6] * z;
    out.y = e[1] * x + e[4] * y + e[7] * z;
    out.z = e[2] * x + e[5] * y + e[8] * z;
    return out;
  }

  /**
   * Computes world inverse inertia tensor: I_inv_world = R * I_inv_body * R^T
   * R is rotation matrix (from quaternion), I_inv_body is diagonal body-space inverse inertia matrix.
   */
  setWorldInverseInertia(RMat, diagInvInertia) {
    const eR = RMat.elements;
    const ix = diagInvInertia.x, iy = diagInvInertia.y, iz = diagInvInertia.z;

    // R * I_inv_body
    const m0 = eR[0] * ix, m3 = eR[3] * iy, m6 = eR[6] * iz;
    const m1 = eR[1] * ix, m4 = eR[4] * iy, m7 = eR[7] * iz;
    const m2 = eR[2] * ix, m5 = eR[5] * iy, m8 = eR[8] * iz;

    // Multiply by R^T (which is R transposed)
    const eOut = this.elements;
    eOut[0] = m0 * eR[0] + m3 * eR[3] + m6 * eR[6];
    eOut[1] = m1 * eR[0] + m4 * eR[3] + m7 * eR[6];
    eOut[2] = m2 * eR[0] + m5 * eR[3] + m8 * eR[6];

    eOut[3] = m0 * eR[1] + m3 * eR[4] + m6 * eR[7];
    eOut[4] = m1 * eR[1] + m4 * eR[4] + m7 * eR[7];
    eOut[5] = m2 * eR[1] + m5 * eR[4] + m8 * eR[7];

    eOut[6] = m0 * eR[2] + m3 * eR[5] + m6 * eR[8];
    eOut[7] = m1 * eR[2] + m4 * eR[5] + m7 * eR[8];
    eOut[8] = m2 * eR[2] + m5 * eR[5] + m8 * eR[8];

    return this;
  }
}
