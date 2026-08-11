/**
 * Quat.js — Unit quaternion for 3D rotation.
 *
 * Convention: (x, y, z, w) where w is the scalar part.
 * All operations assume unit quaternions.
 */
export class Quat {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x; this.y = y; this.z = z; this.w = w;
  }

  set(x, y, z, w) { this.x = x; this.y = y; this.z = z; this.w = w; return this; }
  copy(q)         { this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w; return this; }
  clone()         { return new Quat(this.x, this.y, this.z, this.w); }
  identity()      { this.x = 0; this.y = 0; this.z = 0; this.w = 1; return this; }

  normalize() {
    const l = Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z + this.w*this.w);
    if (l > 1e-9) { this.x /= l; this.y /= l; this.z /= l; this.w /= l; }
    return this;
  }

  /**
   * Integrate angular velocity (omega, world-space rad/s) over timestep dt.
   * Updates orientation in-place using first-order approximation.
   * Equivalent to: q += 0.5 * [omega, 0] * q * dt
   */
  integrateAngularVelocity(omega, dt) {
    const half = 0.5 * dt;
    const ox = omega.x * half, oy = omega.y * half, oz = omega.z * half;
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
    this.x += qw * ox - qz * oy + qy * oz;
    this.y += qw * oy + qz * ox - qx * oz;
    this.z += qw * oz - qy * ox + qx * oy;
    this.w -= qx * ox + qy * oy + qz * oz;
    return this.normalize();
  }

  /**
   * Rotate a point/direction vector by this quaternion.
   * Uses the optimised formula: v' = v + 2w(q×v) + 2(q×(q×v))
   * Writes result into `out`. Safe when out !== v.
   */
  rotateVec3(v, out) {
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
    const vx = v.x, vy = v.y, vz = v.z;
    // t = 2 * cross(q.xyz, v)
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    out.x = vx + qw * tx + qy * tz - qz * ty;
    out.y = vy + qw * ty + qz * tx - qx * tz;
    out.z = vz + qw * tz + qx * ty - qy * tx;
    return out;
  }

  /**
   * Rotate by the inverse (conjugate) of this quaternion.
   * Equivalent to rotating from world-space back to local/body-space.
   */
  inverseRotateVec3(v, out) {
    const qx = -this.x, qy = -this.y, qz = -this.z, qw = this.w;
    const vx = v.x, vy = v.y, vz = v.z;
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    out.x = vx + qw * tx + qy * tz - qz * ty;
    out.y = vy + qw * ty + qz * tx - qx * tz;
    out.z = vz + qw * tz + qx * ty - qy * tx;
    return out;
  }

  /** Spherical linear interpolation between a and b, result into out. */
  static slerp(a, b, t, out) {
    let ax = a.x, ay = a.y, az = a.z, aw = a.w;
    let bx = b.x, by = b.y, bz = b.z, bw = b.w;
    let dot = ax*bx + ay*by + az*bz + aw*bw;
    // Ensure shortest path
    if (dot < 0) { bx=-bx; by=-by; bz=-bz; bw=-bw; dot=-dot; }
    if (dot > 0.9999) {
      // Near identical — linear interpolate + normalize
      out.x = ax+(bx-ax)*t; out.y = ay+(by-ay)*t;
      out.z = az+(bz-az)*t; out.w = aw+(bw-aw)*t;
      return out.normalize();
    }
    const theta0 = Math.acos(dot);
    const theta  = theta0 * t;
    const sinT0  = Math.sin(theta0);
    const s0 = Math.cos(theta) - dot * Math.sin(theta) / sinT0;
    const s1 = Math.sin(theta) / sinT0;
    out.x = ax*s0 + bx*s1; out.y = ay*s0 + by*s1;
    out.z = az*s0 + bz*s1; out.w = aw*s0 + bw*s1;
    return out;
  }

  /**
   * Set from axis-angle. axis must be a unit vector.
   * angle in radians.
   */
  setFromAxisAngle(ax, ay, az, angle) {
    const half = angle * 0.5;
    const s = Math.sin(half);
    this.x = ax * s; this.y = ay * s; this.z = az * s; this.w = Math.cos(half);
    return this;
  }
}
