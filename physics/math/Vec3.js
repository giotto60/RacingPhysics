/**
 * Vec3.js — Lightweight 3D vector.
 *
 * Design rule: ALL static operations write into a pre-allocated `out` parameter.
 * NEVER call `new Vec3()` in the hot physics loop — allocate once, reuse forever.
 */
export class Vec3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  set(x, y, z)   { this.x = x; this.y = y; this.z = z; return this; }
  copy(v)         { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
  clone()         { return new Vec3(this.x, this.y, this.z); }

  length()        { return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z); }
  lengthSq()      { return this.x * this.x + this.y * this.y + this.z * this.z; }

  normalize() {
    const l = this.length();
    if (l > 1e-9) { this.x /= l; this.y /= l; this.z /= l; }
    return this;
  }

  negate() { this.x = -this.x; this.y = -this.y; this.z = -this.z; return this; }

  // ── Static zero-alloc operations ─────────────────────────────────────────

  /** out = a + b */
  static add(a, b, out) {
    out.x = a.x + b.x; out.y = a.y + b.y; out.z = a.z + b.z;
    return out;
  }

  /** out = a - b */
  static sub(a, b, out) {
    out.x = a.x - b.x; out.y = a.y - b.y; out.z = a.z - b.z;
    return out;
  }

  /** out = a * scalar */
  static scale(a, s, out) {
    out.x = a.x * s; out.y = a.y * s; out.z = a.z * s;
    return out;
  }

  /** out = a + b * scalar */
  static addScaled(a, b, s, out) {
    out.x = a.x + b.x * s; out.y = a.y + b.y * s; out.z = a.z + b.z * s;
    return out;
  }

  /** Dot product */
  static dot(a, b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

  /** out = a × b (cross product) */
  static cross(a, b, out) {
    // Store intermediates to allow out === a or out === b
    const ax = a.x, ay = a.y, az = a.z;
    const bx = b.x, by = b.y, bz = b.z;
    out.x = ay * bz - az * by;
    out.y = az * bx - ax * bz;
    out.z = ax * by - ay * bx;
    return out;
  }

  /** Linear interpolation: out = a + (b - a) * t */
  static lerp(a, b, t, out) {
    out.x = a.x + (b.x - a.x) * t;
    out.y = a.y + (b.y - a.y) * t;
    out.z = a.z + (b.z - a.z) * t;
    return out;
  }

  /** out = normalize(v) */
  static normalize(v, out) {
    const l = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (l > 1e-9) { out.x = v.x / l; out.y = v.y / l; out.z = v.z / l; }
    else          { out.x = 0; out.y = 0; out.z = 0; }
    return out;
  }
}
