/**
 * InputManager.js — Keyboard input listener for vehicle controls.
 */
export class InputManager {
  constructor() {
    this.keys = {};

    this.inputs = {
      throttle: 0,
      brake: 0,
      steer: 0,
      reset: false
    };

    window.addEventListener('keydown', (e) => {
      this.keys[e.code] = true;
    });

    window.addEventListener('keyup', (e) => {
      this.keys[e.code] = false;
    });
  }

  update() {
    // Throttle: KeyW or ArrowUp
    const forward = this.keys['KeyW'] || this.keys['ArrowUp'];
    // Brake / Reverse: KeyS or ArrowDown
    const backward = this.keys['KeyS'] || this.keys['ArrowDown'];
    // Steer Left: KeyA or ArrowLeft
    const left = this.keys['KeyA'] || this.keys['ArrowLeft'];
    // Steer Right: KeyD or ArrowRight
    const right = this.keys['KeyD'] || this.keys['ArrowRight'];

    this.inputs.throttle = forward ? 1.0 : 0.0;
    this.inputs.brake = backward ? 1.0 : 0.0;

    let targetSteer = 0;
    if (left) targetSteer -= 1.0;
    if (right) targetSteer += 1.0;

    this.inputs.steer = targetSteer;
    this.inputs.reset = !!this.keys['KeyR'];
  }
}
