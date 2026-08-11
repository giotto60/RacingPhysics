/**
 * main.js — Application entry point with live physics logging and HUD.
 */
import { PhysicsWorld } from './physics/PhysicsWorld.js';
import { InputManager } from './game/InputManager.js';
import { Renderer } from './render/Renderer.js';

let carSpec = null;
let physicsWorld = null;
let inputManager = null;
let renderer = null;

let lastTime = performance.now();
let frameCount = 0;

// HUD elements
const speedValueEl  = document.getElementById('speed-value');
const gearValueEl   = document.getElementById('gear-value');
const rpmBarFillEl  = document.getElementById('rpm-bar-fill');
const rpmValueEl    = document.getElementById('rpm-value');
const throttleFill  = document.getElementById('throttle-fill');
const brakeFill     = document.getElementById('brake-fill');
const steerFill     = document.getElementById('steer-fill');

const debugBodyY    = document.getElementById('debug-body-y');
const debugAccel    = document.getElementById('debug-accel');
const debugRoll     = document.getElementById('debug-roll');
const debugFx       = document.getElementById('debug-fx');
const debugFy       = document.getElementById('debug-fy');
const debugFz       = document.getElementById('debug-fz');

const teleLoads = [
  document.getElementById('tl-fl'),
  document.getElementById('tl-fr'),
  document.getElementById('tl-rl'),
  document.getElementById('tl-rr')
];
const teleSlips = [
  document.getElementById('ts-fl'),
  document.getElementById('ts-fr'),
  document.getElementById('ts-rl'),
  document.getElementById('ts-rr')
];

async function init() {
  try {
    const canvas = document.getElementById('canvas');

    const res = await fetch('./data/cars/default.json');
    if (!res.ok) {
      throw new Error(`Failed to load car spec JSON: HTTP ${res.status}`);
    }
    carSpec = await res.json();

    physicsWorld = new PhysicsWorld(carSpec);
    inputManager = new InputManager();
    renderer = new Renderer(canvas);

    lastTime = performance.now();
    requestAnimationFrame(gameLoop);
  } catch (err) {
    if (window.showError) window.showError(`[Init Failed] ${err.stack || err.message}`);
    console.error(err);
  }
}

function gameLoop(now) {
  try {
    const dt = Math.min(0.1, (now - lastTime) / 1000);
    lastTime = now;
    frameCount++;

    inputManager.update();
    const inputs = inputManager.inputs;

    if (inputs.reset) {
      physicsWorld.vehicle.reset(0, 0.50, 0);
    }

    physicsWorld.update(dt, inputs);

    const state = physicsWorld.getInterpolatedVehicleState();

    renderer.update(state);

    updateHUD(state, inputs);

    // Console logging every 120 frames (~2 seconds)
    if (frameCount % 120 === 0) {
      const rb = physicsWorld.vehicle.rigidBody;
      console.log(`[Physics Log #${frameCount}] Speed: ${Math.round(state.speedKmH)} km/h | Body Y: ${rb.position.y.toFixed(2)}m | Roll: ${(rb.orientation.z * 57.3).toFixed(1)}° | Acc: (${rb.acceleration.x.toFixed(1)}, ${rb.acceleration.y.toFixed(1)}, ${rb.acceleration.z.toFixed(1)})`);
    }
  } catch (err) {
    if (window.showError) window.showError(`[Frame Error] ${err.stack || err.message}`);
    console.error(err);
    return;
  }

  requestAnimationFrame(gameLoop);
}

function updateHUD(state, inputs) {
  speedValueEl.textContent = Math.round(state.speedKmH);
  gearValueEl.textContent = state.gear > 0 ? state.gear : (state.gear === 0 ? 'N' : 'R');

  const rpm = Math.round(state.rpm);
  const maxRPM = carSpec.engine.maxRPM;
  const rpmPct = Math.min(100, Math.max(0, (rpm / maxRPM) * 100));
  rpmBarFillEl.style.width = `${rpmPct}%`;
  rpmValueEl.textContent = `${rpm} RPM`;

  throttleFill.style.width = `${inputs.throttle * 100}%`;
  brakeFill.style.width    = `${inputs.brake * 100}%`;

  const steerPct = Math.abs(inputs.steer) * 50;
  if (inputs.steer < 0) {
    steerFill.style.left = `${50 - steerPct}%`;
    steerFill.style.width = `${steerPct}%`;
  } else {
    steerFill.style.left = '50%';
    steerFill.style.width = `${steerPct}%`;
  }

  const wheels = state.wheels;
  let totalFx = 0, totalFy = 0, totalFz = 0;

  for (let i = 0; i < 4; i++) {
    if (wheels[i]) {
      teleLoads[i].textContent = `${Math.round(wheels[i].Fz)} N`;
      const slipDeg = (wheels[i].slipAngle * (180 / Math.PI)).toFixed(1);
      teleSlips[i].textContent = `${slipDeg}°`;

      totalFx += wheels[i].Fx || 0;
      totalFy += wheels[i].Fy || 0;
      totalFz += wheels[i].Fz || 0;
    }
  }

  // Live Physics Debug Panel
  const rb = physicsWorld.vehicle.rigidBody;
  debugBodyY.textContent = `Body Y: ${rb.position.y.toFixed(2)} m`;
  debugAccel.textContent = `Acc (X/Y/Z): ${rb.acceleration.x.toFixed(1)} / ${rb.acceleration.y.toFixed(1)} / ${rb.acceleration.z.toFixed(1)}`;
  debugRoll.textContent  = `Roll / Pitch: ${(rb.orientation.z * 57.3).toFixed(1)}° / ${(rb.orientation.x * 57.3).toFixed(1)}°`;
  debugFx.textContent    = `Fx Total: ${Math.round(totalFx)} N`;
  debugFy.textContent    = `Fy Total: ${Math.round(totalFy)} N`;
  debugFz.textContent    = `Fz Total: ${Math.round(totalFz)} N`;
}

init();
