/**
 * main.js — Application entry point.
 * Wires Physics, Renderer, InputManager, and HUD telemetry.
 */
import { PhysicsWorld } from './physics/PhysicsWorld.js';
import { InputManager } from './game/InputManager.js';
import { Renderer } from './render/Renderer.js';

let carSpec = null;
let physicsWorld = null;
let inputManager = null;
let renderer = null;

let lastTime = performance.now();

// HUD elements
const speedValueEl  = document.getElementById('speed-value');
const gearValueEl   = document.getElementById('gear-value');
const rpmBarFillEl  = document.getElementById('rpm-bar-fill');
const rpmValueEl    = document.getElementById('rpm-value');
const throttleFill  = document.getElementById('throttle-fill');
const brakeFill     = document.getElementById('brake-fill');
const steerFill     = document.getElementById('steer-fill');

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

    // Load car spec
    const res = await fetch('./data/cars/default.json');
    if (!res.ok) {
      throw new Error(`Failed to load car spec JSON: HTTP ${res.status}`);
    }
    carSpec = await res.json();

    // Instantiate sub-systems
    physicsWorld = new PhysicsWorld(carSpec);
    inputManager = new InputManager();
    renderer = new Renderer(canvas);

    // Start game loop
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

    // 1. Process user controls
    inputManager.update();
    const inputs = inputManager.inputs;

    if (inputs.reset) {
      physicsWorld.vehicle.reset(0, 0.72, 0);
    }

    // 2. Step physics engine (120Hz fixed step accumulator)
    physicsWorld.update(dt, inputs);

    // 3. Get interpolated state for smooth render frame
    const state = physicsWorld.getInterpolatedVehicleState();

    // 4. Update Three.js scene & render
    renderer.update(state);

    // 5. Update HUD Telemetry
    updateHUD(state, inputs);
  } catch (err) {
    if (window.showError) window.showError(`[Frame Error] ${err.stack || err.message}`);
    console.error(err);
    return; // stop loop on error to avoid spamming
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
  for (let i = 0; i < 4; i++) {
    if (wheels[i]) {
      teleLoads[i].textContent = `${Math.round(wheels[i].Fz)} N`;
      const slipDeg = (wheels[i].slipAngle * (180 / Math.PI)).toFixed(1);
      teleSlips[i].textContent = `${slipDeg}°`;
    }
  }
}

init();
