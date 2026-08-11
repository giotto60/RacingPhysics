import { readFileSync } from 'fs';
import { PhysicsWorld } from './physics/PhysicsWorld.js';

try {
  console.log("Loading car spec...");
  const carSpec = JSON.parse(readFileSync('./data/cars/default.json', 'utf8'));
  console.log("Initializing PhysicsWorld...");
  const world = new PhysicsWorld(carSpec);
  console.log("Stepping physics...");
  for (let i = 0; i < 100; i++) {
    world.update(1/60, { throttle: 1, brake: 0, steer: 0, reset: false });
  }
  const state = world.getInterpolatedVehicleState();
  console.log("Success! Car speed after 100 steps:", state.speedKmH, "km/h");
} catch (err) {
  console.error("CRASH ERROR:", err);
}
