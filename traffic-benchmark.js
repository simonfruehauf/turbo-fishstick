import {performance} from 'node:perf_hooks';
import {Simulation,demoWorld} from './src/model.js';

// Reproducible simulation work, excluding graph construction and rendering.
const originalRandom=Math.random,rows=[];
try {
 for(const id of ['starter','bay','clover','magic','district','lane-lab']){
  let seed=77;Math.random=()=>((seed=(Math.imul(seed,1664525)+1013904223)>>>0)/4294967296);
  const sim=new Simulation(demoWorld(id)),samples=[];
  let vehicleSteps=0,waitingSteps=0;
  for(let i=0;i<6000;i++){
   const start=performance.now();sim.tick(.05);samples.push(performance.now()-start);
   vehicleSteps+=sim.cars.length;waitingSteps+=sim.cars.filter(c=>c.speed<3).length;
  }
  samples.sort((a,b)=>a-b);
  rows.push({map:id,trips:sim.completed,cleared:sim.cleared??'untracked',averageCars:Math.round(vehicleSteps/6000),waitingPercent:Math.round(100*waitingSteps/Math.max(1,vehicleSteps)),tickMeanMs:+(samples.reduce((a,b)=>a+b,0)/samples.length).toFixed(3),tickP95Ms:+samples[Math.floor(samples.length*.95)].toFixed(3)});
 }
} finally {Math.random=originalRandom}
console.table(rows);
