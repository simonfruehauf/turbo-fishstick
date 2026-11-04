import {test} from 'node:test';
import assert from 'node:assert/strict';
import {laneDivider,laneBoundary,laneCenter,travelLaneCenter} from './src/lane-geometry.js';
import {pickLane} from './src/lane-sections.js';
import {Simulation,emptyWorld,demoWorld,route} from './src/model.js';

test('lane markings, vehicles and selection share physical lane geometry',()=>{
 for(const [forward,reverse] of [[2,2],[3,1],[1,3],[3,0],[0,3]]){
  const road={laneAtoB:forward,laneBtoA:reverse,oneway:!forward||!reverse};
  assert.equal(laneDivider(road),(reverse-forward)*4);
  for(const direction of [0,1])for(let lane=0;lane<(direction?reverse:forward);lane++){
   const center=laneCenter(road,direction,lane),inner=laneBoundary(road,direction,lane),outer=laneBoundary(road,direction,lane+1);
   assert.equal(center,(inner+outer)/2);
   assert.equal(travelLaneCenter(road,direction,lane),center*(direction?-1:1));
   const hit=pickLane(road,{x:50,y:center},{x:0,y:0},{x:100,y:0});
   assert.deepEqual([hit.direction,hit.index],[direction,lane]);
  }
 }
 const road={laneAtoB:2,laneBtoA:2};
 assert.deepEqual([laneBoundary(road,1,1),laneBoundary(road,0,1)],[-8,8]);
 assert.deepEqual([laneCenter(road,1,1),laneCenter(road,1,0),laneCenter(road,0,0),laneCenter(road,0,1)],[-12,-4,4,12]);
});
function fullWorld(withPort=false){
 const w=emptyWorld(12);w.terrain=[];w.roads=[{id:'road',a:{x:0,y:500},b:{x:1024,y:500},z:0,laneAtoB:1,laneBtoA:1}];
 w.buildings=[{id:'a',type:'home',x:200,y:465},{id:'b',type:'home',x:800,y:465}];w.ports=withPort?[{id:'exit',x:1024,y:500}]:[];w.demand=0;return w;
}
function fill(sim){sim.parked=[{destinationId:'a',parkingSlot:0,arrived:0,color:'#fff',length:10},{destinationId:'b',parkingSlot:0,arrived:0,color:'#fff',length:10}];sim.time=40}
function assertCapacity(sim){
 for(const b of sim.world.buildings){const slots=[...sim.parked.filter(p=>p.destinationId===b.id).map(p=>p.parkingSlot),...sim.parked.filter(p=>p.nextTrip?.destinationId===b.id).map(p=>p.nextTrip.parkingSlot),...sim.cars.filter(c=>c.destinationId===b.id).map(c=>c.parkingSlot)].filter(s=>s!==null&&s!==undefined);assert.equal(new Set(slots).size,slots.length,`duplicate reservation at ${b.id}`);assert.ok(slots.length<=sim.parkingCapacity(b))}
}
test('full parking lets parked vehicles leave and claim a freed destination without overbooking',()=>{
 const sim=new Simulation(fullWorld());fill(sim);
 const trip=sim.planDeparture(sim.parked[0]);assert.ok(trip?.requiresParking);assert.equal(trip.parkingSlot,null);
 let hadDeparture=false;for(let i=0;i<1800;i++){sim.tick(.1);assertCapacity(sim);hadDeparture ||= sim.cars.length>0}
 assert.ok(hadDeparture);assert.ok(sim.completed>=2,`only ${sim.completed} trips completed`);
});
test('a reachable port accepts a departing vehicle without a parking reservation',()=>{
 const sim=new Simulation(fullWorld(true));fill(sim);
 const trip=sim.planDeparture(sim.parked[0]);assert.equal(trip.destinationId,'exit');assert.equal(trip.parkingSlot,null);assert.ok(!trip.requiresParking);
});
test('traffic continues after the reported demo saturation period',()=>{
 const original=Math.random;let seed=77;Math.random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296};
 try{for(const id of ['ramp','bay','editor','stack']){const sim=new Simulation(demoWorld(id));let at200=0;for(let i=0;i<4000;i++){sim.tick(.1);if(i===1999)at200=sim.completed;if(i%10===0)assertCapacity(sim)}assert.ok(sim.completed>at200,`${id}: no trips completed after 200s (${at200} total)`);}}
 finally{Math.random=original}
});

test('unreserved arrivals wait until a real parking space is free',()=>{
 const sim=new Simulation(fullWorld());fill(sim);
 const start=sim.graph.attachments.find(a=>a.id==='a').node,end=sim.graph.attachments.find(a=>a.id==='b').node,path=route(sim.graph,start,end);
 const edge=path.at(-1);sim.cars=[{id:1,destinationId:'b',destinationNode:end,parkingSlot:null,requiresParking:true,edge,path:[edge],index:0,pos:edge.length-2,speed:0,lane:edge.lane??0,routeRevision:sim.ruleRevision,length:10,color:'#fff'}];
 sim.parked.forEach(p=>p.arrived=sim.time);for(let i=0;i<20;i++)sim.tick(.1);
 assert.equal(sim.completed,0);assert.equal(sim.cars.length,1);assert.equal(sim.cars[0].parkingSlot,null);assertCapacity(sim);
 sim.parked=sim.parked.filter(p=>p.destinationId!=='b');for(let i=0;i<100&&sim.cars.length;i++)sim.tick(.1);
 assert.equal(sim.cars.length,0);assert.equal(sim.completed,1);assertCapacity(sim);
});
test('a trip to an external port completes without occupying parking',()=>{
 const sim=new Simulation(fullWorld(true));sim.parked=[];
 const start=sim.graph.attachments.find(a=>a.id==='a').node,end=sim.graph.attachments.find(a=>a.id==='exit').node,path=route(sim.graph,start,end),edge=path[0];
 sim.cars=[{id:1,destinationId:'exit',destinationNode:end,parkingSlot:null,edge,path,index:0,pos:0,speed:0,lane:edge.lane??0,routeRevision:0,length:10,color:'#fff'}];
 for(let i=0;i<1500&&sim.cars.length;i++)sim.tick(.1);
 assert.equal(sim.cars.length,0);assert.equal(sim.completed,1);assert.equal(sim.parked.length,0);
});
