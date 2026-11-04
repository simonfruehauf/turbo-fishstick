import {test} from 'node:test';
import assert from 'node:assert/strict';
import {parkingSlots,pointInPolygon,validParkingPolygon} from './src/parking-layout.js';
import {buildingOverlapsWorld} from './src/building-clearance.js';
import {emptyWorld,Simulation} from './src/model.js';
const rect=(x,y,w,h)=>[{x,y},{x:x+w,y},{x:x+w,y:y+h},{x,y:y+h}];
test('outlined parking generates distinct car-sized spaces inside the area',()=>{
 const polygon=rect(50,50,100,100),lot={type:'parking',x:100,y:100,polygon},slots=parkingSlots(lot);
 assert.ok(slots.length>10);assert.ok(slots.every(s=>s.length===18&&s.width===8&&pointInPolygon(s,polygon)));
 for(let i=0;i<slots.length;i++)for(let j=i+1;j<slots.length;j++)assert.ok(Math.hypot(slots[i].x-slots[j].x,slots[i].y-slots[j].y)>=9);
});
test('invalid crossing outlines are rejected and building collisions use the polygon',()=>{
 assert.equal(validParkingPolygon([{x:0,y:0},{x:100,y:100},{x:0,y:100},{x:100,y:0}]),false);
 const w=emptyWorld(1);w.roads=[];w.buildings=[{id:'home',type:'home',x:60,y:60}];
 assert.equal(buildingOverlapsWorld(w,{type:'parking',x:80,y:80,polygon:rect(50,50,60,60)}),true);
});
test('capacity and road attachment follow an outlined parking area',()=>{
 const w=emptyWorld(1),polygon=rect(20,30,80,80),b={id:'lot',type:'parking',x:60,y:70,polygon,accessPoint:{x:60,y:30}};
 w.roads=[{id:'road',a:{x:0,y:0},b:{x:150,y:0},z:0,laneAtoB:1,laneBtoA:1}];w.buildings=[b];w.ports=[];
 const sim=new Simulation(w);assert.equal(sim.parkingCapacity(b),parkingSlots(b).length);assert.ok(sim.graph.attachments.some(a=>a.id==='lot'));assert.equal(sim.availableSlot('lot'),0);
});

test('parking bays open onto an uninterrupted entrance aisle',()=>{
 const polygon=rect(50,50,100,100),lot={type:'parking',x:100,y:100,polygon,accessPoint:{x:50,y:100},accessDirection:{x:1,y:0}},slots=parkingSlots(lot);
 assert.ok(slots.length>5);for(const slot of slots){assert.equal(Math.abs(slot.y-100),17);for(let x=51;x<=slot.x;x+=3)for(const y of [93,100,107])assert.ok(pointInPolygon({x,y},polygon))}
});
test('a disconnected outlined lot is not attached to the traffic graph',()=>{
 const w=emptyWorld(1);w.roads=[{id:'road',a:{x:0,y:0},b:{x:150,y:0},z:0,laneAtoB:1,laneBtoA:1}];w.buildings=[{id:'lot',type:'parking',x:60,y:70,polygon:rect(20,30,80,80),accessPoint:{x:60,y:30},parkingDisconnected:true}];w.ports=[];
 assert.equal(new Simulation(w).graph.attachments.some(a=>a.id==='lot'),false);
});

test('large lots fill multiple aisles while retaining an entrance route',()=>{
 const polygon=rect(0,0,180,180),lot={type:'parking',x:90,y:90,polygon,accessPoint:{x:0,y:90},accessDirection:{x:1,y:0}},slots=parkingSlots(lot);
 assert.ok(slots.length>30);assert.ok(slots.some(s=>s.aisleOffset!==0));assert.ok(slots.every(s=>s.aisleOffset===0||s.aisleDistance>=23));
});

test('homes use front or side parking and fall back to an attached nearby lot',async()=>{
 const {findHomeParking}=await import('./src/home-parking.js');const road={id:'r',a:{x:0,y:100},b:{x:300,y:100},z:0,laneAtoB:1,laneBtoA:1},w=emptyWorld(1);w.roads=[road];w.ports=[];
 const front={id:'front',type:'home',x:100,y:60};w.buildings=[front];assert.equal(findHomeParking(w,front)?.kind,'front');
 const side={id:'side',type:'home',x:150,y:75};w.buildings=[side];assert.equal(findHomeParking(w,side)?.kind,'side');
 const left={id:'left',type:'home',x:125,y:75},right={id:'right',type:'home',x:175,y:75},lot={id:'lot',type:'parking',x:240,y:45,polygon:rect(205,10,70,70),accessPoint:{x:240,y:80},accessDirection:{x:0,y:-1},accessRoadId:'r'};w.buildings=[side,left,right,lot];
 assert.equal(findHomeParking(w,side),null);const sim=new Simulation(w),home=sim.graph.attachments.find(a=>a.id==='side'),lotNode=sim.graph.attachments.find(a=>a.id==='lot');assert.equal(home.parkingId,'lot');assert.equal(home.node,lotNode.node);assert.equal(sim.availableSlot('side'),-1);assert.equal(sim.availableSlot('lot'),0);
});
