import {test} from 'node:test';
import assert from 'node:assert/strict';
import {Simulation,emptyWorld,route,defaultMovement,movementKey} from './src/model.js';
import {signalState,trafficSnapshot} from './src/traffic-rules.js';
import {parkingAnimation,vehicleRoutePoints,departureLanePoint} from './src/vehicle-route.js';
import {intersectionMarkers,spacedNodeHandles} from './src/intersection-markers.js';

const road=(id,a,b,groupId=id,lanes=1)=>({id,groupId,a,b,z:0,laneAtoB:lanes,laneBtoA:lanes});
const blank=()=>({...emptyWorld(1),terrain:[],demand:0});
function crossing(lanes=1){
 const w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100},'h',lanes),road('v',{x:100,y:0},{x:100,y:200},'v',lanes)];
 const sim=new Simulation(w),j=sim.graph.junctionNodes[0];
 return {w,sim,j,west:j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),east:j.out.find(e=>sim.graph.nodes.get(e.to).x===200),south:j.out.find(e=>sim.graph.nodes.get(e.to).y===200),north:j.incoming.find(e=>sim.graph.nodes.get(e.from).y===0),fromEast:j.incoming.find(e=>sim.graph.nodes.get(e.from).x===200)};
}
function driver(id,edge,next,extra={}){return {id,edge,path:next?[edge,next]:[edge],index:0,pos:Math.max(0,edge.length-23),speed:0,lane:edge.lane??0,routeRevision:0,length:10,...extra}}
function advance(sim,seconds){for(let t=0;t<seconds-1e-9;t+=.05)sim.tick(.05)}

test('signals have amber and all-red clearance, and never show opposing green',()=>{
 assert.equal(signalState(11.99,12,true),'green');assert.equal(signalState(12,12,true),'amber');
 assert.equal(signalState(14,12,true),'red');assert.equal(signalState(14,12,false),'red');assert.equal(signalState(15,12,false),'green');
 for(let t=0;t<100;t+=.05)assert.ok(!(signalState(t,7,true,3)==='green'&&signalState(t,7,false,3)==='green'));
});
test('right on red is disabled by default',()=>{
 const {w,sim,j,west,south}=crossing();w.junctions[j.id]={mode:'signal'};sim.time=16;const car=driver(1,west,south);sim.cars=[car];advance(sim,2);
 assert.equal(car.speed,0);assert.equal(car.junctionCommit,undefined);
});
test('right on red requires a full stop before an outside-lane right turn',()=>{
 const {w,sim,j,west,south}=crossing();w.junctions[j.id]={mode:'signal',rightOnRed:true};sim.time=16;const car=driver(1,west,south);sim.cars=[car];advance(sim,.9);
 assert.equal(car.speed,0);assert.equal(car.junctionCommit,undefined);advance(sim,.3);assert.equal(car.junctionCommit,j.id);assert.ok(car.speed>0);
});
test('right on red gives way to conflicting green traffic',()=>{
 const {w,sim,j,west,south,north}=crossing();w.junctions[j.id]={mode:'signal',rightOnRed:true};sim.time=16;
 const car=driver(1,west,south),green=driver(2,north,south,{routeBlocked:true,blockedSince:16,retryRouteAt:Infinity});sim.cars=[car,green];advance(sim,2);
 assert.equal(car.speed,0);assert.equal(car.junctionCommit,undefined);
});
test('right on red is restricted to the outside lane',()=>{
 const {w,sim,j,west,south}=crossing(3);w.junctions[j.id]={mode:'signal',rightOnRed:true};sim.time=16;
 const car=driver(1,west,south,{lane:0});sim.cars=[car];advance(sim,1.5);assert.equal(car.speed,0);assert.equal(car.junctionCommit,undefined);
});
test('right on red holds at the line when the destination lane is full',()=>{
 const {w,sim,j,west,south}=crossing();w.junctions[j.id]={mode:'signal',rightOnRed:true};sim.time=16;
 const car=driver(1,west,south),queue=driver(2,south,null,{pos:10,routeBlocked:true,blockedSince:16,retryRouteAt:Infinity});sim.cars=[car,queue];advance(sim,1.5);
 assert.equal(car.speed,0);assert.equal(car.junctionCommit,undefined);
});
test('right on red never permits straight travel, amber, or all-red clearance',()=>{
 for(const [time,straight] of [[16,true],[12,false],[14.1,false]]){
  const {w,sim,j,west,south,east}=crossing();w.junctions[j.id]={mode:'signal',rightOnRed:true};sim.time=time;
  const car=driver(1,west,straight?east:south);sim.cars=[car];advance(sim,.5);assert.equal(car.speed,0);assert.equal(car.junctionCommit,undefined);
 }
});
test('a green light keeps a moving platoon flowing through an occupied but clearing exit',()=>{
 const {w,sim,j,west,east}=crossing();w.junctions[j.id]={mode:'signal'};
 const follower=driver(1,west,east,{pos:west.length-35,speed:30});
 const leader=driver(2,east,null,{path:[west,east],index:1,pos:5,speed:30,junctionCommit:j.id,junctionEntered:true,junctionClearance:23});
 sim.cars=[follower,leader];sim.locks.set(j.id,new Map([[2,{approach:west.id,exit:east.id,inLane:0,lane:0}]]));
 for(let i=0;i<35;i++){sim.tick(.05);assert.ok(follower.speed>15,`unnecessary stop: ${follower.waitReason}`)}
 assert.ok(follower.index>0);
});
test('a green light still holds a straight movement when its exit queue cannot clear',()=>{
 const {w,sim,j,west,east}=crossing();w.junctions[j.id]={mode:'signal'};
 const follower=driver(1,west,east),queue=driver(2,east,null,{pos:10,routeBlocked:true,blockedSince:0,retryRouteAt:Infinity});
 sim.cars=[follower,queue];advance(sim,1);assert.equal(follower.speed,0);assert.equal(follower.waitReason,'Exit is blocked');
});
test('left turns on green yield to oncoming straight traffic',()=>{
 const {w,sim,j,west,north,fromEast}=crossing();w.junctions[j.id]={mode:'signal'};
 const left=driver(1,west,j.out.find(e=>e.to===north.from)),straight=driver(2,fromEast,j.out.find(e=>e.to===west.from));
 sim.cars=[left,straight];sim.tick(.05);assert.equal(left.speed,0);assert.ok(straight.speed>0);
});
test('default lane links preserve order, with inside left turns and outside right turns',()=>{
 const {sim,west,east,south,north,j}=crossing(3),toNorth=j.out.find(e=>e.to===north.from);
 assert.ok(defaultMovement(sim.graph,west,1,east,1));assert.ok(!defaultMovement(sim.graph,west,1,east,0));
 assert.ok(defaultMovement(sim.graph,west,2,south,2));assert.ok(!defaultMovement(sim.graph,west,0,south,0));
 assert.ok(defaultMovement(sim.graph,west,0,toNorth,0));assert.ok(!defaultMovement(sim.graph,west,2,toNorth,2));
});
test('unsigned traffic gives way to the right without stopping an empty intersection',()=>{
 const {sim,west,east,north,south}=crossing();
 const eastbound=driver(1,west,east);
 const southern=sim.graph.nodes.get(south.to).out.find(e=>e.to===west.to),toNorth=sim.graph.nodes.get(west.to).out.find(e=>e.to===north.from);
 const fromRight=driver(2,southern,toNorth);sim.cars=[eastbound,fromRight];sim.tick(.05);
 assert.equal(eastbound.speed,0);assert.ok(fromRight.speed>0);
});
test('a left turn yields to oncoming straight traffic even when updated first',()=>{
 const {sim,j,west,north,fromEast}=crossing(),toNorth=j.out.find(e=>e.to===north.from),toWest=j.out.find(e=>e.to===west.from);
 const left=driver(1,west,toNorth),straight=driver(2,fromEast,toWest);sim.cars=[left,straight];sim.tick(.05);
 assert.equal(left.speed,0);assert.ok(straight.speed>0);
});
test('a circular wait resolves by arrival order, ignoring followers behind the stop line',()=>{
 const {sim,j}=crossing();
 sim.time=10;sim.cars=j.incoming.map((edge,i)=>driver(i+1,edge,j.out.find(e=>e.to!==edge.from&&e.hx*edge.hx+e.hy*edge.hy>.9),{junctionWaitingSince:5}));
 const front=sim.cars[0];sim.cars.push({...front,id:10,pos:front.pos-17,junctionWaitingSince:undefined});
 const index=trafficSnapshot(sim.graph,sim.cars);assert.equal(index.approaches.get(j.id).length,4);
 advance(sim,8);assert.ok(sim.completed>0||sim.cars.some(c=>c.junctionEntered),'intersection stayed in circular wait');
});

function straightSimulation(pieces){
 const w=blank();w.roads=Array.from({length:pieces},(_,i)=>road('r'+i,{x:i*400/pieces,y:300},{x:(i+1)*400/pieces,y:300},'run'));
 const sim=new Simulation(w),path=route(sim.graph,'0,300@0','400,300@0');sim.cars=[driver(1,path[0],null,{path,pos:0,speed:48})];return sim;
}
const xOf=(sim,car)=>sim.graph.nodes.get(car.edge.from).x+car.pos;
test('movement is independent of how many short segments make up a road',()=>{
 const long=straightSimulation(1),short=straightSimulation(200);advance(long,2);advance(short,2);
 assert.ok(Math.abs(xOf(long,long.cars[0])-xOf(short,short.cars[0]))<1e-6);
 assert.ok(short.cars[0].pos>=0&&short.cars[0].pos<=short.cars[0].edge.length);
});
test('large time steps use the same fixed updates as normal playback',()=>{
 const a=straightSimulation(100),b=straightSimulation(100);a.tick(.4);advance(b,.4);
 assert.ok(Math.abs(xOf(a,a.cars[0])-xOf(b,b.cars[0]))<1e-6);
});
test('car following uses a stable snapshot across an edge boundary',()=>{
 for(const reverse of [false,true]){
  const sim=straightSimulation(20),path=sim.cars[0].path;
  const follower=driver(reverse?2:1,path[0],null,{path,pos:8,speed:20}),leader=driver(reverse?1:2,path[1],null,{path,index:1,pos:5,routeBlocked:true,blockedSince:0,retryRouteAt:Infinity});
  sim.cars=[follower,leader];advance(sim,2);assert.ok(xOf(sim,leader)-xOf(sim,follower)>=15-1e-6);
 }
});
test('a departure checks following traffic across a short predecessor segment',()=>{
 const sim=straightSimulation(20),path=sim.cars[0].path,car=sim.cars[0];car.pos=19;car.speed=35;
 assert.equal(sim.entryIsClear(path.slice(1)),false);
});
test('waiting for parking does not prevent crossing intermediate edges',()=>{
 const sim=straightSimulation(20),car=sim.cars[0];car.requiresParking=true;car.parkingSlot=null;car.destinationId='missing';advance(sim,1);
 assert.ok(car.index>0);assert.ok(car.pos<=car.edge.length);
});
test('parking alternatives reserve distinct real spaces without moving the vehicles',()=>{
 const w=blank();w.roads=[road('r',{x:0,y:300},{x:400,y:300})];w.ports=[{id:'start',x:0,y:300}];
 w.buildings=[{id:'full',type:'shop',x:200,y:270},{id:'free',type:'shop',x:350,y:270}];
 const sim=new Simulation(w),path=route(sim.graph,'0,300@0','200,300@0');
 sim.parked=Array.from({length:4},(_,parkingSlot)=>({destinationId:'full',parkingSlot,arrived:0}));
 const a=driver(1,path[0],null,{path,pos:80,destinationId:'full',parkingSlot:null,requiresParking:true}),b={...a,id:2,pos:100};sim.cars=[a,b];
 assert.ok(sim.findParkingAlternative(a));assert.ok(sim.findParkingAlternative(b));
 assert.equal(a.destinationId,'free');assert.equal(b.destinationId,'free');assert.notEqual(a.parkingSlot,b.parkingSlot);assert.equal(a.pos,80);assert.equal(b.pos,100);assert.equal(a.edge.id,path[0].id);
});
test('one full parking destination does not suppress unrelated traffic between map ports',()=>{
 const w=blank();w.roads=[road('r',{x:0,y:300},{x:400,y:300})];w.ports=[{id:'left',x:0,y:300},{id:'right',x:400,y:300}];w.buildings=[{id:'full',type:'shop',x:200,y:270}];
 const sim=new Simulation(w),path=route(sim.graph,'0,300@0','200,300@0');sim.cars=[driver(1,path[0],null,{path,pos:190,destinationId:'full',parkingSlot:null,requiresParking:true})];sim.serial=1;
 const random=Math.random;let i=0;Math.random=()=>[.9,.4][i++%2];
 try{sim.spawn()}finally{Math.random=random}
 assert.equal(sim.cars.length,2);assert.equal(sim.cars[1].destinationId,'left');
});
test('a space freed by a departure goes to an existing waiting car before a new trip',()=>{
 const w=blank();w.demand=1;w.roads=[road('r',{x:0,y:300},{x:400,y:300})];w.ports=[{id:'left',x:0,y:300},{id:'right',x:400,y:300}];w.buildings=[{id:'home',type:'home',x:200,y:265}];
 const sim=new Simulation(w),home=sim.graph.attachments.find(a=>a.id==='home').node,path=route(sim.graph,'0,300@0',home),departure=route(sim.graph,home,'0,300@0');
 const waiting=driver(1,path[0],null,{path,pos:path[0].length-30,destinationId:'home',parkingSlot:null,requiresParking:true});sim.cars=[waiting];sim.serial=1;sim.time=40;sim.timer=.31;
 sim.parked=[{destinationId:'home',parkingSlot:0,arrived:0,departStarted:37,nextTrip:{path:departure,destinationId:'left',destinationNode:'0,300@0',parkingSlot:null},tripRevision:0,length:10}];
 const random=Math.random;let i=0;Math.random=()=>[.4,.1][i++%2];try{sim.tick(.05)}finally{Math.random=random}
 assert.equal(sim.parked.length,0);assert.equal(waiting.parkingSlot,0);assert.equal(sim.cars.filter(c=>c.destinationId==='home'&&c.parkingSlot===0).length,1);
});
test('editing intersection rules preserves parked positions and departure animation',()=>{
 const {sim,west,east}=crossing(),trip={path:[west,east],destinationId:'outside',parkingSlot:null};
 const parked={id:5,destinationId:'home',parkingSlot:0,start:{x:3,y:7},arrived:0,departStarted:38.5,nextTrip:trip};sim.time=40;sim.parked=[parked];
 const before=parkingAnimation(parked,sim.time);for(let i=0;i<5;i++)sim.rulesChanged();
 assert.deepEqual(parkingAnimation(parked,sim.time),before);assert.equal(parked.nextTrip,trip);assert.equal(parked.departStarted,38.5);assert.deepEqual(parked.start,{x:3,y:7});
 const anchor=departureLanePoint(sim.graph,trip);assert.ok(Math.abs(anchor.x)<.001&&Math.abs(anchor.y-104)<.001);
});
test('unavailable departure routes freeze parking animation instead of replaying it',()=>{
 const car={arrived:0,departStarted:38,nextTrip:{},departurePausedAt:39};
 assert.deepEqual(parkingAnimation(car,40),parkingAnimation(car,60));
});
test('the inspected path starts at the current position and ends at the planned destination',()=>{
 const sim=straightSimulation(20),car=sim.cars[0];car.pos=12;const points=vehicleRoutePoints(sim.graph,car);
 assert.equal(points[0].x,12);assert.equal(points.at(-1).x,400);assert.ok(points.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)));
});
test('a planned lane change remains continuous over many short road pieces',()=>{
 const w=blank();w.roads=Array.from({length:20},(_,i)=>road('r'+i,{x:i*20,y:300},{x:(i+1)*20,y:300},'main',2));
 w.roads.push(road('after',{x:400,y:300},{x:600,y:300},'main',2),{...road('exit',{x:400,y:300},{x:400,y:500}),oneway:true,laneBtoA:0});
 const sim=new Simulation(w),path=route(sim.graph,'0,300@0','400,500@0',{},0);
 assert.ok(path);assert.equal(path.find(e=>e.road==='exit').lane,0);
 const car=driver(1,path[0],null,{path,pos:0,lane:0}),points=vehicleRoutePoints(sim.graph,car).filter(p=>p.x<350);
 for(let i=1;i<points.length;i++)assert.ok(Math.abs(points[i].y-points[i-1].y)<5,'lane preview jumps between consecutive road pieces');
});
test('lane connection overrides still route through an explicitly chosen lane',()=>{
 const {sim,w,j,west,south}=crossing(2);w.junctions[j.id]={connectors:[movementKey(west,0,south,1)]};
 const path=route(sim.graph,west.from,south.to,w.junctions,0,west.id);assert.ok(path);assert.equal(path[0].exitLane,0);assert.equal(path[1].lane,1);
});
test('junction lane handles stay separated at different zoom levels',()=>{
 const {sim,j}=crossing(3);
 for(const zoom of [.4,1,2.5]){const markers=intersectionMarkers(sim.graph,j.id,zoom);assert.equal(markers.length,24);
  for(let i=0;i<markers.length;i++)for(let k=i+1;k<markers.length;k++)assert.ok(Math.hypot(markers[i].x-markers[k].x,markers[i].y-markers[k].y)*zoom>=16-1e-6);
 }
});
test('move-node handles thin dense curve points while retaining junctions',()=>{
 const nodes=Array.from({length:40},(_,i)=>({id:i,x:i*2,y:100,z:0,degree:2}));nodes[10].junction=true;
 const shown=spacedNodeHandles(nodes,1);assert.ok(shown.length<10);assert.ok(shown.includes(nodes[10]));
 for(let i=0;i<shown.length;i++)for(let k=i+1;k<shown.length;k++)assert.ok(Math.hypot(shown[i].x-shown[k].x,shown[i].y-shown[k].y)>=14);
});
