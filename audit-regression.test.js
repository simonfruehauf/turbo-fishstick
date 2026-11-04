import {test} from 'node:test';
import assert from 'node:assert/strict';
import {Simulation,emptyWorld,buildGraph,route,validateRoadPath} from './src/model.js';
import {applyLaneSection} from './src/lane-sections.js';
function blank(){let w=emptyWorld(11);w.terrain=[];w.roads=[];w.buildings=[];w.ports=[];w.demand=0;return w}
const road=(id,a,b,groupId=id)=>({id,groupId,a,b,z:0,lanes:1,laneAtoB:1,laneBtoA:1});
test('rejects overlapping road paths while allowing an endpoint connection',()=>{
 let w=blank();w.roads=[road('r',{x:10,y:50},{x:200,y:50})];
 assert.match(validateRoadPath(w,[{x:10,y:50},{x:200,y:50}],0,0).error,/already occupies/);
 assert.match(validateRoadPath(w,[{x:50,y:50},{x:250,y:50}],0,0).error,/already occupies/);
 assert.equal(validateRoadPath(w,[{x:200,y:50},{x:300,y:50}],0,0).error,null);
 assert.equal(validateRoadPath(w,[{x:10,y:50},{x:200,y:50}],1,1).error,null);
});
test('a road that crosses its own group is a real junction',()=>{
 let w=blank();w.roads=[road('a',{x:50,y:50},{x:150,y:150},'g'),road('b',{x:150,y:150},{x:50,y:150},'g'),road('c',{x:50,y:150},{x:150,y:50},'g')];
 let node=[...buildGraph(w).nodes.values()].find(n=>n.x===100&&n.y===100);assert.equal(node.degree,4);assert.equal(node.junction,true);
});
test('lane edits update legacy count along with directional lanes',()=>{
 let w=blank(),r=road('r',{x:0,y:50},{x:300,y:50});w.roads=[r];applyLaneSection(w,[{road:r,lo:0,hi:1}],3,1,()=>crypto.randomUUID());
 assert.equal(w.roads[0].lanes,3);assert.equal(w.roads[0].laneAtoB,3);
});
test('unreachable cars retry and eventually clear without keeping their reservation',()=>{
 let w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];
 w.buildings=[{id:'a',type:'home',x:10,y:65},{id:'b',type:'home',x:135,y:190}];
 let sim=new Simulation(w),from=sim.graph.attachments.find(a=>a.id==='a').node,to=sim.graph.attachments.find(a=>a.id==='b').node,path=route(sim.graph,from,to),edge=path[0],junction=[...sim.graph.nodes.values()].find(n=>n.junction);
 assert.ok(path.length>1);sim.cars=[{id:1,destinationId:'b',destinationNode:to,parkingSlot:0,edge,path,index:0,pos:1,speed:0,lane:edge.lane??0,routeRevision:0,length:10,color:'#fff'}];
 w.junctions[junction.id]={connectors:[]};sim.rulesChanged();sim.tick(.1);assert.equal(sim.cars[0].routeBlocked,true);
 for(let i=0;i<610;i++)sim.tick(.1);assert.equal(sim.cars.length,0);assert.equal(sim.availableSlot('b'),0);
});
test('unreachable cars recover if their route returns before timeout',()=>{
 let w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];
 w.buildings=[{id:'a',type:'home',x:10,y:65},{id:'b',type:'home',x:135,y:190}];let sim=new Simulation(w),from=sim.graph.attachments.find(a=>a.id==='a').node,to=sim.graph.attachments.find(a=>a.id==='b').node,path=route(sim.graph,from,to),edge=path[0],junction=[...sim.graph.nodes.values()].find(n=>n.junction);
 sim.cars=[{id:1,destinationId:'b',destinationNode:to,parkingSlot:0,edge,path,index:0,pos:1,speed:0,lane:edge.lane??0,routeRevision:0,length:10,color:'#fff'}];w.junctions[junction.id]={connectors:[]};sim.rulesChanged();sim.tick(.1);assert.equal(sim.cars[0].routeBlocked,true);delete w.junctions[junction.id].connectors;sim.rulesChanged();sim.tick(.1);assert.equal(sim.cars[0].routeBlocked,false);
});
test('new buildings and ground roads cannot overlap existing footprints',async()=>{
 const {buildingOverlapsWorld}=await import('./src/building-clearance.js');
 const w=blank();w.roads=[road('r',{x:0,y:100},{x:300,y:100})];w.buildings=[{id:'home',type:'home',x:100,y:150}];
 assert.equal(buildingOverlapsWorld(w,{type:'home',x:100,y:150}),true);
 assert.equal(buildingOverlapsWorld(w,{type:'home',x:200,y:100}),true);
 assert.equal(buildingOverlapsWorld(w,{type:'home',x:200,y:150}),false);
 assert.match(validateRoadPath(w,[{x:50,y:150},{x:150,y:150}],0,0,15).error,/building/i);
 assert.equal(validateRoadPath(w,[{x:50,y:150},{x:150,y:150}],1,1,15).error,null);
});
test('garden district industry buildings have ground-road access',async()=>{
 const {demoWorld}=await import('./src/model.js');const w=demoWorld('district'),graph=buildGraph(w);
 for(const building of w.buildings.filter(b=>b.type==='industry'))assert.ok(graph.attachments.some(a=>a.id===building.id));
});
test('closely spaced intersections retain the first lock until the rear clears',()=>{
 let w=blank();w.roads=[road('main',{x:0,y:100},{x:500,y:100}),road('x1',{x:200,y:0},{x:200,y:200}),road('x2',{x:220,y:0},{x:220,y:200})];w.ports=[{id:'start',x:0,y:100},{id:'end',x:500,y:100}];
 let sim=new Simulation(w),from=sim.graph.attachments.find(a=>a.id==='start').node,to=sim.graph.attachments.find(a=>a.id==='end').node,path=route(sim.graph,from,to),edge=path[0];
 sim.cars=[{id:1,destinationId:'end',destinationNode:to,parkingSlot:null,edge,path,index:0,pos:0,speed:0,lane:edge.lane??0,routeRevision:0,length:10,color:'#fff'}];
 let both=false;for(let i=0;i<1200&&sim.cars.length;i++){sim.tick(.05);let c=sim.cars[0];if(c?.priorJunctions?.length){both=true;for(const held of c.priorJunctions)assert.ok(sim.locks.get(held.id)?.has(c.id),'first junction lock released before clearance')}}
 assert.ok(both,'expected the car to hold two nearby junctions');assert.equal(sim.completed,1);
});
test('a vehicle stalled for two minutes is cleared with its junction lock',()=>{
 let w=blank();w.roads=[road('main',{x:0,y:100},{x:500,y:100}),road('cross',{x:200,y:0},{x:200,y:200})];w.ports=[{id:'start',x:0,y:100},{id:'end',x:500,y:100}];
 const sim=new Simulation(w),start=sim.graph.attachments.find(a=>a.id==='start').node,end=sim.graph.attachments.find(a=>a.id==='end').node,path=route(sim.graph,start,end),edge=path[0],junction=[...sim.graph.nodes.values()].find(n=>n.junction);
 sim.time=121;sim.cars=[{id:1,destinationId:'end',destinationNode:end,parkingSlot:null,edge,path,index:0,pos:0,speed:0,lane:edge.lane??0,routeRevision:0,length:10,color:'#fff',progressAt:0,progressDistance:0,junctionCommit:junction.id}];sim.locks.set(junction.id,new Map([[1,{approach:edge.id,exit:path[1].id,lane:0}]]));
 sim.tick(.01);sim.tick(.01);assert.equal(sim.cars.length,0);assert.equal(sim.locks.has(junction.id),false);
});

test('stop signs hold a vehicle at the junction before letting it proceed',()=>{
 const w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),edge=j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),next=j.out.find(e=>sim.graph.nodes.get(e.to).x===200),path=[edge,next];
 w.junctions[j.id]={mode:'priority',approaches:{[edge.id]:'stop'}};
 sim.cars=[{id:1,edge,path,index:0,pos:edge.length-23,speed:0,lane:0,routeRevision:0,length:10}];
 sim.tick(.1);assert.equal(sim.cars[0].speed,0);assert.equal(sim.cars[0].junctionCommit,undefined);
 for(let i=0;i<12;i++)sim.tick(.1);
 assert.equal(sim.cars[0].junctionCommit,j.id);assert.ok(sim.cars[0].speed>0);
});

test('a stopped yielding vehicle releases its reservation for a roundabout exit',()=>{
 const w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];w.roads[0].roundabout=true;
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),yieldEdge=j.incoming.find(e=>sim.graph.nodes.get(e.from).y===0),priorityEdge=j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),exit=j.out.find(e=>sim.graph.nodes.get(e.to).y===200),yieldExit=exit;
 w.junctions[j.id]={mode:'priority',approaches:{[yieldEdge.id]:'yield',[priorityEdge.id]:'priority'}};
 const waiting={id:1,edge:yieldEdge,path:[yieldEdge,yieldExit],index:0,pos:yieldEdge.length-23,speed:0,lane:0,routeRevision:0,length:10,junctionCommit:j.id};
 const turning={id:2,edge:priorityEdge,path:[priorityEdge,exit],index:0,pos:priorityEdge.length-23,speed:0,lane:0,routeRevision:0,length:10};
 sim.cars=[waiting,turning];sim.locks.set(j.id,new Map([[1,{approach:yieldEdge.id,exit:yieldExit.id,lane:0}]]));
 sim.tick(.1);assert.equal(waiting.junctionCommit,null);assert.equal(waiting.speed,0);assert.equal(turning.junctionCommit,j.id);assert.ok(turning.speed>0);
});


test('nonconflicting right turns can proceed together despite a yield sign',()=>{
 const w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),west=j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),north=j.incoming.find(e=>sim.graph.nodes.get(e.from).y===0),south=j.out.find(e=>sim.graph.nodes.get(e.to).y===200),east=j.out.find(e=>sim.graph.nodes.get(e.to).x===200);
 w.junctions[j.id]={mode:'priority',approaches:{[north.id]:'yield',[west.id]:'priority'}};
 const a={id:1,edge:west,path:[west,south],index:0,pos:west.length-23,speed:0,lane:0,routeRevision:0,length:10},b={id:2,edge:north,path:[north,east],index:0,pos:north.length-23,speed:0,lane:0,routeRevision:0,length:10};
 sim.cars=[a,b];sim.tick(.1);assert.equal(a.junctionCommit,j.id);assert.equal(b.junctionCommit,j.id);assert.ok(a.speed>0&&b.speed>0);
});

test('a right-turning car advances toward available space behind a queued vehicle',()=>{
 const w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),west=j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),south=j.out.find(e=>sim.graph.nodes.get(e.to).y===200);
 const turning={id:1,edge:west,path:[west,south],index:0,pos:west.length-23,speed:0,lane:0,routeRevision:0,length:10},queue={id:2,edge:south,path:[south],index:0,pos:29,speed:0,lane:0,routeRevision:0,length:10,routeBlocked:true,blockedSince:0,retryRouteAt:Infinity};
 sim.cars=[turning,queue];for(let i=0;i<20;i++)sim.tick(.1);
 const progress=turning.edge.id===west.id?turning.pos:west.length+turning.pos;assert.ok(progress>west.length-23,'turning vehicle should advance');assert.ok(progress<=west.length+queue.pos-17+.1,'turning vehicle must keep a safe gap');
});

test('an empty unsigned intersection does not require a stop',()=>{
 const w=blank();w.roads=[road('h',{x:0,y:100},{x:200,y:100}),road('v',{x:100,y:0},{x:100,y:200})];
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),west=j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),east=j.out.find(e=>sim.graph.nodes.get(e.to).x===200);
 const car={id:1,edge:west,path:[west,east],index:0,pos:west.length-30,speed:20,lane:0,routeRevision:0,length:10};sim.cars=[car];
 for(let i=0;i<10;i++){sim.tick(.1);assert.ok(car.speed>0,'car stopped without a reason')}
});

test('a roundabout entry yields to circulating traffic on earlier short segments',()=>{
 const w=blank();w.roads=[road('r1',{x:50,y:100},{x:80,y:100},'ring'),road('r2',{x:80,y:100},{x:90,y:100},'ring'),road('r3',{x:90,y:100},{x:100,y:100},'ring'),road('r4',{x:100,y:100},{x:170,y:100},'ring'),road('entry',{x:100,y:200},{x:100,y:100})];
 for(const r of w.roads.filter(r=>r.groupId==='ring')){r.roundabout=true;r.oneway=true;r.laneBtoA=0}
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.x===100&&n.y===100),entry=j.incoming.find(e=>!e.roundabout),exit=j.out.find(e=>e.roundabout),ring=sim.graph.edges.find(e=>e.road==='r1'),r2=sim.graph.edges.find(e=>e.road==='r2'),r3=sim.graph.edges.find(e=>e.road==='r3');
 w.junctions[j.id]={mode:'priority',approaches:{[entry.id]:'yield'}};
 const joining={id:1,edge:entry,path:[entry,exit],index:0,pos:entry.length-23,speed:0,lane:0,routeRevision:0,length:10},circulating={id:2,edge:ring,path:[ring,r2,r3,exit],index:0,pos:ring.length-5,speed:10,lane:0,routeRevision:0,length:10};
 sim.cars=[joining,circulating];sim.tick(.1);
 assert.equal(joining.speed,0);assert.equal(joining.junctionCommit,undefined);assert.equal(joining.waitReason,'Waiting for roundabout traffic');assert.ok(circulating.speed>0);
});

test('a car leaving a roundabout advances into free exit space behind a queue',()=>{
 const w=blank();w.roads=[road('ring',{x:0,y:100},{x:100,y:100}),road('exit',{x:100,y:100},{x:200,y:100}),road('entry',{x:100,y:200},{x:100,y:100})];w.roads[0].roundabout=true;w.roads[0].laneBtoA=0;w.roads[0].oneway=true;
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),ring=j.incoming.find(e=>e.roundabout),exit=j.out.find(e=>e.road==='exit');
 const leaving={id:1,edge:ring,path:[ring,exit],index:0,pos:ring.length-23,speed:0,lane:0,routeRevision:0,length:10},queue={id:2,edge:exit,path:[exit],index:0,pos:29,speed:0,lane:0,routeRevision:0,length:10,routeBlocked:true,blockedSince:0,retryRouteAt:Infinity};
 sim.cars=[leaving,queue];for(let i=0;i<20;i++)sim.tick(.1);
 const progress=leaving.edge.id===ring.id?leaving.pos:ring.length+leaving.pos;assert.ok(progress>ring.length-23);assert.ok(progress<=ring.length+queue.pos-17+.1);
});

test('exit space does not include traffic beyond the next intersection',()=>{
 const w=blank();w.roads=[road('ring',{x:0,y:100},{x:100,y:100}),road('exit',{x:100,y:100},{x:130,y:100}),road('after',{x:130,y:100},{x:200,y:100}),road('entry',{x:100,y:200},{x:100,y:100}),road('cross',{x:130,y:0},{x:130,y:200})];w.roads[0].roundabout=true;w.roads[0].laneBtoA=0;w.roads[0].oneway=true;
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.x===100&&n.y===100),ring=j.incoming.find(e=>e.roundabout),exit=j.out.find(e=>e.road==='exit'),after=sim.graph.edges.find(e=>e.road==='after'&&e.from===exit.to);
 const leaving={id:1,edge:ring,path:[ring,exit,after],index:0,pos:ring.length-23,speed:0,lane:0,routeRevision:0,length:10},other={id:2,edge:after,path:[after],index:0,pos:5,speed:0,lane:0,routeRevision:0,length:10,routeBlocked:true,blockedSince:0,retryRouteAt:Infinity};
 sim.cars=[leaving,other];sim.tick(.1);assert.ok(leaving.speed>0);assert.notEqual(leaving.waitReason,'Exit is blocked');
});

test('a main-road vehicle respects a conflicting merge reservation',()=>{
 const w=blank();w.roads=[road('main',{x:0,y:100},{x:200,y:100}),road('ramp',{x:100,y:200},{x:100,y:100})];
 const sim=new Simulation(w),j=[...sim.graph.nodes.values()].find(n=>n.junction),main=j.incoming.find(e=>sim.graph.nodes.get(e.from).x===0),ramp=j.incoming.find(e=>e.road==='ramp'),exit=j.out.find(e=>sim.graph.nodes.get(e.to).x===200);
 j.mergeGroups.add('main');
 const car={id:1,edge:main,path:[main,exit],index:0,pos:main.length-23,speed:0,lane:0,routeRevision:0,length:10};
 const merging={id:2,edge:ramp,path:[ramp,exit],index:0,pos:ramp.length-23,speed:0,lane:0,routeRevision:0,length:10,junctionCommit:j.id,junctionEntered:true,junctionClearance:20};
 sim.cars=[car,merging];sim.locks.set(j.id,new Map([[2,{approach:ramp.id,exit:exit.id,inLane:0,lane:0}]]));sim.tick(.1);
 assert.equal(car.speed,0);assert.equal(car.waitReason,'Waiting for crossing traffic');
});

test('departures do not enter a lane reserved by a changing vehicle',()=>{
 const w=blank();w.roads=[road('r',{x:0,y:100},{x:200,y:100})];w.roads[0].laneAtoB=2;
 const sim=new Simulation(w),edge=sim.graph.edges.find(e=>e.direction===0),parked={length:10,color:'#fff',nextTrip:{path:[{...edge,lane:1}],destinationId:'end',destinationNode:edge.to,parkingSlot:null}};
 sim.cars=[{id:1,edge,pos:5,lane:0,laneChange:{from:0,to:1,distance:0,length:32}}];
 assert.equal(sim.depart(parked),false);
});
