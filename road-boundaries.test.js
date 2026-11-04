import {test} from 'node:test';
import assert from 'node:assert/strict';
import {nearbyRoadBoundaries} from './src/road-boundaries.js';
import {junctionWidthFlares,flareHalfWidth} from './src/junction-width.js';
import {buildGraph,emptyWorld} from './src/model.js';
const road=(id,a,b,z=0)=>({id,groupId:id,a,b,z,zEnd:z,laneAtoB:1,laneBtoA:0,oneway:true});
const graph=(nodes=[])=>({nodes:new Map(nodes.map((n,i)=>[i,n]))});
test('overlapping parallel roads get a boundary',()=>{
 assert.equal(nearbyRoadBoundaries([road('a',{x:0,y:0},{x:100,y:0}),road('b',{x:0,y:20},{x:100,y:20})],graph()).length,1);
});
test('separate and crossing roads do not get boundaries',()=>{
 let a=road('a',{x:0,y:0},{x:100,y:0});
 assert.equal(nearbyRoadBoundaries([a,road('b',{x:0,y:30},{x:100,y:30})],graph()).length,0);
 assert.equal(nearbyRoadBoundaries([a,road('b',{x:50,y:-40},{x:50,y:40})],graph()).length,0);
});
test('near a shared junction the mouth stays open',()=>{
 let node={x:10,y:0,z:0,degree:3,roads:new Set(['a','b'])};
 assert.equal(nearbyRoadBoundaries([road('a',{x:0,y:0},{x:20,y:0}),road('b',{x:0,y:18},{x:20,y:18})],graph([node])).length,0);
});
test('segments of the same road do not create a separator',()=>{
 let a=road('a',{x:0,y:0},{x:100,y:0});
 let b={...road('b',{x:0,y:20},{x:100,y:20}),groupId:'a'};
 assert.equal(nearbyRoadBoundaries([a,b],graph()).length,0);
});
test('widely separated elevations are left to overpass rendering',()=>{
 let a=road('a',{x:0,y:0},{x:100,y:0});
 let b=road('b',{x:0,y:48},{x:100,y:48},1);
 assert.equal(nearbyRoadBoundaries([a,b],graph()).length,0);
});
test('the cloverleaf preserves separate near-parallel ramp edges',async()=>{
 const {demoWorld,buildGraph}=await import('./src/model.js');
 const world=demoWorld('clover');
 assert.ok(nearbyRoadBoundaries(world.roads,buildGraph(world)).some(x=>x.clips.length>1));
});

function unevenJunction(){
 const world={...emptyWorld(1),terrain:[],roads:[
  {id:'left',a:{x:0,y:200},b:{x:200,y:200},z:0,laneAtoB:1,laneBtoA:1},
  {id:'right',a:{x:200,y:200},b:{x:400,y:200},z:0,laneAtoB:1,laneBtoA:2},
  {id:'top',a:{x:200,y:0},b:{x:200,y:200},z:0,laneAtoB:1,laneBtoA:1}
 ]};
 return world;
}
test('unequal T-junction widths flare the narrow approach into the wider road',()=>{
 const g=buildGraph(unevenJunction()),node=g.nodes.get('200,200@0'),flares=junctionWidthFlares(g,node);
 assert.equal(flares.length,1);const flare=flares[0];assert.equal(flare.otherId,'0,200@0');
 assert.equal(flare.sections[0].width,19);assert.equal(flare.sections.at(-1).width,15);assert.ok(flare.reach>=32);
 assert.equal(flare.sections[0].x,200);assert.ok(flare.sections.at(-1).x<170);
 for(let i=1;i<flare.sections.length;i++)assert.ok(flare.sections[i].width<=flare.sections[i-1].width);
 assert.equal(flareHalfWidth(flare,0),19);assert.equal(flareHalfWidth(flare,flare.reach),15);
});
test('equal-width junctions keep their existing outline',()=>{
 const world=unevenJunction();world.roads[1].laneBtoA=1;const g=buildGraph(world);
 assert.deepEqual(junctionWidthFlares(g,g.nodes.get('200,200@0')),[]);
});
test('a width change just before the T is included across a short road join',()=>{
 const world=unevenJunction();world.roads[0].b.x=199;world.roads[1].a.x=199;
 const g=buildGraph(world),flare=junctionWidthFlares(g,g.nodes.get('200,200@0'))[0];
 assert.ok(flare);assert.equal(flare.otherId,'199,200@0');assert.equal(flare.narrow,15);assert.equal(flare.wide,19);
 assert.ok(flare.reach>30);assert.equal(flare.sections.at(-1).width,15);
});
test('a junction width flare follows short curved road pieces',()=>{
 const world=unevenJunction();world.roads.shift();
 const points=[{x:200,y:200},{x:185,y:200},{x:170,y:202},{x:155,y:207},{x:135,y:217},{x:100,y:240}];
 for(let i=1;i<points.length;i++)world.roads.push({id:'curve'+i,groupId:'curve',a:points[i-1],b:points[i],z:0,laneAtoB:1,laneBtoA:1});
 const g=buildGraph(world),flare=junctionWidthFlares(g,g.nodes.get('200,200@0'))[0];
 assert.ok(flare);assert.ok(flare.sections.at(-1).y>205);assert.equal(flare.sections.at(-1).width,15);
 assert.ok(flare.sections.every(p=>[p.x,p.y,p.dx,p.dy,p.width].every(Number.isFinite)));
});
test('a width flare stops before a nearby intersection',()=>{
 const world=unevenJunction();world.roads.push({id:'nearby',a:{x:180,y:100},b:{x:180,y:300},z:0,laneAtoB:1,laneBtoA:1});
 const g=buildGraph(world),flare=junctionWidthFlares(g,g.nodes.get('200,200@0'))[0];
 assert.ok(flare);assert.ok(flare.reach<=9);assert.ok(flare.sections.at(-1).x>180);
});
test('junction flares leave roundabout mouths and rising ramps alone',()=>{
 const ring=unevenJunction();ring.roads[1].roundabout=true;let g=buildGraph(ring);assert.deepEqual(junctionWidthFlares(g,g.nodes.get('200,200@0')),[]);
 const ramp=unevenJunction();ramp.roads[0].z=1;ramp.roads[0].zEnd=0;g=buildGraph(ramp);assert.deepEqual(junctionWidthFlares(g,g.nodes.get('200,200@0')),[]);
});
