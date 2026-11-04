import {findHomeParking} from './home-parking.js';
import {parkingSlots} from './parking-layout.js';
import { roadOverlapsBuildings, worldBuildingFootprints } from './building-clearance.js';
import { travelLaneCenter } from './lane-geometry.js';
import { sampleClosedSpline } from './geometry.js';
import {prepareTrafficGraph,trafficSnapshot,signalState,turnType,approachRule,followingSpeed,pathMetrics} from './traffic-rules.js';
export {signalState} from './traffic-rules.js';
// Road topology, terrain, and traffic simulation. Coordinates are world units.
export const SIZE=1024;
export const distance=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
const zOf=r=>r.z??0;
export const nodeKey=(p,z=0)=>`${Math.round(p.x*100)/100},${Math.round(p.y*100)/100}@${z}`;
export function projection(p,a,b){const dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy,t=l?Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/l)):0;return {x:a.x+t*dx,y:a.y+t*dy,t}}
export function intersection(a,b,c,d){let x=b.x-a.x,y=b.y-a.y,u=d.x-c.x,v=d.y-c.y,den=x*v-y*u;if(Math.abs(den)<1e-8)return null;let t=((c.x-a.x)*v-(c.y-a.y)*u)/den,s=((c.x-a.x)*y-(c.y-a.y)*x)/den;if(t<-1e-7||t>1.0000001||s<-1e-7||s>1.0000001)return null;return {x:a.x+t*x,y:a.y+t*y,t,s}}
function rand(seed){let s=seed>>>0;return()=>{s=(Math.imul(s,1664525)+1013904223)>>>0;return s/4294967296}}
export function terrainFor(seed=1){let r=rand(seed);let bump=(cx,cy,rx,ry)=>Array.from({length:18},(_,i)=>{let a=i*Math.PI*2/18,scale=.88+r()*.23;return {x:cx+Math.cos(a)*rx*scale,y:cy+Math.sin(a)*ry*scale}});return [{kind:'water',points:[{x:0,y:0},{x:285,y:0},...bump(95,25,150,115).filter(p=>p.x>=0&&p.y>=0),{x:0,y:180}]},{kind:'forest',points:bump(900,130,105,72)},{kind:'forest',points:bump(115,875,110,65)},{kind:'mountain',points:bump(930,865,80,100)}]}
export function inside(p,poly){let yes=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){let a=poly[i],b=poly[j];if((a.y>p.y)!==(b.y>p.y)&&p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)yes=!yes}return yes}
// Placement uses the same rounded outline as the renderer. Cache sampled
// outlines and bounds because construction previews query them frequently.
const terrainOutlines=new WeakMap();
function terrainOutline(terrain){
 let cached=terrainOutlines.get(terrain),scale=terrain.kind==='forest'?1.08:1;
 if(cached&&cached.source===terrain.points&&cached.scale===scale)return cached;
 const points=sampleClosedSpline(terrain.points||[],scale);
 const bounds=points.reduce((b,p)=>({minX:Math.min(b.minX,p.x),maxX:Math.max(b.maxX,p.x),minY:Math.min(b.minY,p.y),maxY:Math.max(b.maxY,p.y)}),{minX:Infinity,maxX:-Infinity,minY:Infinity,maxY:-Infinity});
 cached={source:terrain.points,scale,points,bounds};terrainOutlines.set(terrain,cached);return cached;
}
export function terrainAt(w,p){
 for(const terrain of w.terrain||[]){
  if(terrain.kind==='field')continue;
  const {points,bounds}=terrainOutline(terrain);
  if(p.x>=bounds.minX&&p.x<=bounds.maxX&&p.y>=bounds.minY&&p.y<=bounds.maxY&&inside(p,points))return terrain.kind;
 }
 return 'land';
}
export function newSeed(){return Math.floor(Math.random()*0x7fffffff)}
export function mapName(seed){const a=['Willow','Moss','Cedar','Fern','Silver','Alder','Oak','Pine'],b=['Bay','Crossing','Haven','Point','Grove','Inlet','Hollow','Shore'];return `${a[seed%a.length]} ${b[Math.floor(seed/8)%b.length]}`}
export function emptyWorld(seed=newSeed()){return {version:1,size:SIZE,seed,name:mapName(seed),terrain:terrainFor(seed),roads:[],buildings:[],junctions:{},ports:[],demand:1}}
export function starterWorld(seed=newSeed()){let w=emptyWorld(seed);w.roads=[[[0,480],[1024,480]],[[512,160],[512,880]],[[190,270],[512,270]],[[512,720],[850,720]],[[220,270],[220,480]],[[800,480],[800,720]]].map(([a,b],i)=>({id:'r'+i,a:{x:a[0],y:a[1]},b:{x:b[0],y:b[1]},z:0,lanes:1,laneAtoB:1,laneBtoA:1}));w.buildings=[{id:'b1',type:'home',x:190,y:225},{id:'b2',type:'home',x:285,y:225},{id:'b3',type:'home',x:380,y:225},{id:'b4',type:'shop',x:720,y:440},{id:'b5',type:'shop',x:850,y:440},{id:'b6',type:'industry',x:680,y:765},{id:'b7',type:'industry',x:790,y:765},{id:'b8',type:'home',x:455,y:370}];w.ports=[{id:'west',side:'W',slot:3,x:0,y:480},{id:'east',side:'E',slot:3,x:1024,y:480}];w.junctions[nodeKey({x:512,y:480})]={mode:'signal',phase:12};return w}
export function migrate(w){if(!w.seed)w.seed=3874;if(!w.name)w.name=mapName(w.seed);if(!w.terrain)w.terrain=terrainFor(w.seed);for(const r of w.roads){r.z??=0;r.lanes??=1;r.laneAtoB??=r.lanes;r.laneBtoA??=(r.oneway?0:r.lanes);r.oneway=r.laneAtoB===0||r.laneBtoA===0}inferRoadGroups(w);for(const k of Object.keys(w.junctions)){if(!k.includes('@')){w.junctions[k+'@0']=w.junctions[k];delete w.junctions[k]}}return w}
export function validWorld(w){return !!w&&w.version===1&&Array.isArray(w.roads)&&Array.isArray(w.buildings)&&Array.isArray(w.ports)&&w.roads.every(r=>r.id&&[r.a?.x,r.a?.y,r.b?.x,r.b?.y].every(Number.isFinite)&&distance(r.a,r.b)>.01)&&w.buildings.every(b=>b.id&&Number.isFinite(b.x)&&Number.isFinite(b.y))}
export function validateRoad(w,a,b,z,zEnd=z,clearance=0){
 const length=distance(a,b);if(length<2)return 'Road is too short.';
 const nx=-(b.y-a.y)/length,ny=(b.x-a.x)/length,steps=Math.max(1,Math.ceil(length/5));
 const lateral=[0],across=Math.ceil(Math.max(0,clearance)/5);
 for(let i=1;i<=across;i++){let d=clearance*i/across;lateral.push(-d,d)}
 for(let i=0;i<=steps;i++){
  const t=i/steps,cx=a.x+(b.x-a.x)*t,cy=a.y+(b.y-a.y)*t,height=z+(zEnd-z)*t;
  for(const offset of lateral){
   const kind=terrainAt(w,{x:cx+nx*offset,y:cy+ny*offset});
   if(kind==='water'&&height<.85)return 'Water needs an elevated bridge. Choose elevation 1.';
   if(kind==='mountain')return 'Mountains block construction. Route around them.';
   if(kind==='forest'&&height<.85)return 'Forest blocks ground roads. Route around it.';
  }
 }
 return null;
}
// Validate the same sampled geometry that will be committed, including the
// height at every point of a curve or ramp.
export function validateRoadPath(world,points,z,zEnd,clearance=0){
 if(points.length<2)return {error:'Choose a road endpoint.',levels:[]};
 if(Math.abs(zEnd-z)>1)return {error:'Use one elevation level per ramp.',levels:[]};
 let lengths=[0];for(let i=1;i<points.length;i++)lengths.push(lengths.at(-1)+distance(points[i-1],points[i]));
 const total=lengths.at(-1),levels=lengths.map(length=>z+(zEnd-z)*(total?length/total:0));
 if(z!==zEnd&&total<90)return {error:'Ramp is too short; use at least 90 world units.',levels};
 const buildingShapes=worldBuildingFootprints(world);
 for(let i=1;i<points.length;i++){
  const a=points[i-1],b=points[i],dx=b.x-a.x,dy=b.y-a.y,length=Math.hypot(dx,dy);
  for(const road of world.roads){
   const rx=road.b.x-road.a.x,ry=road.b.y-road.a.y,rl=Math.hypot(rx,ry);if(length<.01||rl<.01||Math.abs(dx*ry-dy*rx)>length*rl*.001)continue;
   if(Math.abs((road.a.x-a.x)*dy-(road.a.y-a.y)*dx)/length>.02)continue;
   const t0=((road.a.x-a.x)*dx+(road.a.y-a.y)*dy)/(length*length),t1=((road.b.x-a.x)*dx+(road.b.y-a.y)*dy)/(length*length);
   const lo=Math.max(0,Math.min(t0,t1)),hi=Math.min(1,Math.max(t0,t1));if((hi-lo)*length<.5)continue;
   const t=(lo+hi)/2,p={x:a.x+dx*t,y:a.y+dy*t},u=projection(p,road.a,road.b).t;
   const newHeight=levels[i-1]+(levels[i]-levels[i-1])*t,oldHeight=(road.z??0)+((road.zEnd??road.z??0)-(road.z??0))*u;
   if(Math.abs(newHeight-oldHeight)<.01)return {error:'A road already occupies this path.',levels};
  }
  if(roadOverlapsBuildings(world,a,b,levels[i-1],levels[i],clearance,buildingShapes))return {error:'A building blocks this road.',levels};
  let error=validateRoad(world,a,b,levels[i-1],levels[i],clearance);if(error)return {error,levels};
 }
 return {error:null,levels};
}
export function buildGraph(world){let cuts=world.roads.map(r=>[{...r.a,t:0},{...r.b,t:1}]);for(let i=0;i<world.roads.length;i++)for(let j=i+1;j<world.roads.length;j++){const a=world.roads[i],b=world.roads[j];if(zOf(a)!==(a.zEnd??zOf(a))||zOf(b)!==(b.zEnd??zOf(b))||zOf(a)!==zOf(b))continue;let p=intersection(a.a,a.b,b.a,b.b);if(p){cuts[i].push({x:p.x,y:p.y,t:p.t});cuts[j].push({x:p.x,y:p.y,t:p.s})}}
// Join ramp endpoints to level roads without joining grade-separated crossings.
for(let i=0;i<world.roads.length;i++){let r=world.roads[i];for(const [p,z] of [[r.a,r.z??0],[r.b,r.zEnd??r.z??0]])for(let j=0;j<world.roads.length;j++){if(i===j)continue;let other=world.roads[j];if((other.z??0)!==z||(other.zEnd??other.z??0)!==z)continue;let q=projection(p,other.a,other.b);if(distance(p,q)<.02)cuts[j].push(q)}}
const homeParking=new Map(world.buildings.filter(b=>b.type==='home').map(b=>[b.id,findHomeParking(world,b)]));
const attachments=[];for(const b of [...world.buildings,...world.ports]){if(b.parkingDisconnected)continue;let best=null,level=b.z??0;world.roads.forEach((r,i)=>{let homeSpot=homeParking.get(b.id),roadId=homeSpot?.roadId||b.accessRoadId;if(roadId&&r.id!==roadId)return;if(zOf(r)!==level||(r.zEnd??zOf(r))!==level)return;let anchor=homeParking.get(b.id)||b.accessPoint||b,p=projection(anchor,r.a,r.b),d=distance(anchor,p);if(!best||d<best.d)best={...p,d,i}});if(best&&best.d<100){cuts[best.i].push(best);attachments.push({id:b.id,node:nodeKey(best,level)})}}
// Homes without room on their own plot use a nearby connected parking lot.
for(const b of world.buildings){if(b.type!=='home'||homeParking.get(b.id))continue;let attachment=attachments.find(a=>a.id===b.id);if(!attachment)continue;let choices=world.buildings.filter(lot=>lot.type==='parking'&&!lot.parkingDisconnected&&parkingSlots(lot).length&&distance(b,lot)<200).map(lot=>({lot,attachment:attachments.find(a=>a.id===lot.id)})).filter(x=>x.attachment).sort((left,right)=>distance(b,left.lot)-distance(b,right.lot));
 attachment.parkingCandidates=choices.map(choice=>({id:choice.lot.id,node:choice.attachment.node}));
}
const nodes=new Map(),edges=[];world.roads.forEach((r,i)=>{let z=zOf(r),sorted=cuts[i].sort((a,b)=>a.t-b.t).filter((p,j,arr)=>!j||Math.abs(p.t-arr[j-1].t)>1e-6);for(const p of sorted){let level=p.t>.99999?(r.zEnd??z):z,id=nodeKey(p,level);if(!nodes.has(id))nodes.set(id,{id,x:p.x,y:p.y,z:level,out:[],incoming:[],roads:new Set()});nodes.get(id).roads.add(r.groupId??r.id)}for(let j=1;j<sorted.length;j++){let a=nodeKey(sorted[j-1],sorted[j-1].t>.99999?(r.zEnd??z):z),b=nodeKey(sorted[j],sorted[j].t>.99999?(r.zEnd??z):z);if(a===b)continue;let directions=[[a,b,r.laneAtoB??r.lanes??1,0],[b,a,r.laneBtoA??(r.oneway?0:r.lanes??1),1]];for(const [from,to,laneCount,direction] of directions){if(laneCount<1)continue;let e={id:`${r.id}:${from}>${to}`,road:r.id,from,to,lanes:laneCount,direction,zFrom:nodes.get(from).z,zTo:nodes.get(to).z,roundabout:!!r.roundabout,length:distance(nodes.get(from),nodes.get(to))};edges.push(e);nodes.get(from).out.push(e);nodes.get(to).incoming.push(e)}}});for(const n of nodes.values())n.neighbors=new Set();for(const e of edges){nodes.get(e.from).neighbors.add(e.to);nodes.get(e.to).neighbors.add(e.from)}const roadById=new Map(world.roads.map(r=>[r.id,r]));for(const n of nodes.values()){n.degree=n.neighbors.size;n.junction=n.degree>=3;n.mergeGroups=new Set();if(!n.junction)continue;let arms=[...n.incoming,...n.out].map(e=>{let road=roadById.get(e.road),other=nodes.get(e.from===n.id?e.to:e.from);if(!road||!other)return null;let dx=other.x-n.x,dy=other.y-n.y,len=Math.hypot(dx,dy)||1;return {road,group:road.groupId??road.id,neighbor:other.id,dx:dx/len,dy:dy/len,total:(road.laneAtoB??road.lanes??1)+(road.laneBtoA??(road.oneway?0:road.lanes??1))}}).filter(Boolean);for(const main of arms){if(main.total<3)continue;let same=arms.filter(a=>a.group===main.group);if(new Set(same.map(a=>a.neighbor)).size<2)continue;let slip=arms.some(a=>a.group!==main.group&&a.total===1&&a.road.oneway&&Math.abs(a.dx*main.dx+a.dy*main.dy)>(a.road.ramp ? .15 : .35));if(slip)n.mergeGroups.add(main.group)}}for(const attachment of attachments){if(!attachment.parkingCandidates)continue;let seen=new Set([attachment.node]),queue=[attachment.node];for(let i=0;i<queue.length;i++)for(const neighbor of nodes.get(queue[i])?.neighbors||[])if(!seen.has(neighbor)){seen.add(neighbor);queue.push(neighbor)}let choice=attachment.parkingCandidates.find(candidate=>seen.has(candidate.node));if(choice){attachment.node=choice.node;attachment.parkingId=choice.id}else attachment.parkingUnavailable=true;delete attachment.parkingCandidates}
return prepareTrafficGraph({nodes,edges,edgeById:new Map(edges.map(e=>[e.id,e])),roadById,attachments,homeParking})}
export function movementKey(a,la,b,lb){return `${a.id}#${la}>${b.id}#${lb}`}
// Default lane connections keep vehicles on their lane through a continuous
// road. A one-lane slip road joins the outside lane of a wider road; explicit
// junction connections can override either rule.
export function defaultMovement(graph,a,la,b,lb){
 const ar=graph.roadById.get(a.road),br=graph.roadById.get(b.road);if(!ar||!br)return true;
 const ag=ar.groupId??ar.id,bg=br.groupId??br.id;
 if(ag===bg&&a.direction===b.direction)return lb===Math.min(la,b.lanes-1);
 const count=r=>(r.laneAtoB??r.lanes??1)+(r.laneBtoA??(r.oneway?0:r.lanes??1));
 const single=r=>!!r.oneway&&count(r)===1;
 if(count(ar)>=3&&single(br))return la===a.lanes-1;
 if(single(ar)&&count(br)>=3)return lb===b.lanes-1;
 const turn=turnType(a,b);
 if(turn==='left')return la===0&&lb===0;
 if(turn==='right')return la===a.lanes-1&&lb===b.lanes-1;
 // Straight movements preserve lane order instead of crossing each other.
 return lb===Math.min(la,b.lanes-1);
}
// Dijkstra over directed edges and entry lanes. Lane changes may use a
// continuous run of short pieces, but never cross a real junction or a rule.
export function route(graph,start,end,junctions={},startLane=null,startEdge=null,startRemaining=null){
 if(!graph.nodes.has(start)||!graph.nodes.has(end))return null;
 const dist=new Map(),prev=new Map(),todo=[];
 const push=(id,cost)=>{let i=todo.length;todo.push({id,cost});while(i){let parent=(i-1)>>1;if(todo[parent].cost<=cost)break;todo[i]=todo[parent];i=parent}todo[i]={id,cost}};
 const pop=()=>{const first=todo[0],last=todo.pop();if(todo.length){let i=0;while(i*2+1<todo.length){let child=i*2+1;if(child+1<todo.length&&todo[child+1].cost<todo[child].cost)child++;if(todo[child].cost>=last.cost)break;todo[i]=todo[child];i=child}todo[i]=last}return first};
 for(const e of graph.nodes.get(start).out)for(let lane=0;lane<e.lanes;lane++){
  if(startLane!==null&&lane!==startLane||startEdge!==null&&e.id!==startEdge)continue;
  const id=`${e.id}#${lane}`;dist.set(id,e.length);prev.set(id,null);push(id,e.length);
 }
 let goal=null;
 while(todo.length){
  let item=pop(),u=item.id;if(item.cost!==dist.get(u))continue;
  let [eid,laneText]=u.split('#'),edge=graph.edgeById.get(eid),lane=+laneText;if(!edge)continue;
  if(edge.to===end){goal=u;break}
  let road=graph.roadById.get(edge.road),group=road?.groupId??road?.id,current=edge,via=[],extra=0;
  let available=startEdge===edge.id&&startRemaining!==null?startRemaining:edge.length;
  for(let count=0;count<100;count++){
   const node=graph.nodes.get(current.to),cfg=junctions[current.to];if(!node)break;
   for(let departure=0;departure<edge.lanes;departure++){
    if(departure!==lane&&available<55*Math.abs(departure-lane)+15)continue;
    if(count&&departure===lane)continue;
    for(const next of node.out){
     if(next.to===current.from)continue;
     for(let nl=0;nl<next.lanes;nl++){
      if(Array.isArray(cfg?.connectors)?!cfg.connectors.includes(movementKey(current,departure,next,nl)):!defaultMovement(graph,current,departure,next,nl))continue;
      let v=`${next.id}#${nl}`,d=dist.get(u)+extra+next.length+35*Math.abs(departure-lane);
      if(d<(dist.get(v)??Infinity)){dist.set(v,d);prev.set(v,{id:u,exitLane:departure,via:[...via]});push(v,d)}
     }
    }
   }
   if(node.junction||Array.isArray(cfg?.connectors)||edge.lanes<2||available>=55*(edge.lanes-1)+15)break;
   const continuations=node.out.filter(next=>{let r=graph.roadById.get(next.road);return next.to!==current.from&&(r?.groupId??r?.id)===group&&next.direction===edge.direction&&next.lanes===edge.lanes});
   if(continuations.length!==1)break;
   current=continuations[0];if(current.id===edge.id)break;
   via.push(current);extra+=current.length;available+=current.length;
  }
 }
 if(!goal)return null;
 const states=[];for(let u=goal;u;){let step=prev.get(u);states.unshift({id:u,step});u=step?.id??null}
 const result=[];for(let i=0;i<states.length;i++){
  let [eid,l]=states[i].id.split('#'),lane=+l,step=states[i+1]?.step;
  result.push({...graph.edgeById.get(eid),lane,exitLane:step?.exitLane??lane});
  for(const part of step?.via??[])result.push({...part,lane,exitLane:step.exitLane});
 }
 return result;
}
export function roadPieces(road,graph){let a=road.a,b=road.b,points=[{...a,t:0},{...b,t:1}];for(const n of graph.nodes.values()){if(n.z!==zOf(road))continue;let p=projection(n,a,b);if(distance(p,n)<.05&&p.t>1e-5&&p.t<.99999)points.push(p)}points.sort((x,y)=>x.t-y.t);return points.slice(1).map((p,i)=>({a:points[i],b:p}))}
// Check the target lane on the current and adjacent same-road pieces before a
// lane change. A changing vehicle reserves both lanes until the move completes.
function sameRoadRun(graph,a,b){
 if(!a||!b||a.direction!==b.direction||a.lanes!==b.lanes)return false;
 return a.group===b.group&&!graph.nodes.get(a.to)?.junction;
}
function canChangeLane(sim,car,target,laneCars){
 const unsafe=(other,gap)=>{const body=((car.length??10)+(other.length??10))/2+6,front=Math.max(24,body+Math.max(0,car.speed-other.speed)*.7),back=Math.max(24,body+Math.max(0,other.speed-car.speed)*.9);return gap<front&&gap>-back};
 const edge=car.edge,key=(e,l)=>e.trafficIndex*4+l,occupied=(part,test)=>(laneCars.get(key(part,target))||[]).some(o=>o.id!==car.id&&test(o));
 if(occupied(edge,o=>unsafe(o,o.pos-car.pos)))return false;
 let part=edge,behind=car.pos,seen=new Set([edge.id]);
 while(behind<75){
  let node=sim.graph.nodes.get(part.from);if(!node||node.junction)break;
  let choices=node.incoming.filter(p=>!seen.has(p.id)&&sameRoadRun(sim.graph,p,part)&&target<p.lanes);
  if(choices.length!==1)break;let prev=choices[0];seen.add(prev.id);
  if(occupied(prev,o=>unsafe(o,-(behind+prev.length-o.pos))))return false;
  behind+=prev.length;part=prev;
 }
 let ahead=edge.length-car.pos;part=edge;
 for(let i=car.index+1;i<car.path.length&&ahead<75;i++){
  let next=car.path[i];if(!sameRoadRun(sim.graph,part,next)||target>=next.lanes)break;
  if(occupied(next,o=>unsafe(o,ahead+o.pos)))return false;
  ahead+=next.length;part=next;
 }
 return true;
}
// Distance until the planned change must be complete, including curve pieces.
function laneChangeDeadline(graph,car){
 const metrics=pathMetrics(graph,car.path);
 return metrics.starts[metrics.runEnds[car.index]+1]-metrics.starts[car.index]-car.pos;
}
// Keep a vehicle on the same physical road when graph edges are split or
// replaced by an edit. Never move it to another elevation or the opposite lane.
export function remapVehicle(car,oldGraph,newGraph){
 const old=car.edge,from=oldGraph.nodes.get(old.from),to=oldGraph.nodes.get(old.to);
 if(!from||!to||!old.length)return null;
 const t=Math.max(0,Math.min(1,car.pos/old.length)),p={x:from.x+(to.x-from.x)*t,y:from.y+(to.y-from.y)*t,z:from.z+(to.z-from.z)*t};
 const dx=(to.x-from.x)/old.length,dy=(to.y-from.y)/old.length,oldRoad=oldGraph.roadById.get(old.road),oldGroup=oldRoad?.groupId??oldRoad?.id;
 let best=null;
 for(const edge of newGraph.edges){
  const a=newGraph.nodes.get(edge.from),b=newGraph.nodes.get(edge.to);if(!a||!b||!edge.length)continue;
  if(((b.x-a.x)*dx+(b.y-a.y)*dy)/edge.length<.999)continue;
  const q=projection(p,a,b),z=a.z+(b.z-a.z)*q.t,d=distance(p,q);
  if(d>.08||Math.abs(p.z-z)>.01)continue;
  const road=newGraph.roadById.get(edge.road),sameRoad=edge.road===old.road,sameGroup=(road?.groupId??road?.id)===oldGroup;
  const score=d+(sameRoad ? 0 : sameGroup ? .001 : .002);
  if(!best||score<best.score)best={edge,pos:q.t*edge.length,score};
 }
 if(!best)return null;
 const destination=car.destinationId&&newGraph.attachments.find(a=>a.id===car.destinationId)?.node||car.destinationNode||car.path.at(-1)?.to;
 const lane=Math.min(car.lane??old.lane??0,best.edge.lanes-1),edge={...best.edge,lane,exitLane:lane};
 return {...car,edge,path:[edge],index:0,pos:best.pos,lane,laneChange:null,destinationNode:destination,routeBlocked:false};
}
// Geometry is immutable for the lifetime of a graph, so reuse movement comparisons.
const movementConflictCaches=new WeakMap(),movementGeometryCaches=new WeakMap();
// Compare the lane paths through a junction; keep unknown movements conservative.
function movementsConflict(graph,a,b,inLane,outLane,other,otherNext,otherInLane,otherOutLane){
 if(!a||!b||!other||!otherNext)return true;
 if(a.id===other.id&&inLane===otherInLane)return false; // car-following handles the queue
 if(b.id===otherNext.id&&outLane===otherOutLane)return true;
 let cache=movementConflictCaches.get(graph);if(!cache)movementConflictCaches.set(graph,cache=new Map());
 const movementId=(a,b,il,ol)=>((a.trafficIndex*graph.edges.length+b.trafficIndex)*4+il)*4+ol;
 let first=movementId(a,b,inLane,outLane),second=movementId(other,otherNext,otherInLane,otherOutLane);
 if(first>second)[first,second]=[second,first];
 let bucket=cache.get(first);if(bucket?.has(second))return bucket.get(second);
 let geometries=movementGeometryCaches.get(graph);if(!geometries)movementGeometryCaches.set(graph,geometries=new Map());
 const samples=(incoming,outgoing,il,ol)=>{
  const id=movementId(incoming,outgoing,il,ol);if(geometries.has(id))return geometries.get(id);
  const j=graph.nodes.get(incoming.to),from=graph.nodes.get(incoming.from),to=graph.nodes.get(outgoing.to),ir=graph.roadById.get(incoming.road),or=graph.roadById.get(outgoing.road);
  if(!j||!from||!to||!ir||!or||il>=incoming.lanes||ol>=outgoing.lanes)return null;
  let ix=j.x-from.x,iy=j.y-from.y,ox=to.x-j.x,oy=to.y-j.y,im=Math.hypot(ix,iy),om=Math.hypot(ox,oy);if(!im||!om)return null;ix/=im;iy/=im;ox/=om;oy/=om;
  const r=16,io=travelLaneCenter(ir,incoming.direction,il),oo=travelLaneCenter(or,outgoing.direction,ol),p0={x:j.x-ix*r-iy*io,y:j.y-iy*r+ix*io},p3={x:j.x+ox*r-oy*oo,y:j.y+oy*r+ox*oo},p1={x:p0.x+ix*r*.65,y:p0.y+iy*r*.65},p2={x:p3.x-ox*r*.65,y:p3.y-oy*r*.65};
  const points=Array.from({length:17},(_,i)=>{let t=i/16,u=1-t;return {x:u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x,y:u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y}});
  geometries.set(id,points);return points;
 };
 const p=samples(a,b,inLane,outLane),q=samples(other,otherNext,otherInLane,otherOutLane);if(!p||!q)return true;
 const conflict=p.some(x=>q.some(y=>(x.x-y.x)**2+(x.y-y.y)**2<49));
 if(!bucket)cache.set(first,bucket=new Map());bucket.set(second,conflict);return conflict;
}

export function signalIsRed(time,phase,horizontal,offset=0){return signalState(time,phase,horizontal,offset)!=='green'}
export class Simulation{constructor(world){this.world=world;this.graph=buildGraph(world);this.cars=[];this.parked=[];this.time=0;this.timer=0;this.completed=0;this.cleared=0;this.serial=0;this.locks=new Map();this.routeCache=new Map();this.ruleRevision=0}
 rebuild(graph=null){let old=this.graph,next=graph||buildGraph(this.world);this.graph=next;this.cars=this.cars.flatMap(car=>{if(car.requiresParking&&!this.world.buildings.some(b=>b.id===car.destinationId))return [];let mapped=remapVehicle(car,old,next);if(!mapped)return [];mapped.routeRevision=this.ruleRevision-1;mapped.junctionCommit=null;mapped.junctionEntered=false;mapped.priorJunctions=[];return [mapped]});this.parked=this.parked.filter(p=>this.world.buildings.some(b=>b.id===p.destinationId));for(const p of this.parked){p.nextTrip=null;p.departStarted=null;p.departurePausedAt=null;p.departureBlocked=false;p.retryTripAt=0}this.locks.clear();this.routeCache.clear()}
 parkingCapacity(b){return b.type==='home'?(this.graph.homeParking?.get(b.id)?1:0):b.type==='parking'?parkingSlots(b).length:b.type==='shop'?4:3}
 availableSlot(id){let b=this.world.buildings.find(b=>b.id===id);if(!b)return null;let used=new Set([...this.cars.filter(c=>c.destinationId===id).map(c=>c.parkingSlot),...this.parked.filter(c=>c.destinationId===id).map(c=>c.parkingSlot),...this.parked.filter(c=>c.nextTrip?.destinationId===id).map(c=>c.nextTrip.parkingSlot)]);for(let i=0;i<this.parkingCapacity(b);i++)if(!used.has(i))return i;return -1}
 planDeparture(parked){
  const from=this.graph.attachments.find(a=>a.id===parked.destinationId);if(!from)return null;
  const options=this.graph.attachments.filter(a=>!a.parkingUnavailable&&a.id!==from.id&&a.node!==from.node),offset=Math.floor(Math.random()*Math.max(1,options.length));
  let waitingTrip=null;
  for(let i=0;i<options.length;i++){
   const to=options[(i+offset)%options.length],destinationId=to.parkingId||to.id,slot=this.availableSlot(destinationId);
   const key='park|'+from.node+'|'+to.node;let path=this.routeCache.get(key);
   if(path===undefined){path=route(this.graph,from.node,to.node,this.world.junctions)||null;this.routeCache.set(key,path)}
   if(!path?.length||!this.entryIsClear(path,parked.length??10))continue;
   if(slot!==-1)return {destinationId,destinationNode:to.node,parkingSlot:slot,path};
   // A parked vehicle may leave even when every destination is full. It
   // reserves a real space as soon as one becomes available and waits on the
   // road before arriving until then. Its departure frees its original slot.
   waitingTrip??={destinationId,destinationNode:to.node,parkingSlot:null,path,requiresParking:true};
  }
  return waitingTrip;
 }
 entryIsClear(path,length=10){
  const edge=path[0],lane=edge.lane??0,parts=new Map();let covered=0;
  for(const part of path){if(covered>35)break;parts.set(part.id,{offset:covered,lane:part.lane??lane});covered+=part.length;if(this.graph.nodes.get(part.to)?.junction)break}
  let part=edge,behind=0;const seen=new Set([edge.id]);
  while(behind<60&&!this.graph.nodes.get(part.from)?.junction){
   const incoming=this.graph.nodes.get(part.from)?.incoming.filter(e=>!seen.has(e.id)&&sameRoadRun(this.graph,e,part));if(incoming?.length!==1)break;
   part=incoming[0];seen.add(part.id);behind+=part.length;parts.set(part.id,{offset:-behind,lane});
  }
  return !this.cars.some(car=>{
   const entry=parts.get(car.edge.id);if(!entry||((car.lane??car.edge.lane??0)!==entry.lane&&car.laneChange?.to!==entry.lane))return false;
   const position=entry.offset+car.pos,space=(length+(car.length??10))/2+5;
   return position<space+12&&position>-(space+Math.max(10,(car.speed??0)*.8));
  });
 }
 findParkingAlternative(car){
  if(car.laneChange||car.junctionCommit)return false;
  const options=[];
  for(const attachment of this.graph.attachments){
   const id=attachment.parkingId||attachment.id;if(id===car.destinationId||attachment.parkingUnavailable)continue;
   const slot=this.availableSlot(id);if(slot===-1)continue;
   const node=this.graph.nodes.get(attachment.node),origin=this.graph.nodes.get(car.edge.to);
   options.push({attachment,id,slot,score:distance(node,origin)+(slot===null?2048:0)});
  }
  options.sort((a,b)=>a.score-b.score);
  const offset=(car.parkingOptionCursor??0)%Math.max(1,options.length);
  for(let i=0;i<Math.min(6,options.length);i++){
   const option=options[(offset+i)%options.length];
   const path=route(this.graph,car.edge.from,option.attachment.node,this.world.junctions,car.lane,car.edge.id,car.edge.length-car.pos);
   if(!path?.length)continue;
   car.destinationId=option.id;car.destinationNode=option.attachment.node;car.parkingSlot=option.slot;car.requiresParking=option.slot!==null;
   car.path=path;car.index=0;car.edge=path[0];car.routeRevision=this.ruleRevision;car.routeBlocked=false;delete car.blockedSince;delete car.retryRouteAt;car.parkingOptionCursor=0;return true;
  }
  car.parkingOptionCursor=offset+6;return false;
 }
 depart(parked){
  const trip=parked.nextTrip;if(!trip)return false;const edge=trip.path[0],lane=edge.lane??0;
  if(this.cars.length>=140||!this.entryIsClear(trip.path,parked.length??10))return false;
  this.cars.push({id:++this.serial,destinationId:trip.destinationId,destinationNode:trip.destinationNode,parkingSlot:trip.parkingSlot,requiresParking:!!trip.requiresParking,edge,path:trip.path,index:0,pos:0,speed:0,lane,routeRevision:this.ruleRevision,length:parked.length,color:parked.color});return true;
 }
 rulesChanged(){
  this.ruleRevision++;this.routeCache.clear();
  // Revalidate a departure without restarting its animation or residence time.
  // Replaying all departure timers made parked cars jump whenever a link changed.
  for(const p of this.parked)if(p.nextTrip)p.tripRevision=this.ruleRevision-1;
 }

 spawn(){let a=this.graph.attachments.filter(a=>!a.parkingUnavailable);if(a.length<2||this.cars.length>=140)return;let from=a[Math.floor(Math.random()*a.length)],to=a[Math.floor(Math.random()*a.length)];if(from.id===to.id||from.node===to.node)return;let destinationId=to.parkingId||to.id,parkingSlot=this.availableSlot(destinationId);if(parkingSlot===-1)return;let maxLanes=Math.max(1,...this.graph.nodes.get(from.node).out.map(e=>e.lanes)),preferred=this.serial%maxLanes,key=from.node+'|'+to.node+'#'+preferred,path=this.routeCache.get(key);if(path===undefined){path=route(this.graph,from.node,to.node,this.world.junctions,preferred)||route(this.graph,from.node,to.node,this.world.junctions)||null;this.routeCache.set(key,path)}if(!path?.length)return;let lane=path[0].lane??(this.serial%(path[0].lanes||1));if(!this.entryIsClear(path))return;let truck=this.world.buildings.find(b=>b.id===destinationId)?.type==='industry'&&this.serial%3===0;this.cars.push({id:++this.serial,destinationId,destinationNode:to.node,parkingSlot,edge:path[0],path,index:0,pos:0,speed:0,lane,routeRevision:this.ruleRevision,length:truck?17:10,color:truck?'#d18d59':['#4d90c7','#edb654','#d67961','#83b66e','#9b8abe'][this.serial%5]})}
 tick(dt){
  if(!Number.isFinite(dt)||dt<=0)return;
  if(dt>.05+1e-9){while(dt>.05+1e-9){this.tick(.05);dt-=.05}if(dt>1e-9)this.tick(dt);return}
  this.time+=dt;
  // Claim freed slots before generating new traffic, preserving capacity and
  // giving vehicles already on the road priority over new arrivals.
  for(const car of this.cars)if(car.requiresParking&&car.parkingSlot===null){const slot=this.availableSlot(car.destinationId);if(slot!==null&&slot!==-1)car.parkingSlot=slot}
  // A full destination should lead to another real space (or a connected
  // exit), rather than a stationary queue that only the jam timeout can clear.
  let parkingReplans=0;
  for(const car of this.cars)if(car.requiresParking&&car.parkingSlot===null){
   car.parkingSearchSince??=this.time;
   if(parkingReplans<4&&this.time-car.parkingSearchSince>=5&&this.time>=(car.retryParkingAt??0)){parkingReplans++;this.findParkingAlternative(car);car.retryParkingAt=this.time+5}
  }
  for(const p of this.parked){
   if(this.time-p.arrived<32)continue;
   if(p.nextTrip&&p.tripRevision!==this.ruleRevision&&this.time>=(p.retryTripAt??0)){
    const trip=this.planDeparture(p);
    if(trip){if(p.departurePausedAt!=null)p.departStarted+=this.time-p.departurePausedAt;p.departurePausedAt=null;p.departureBlocked=false;p.nextTrip=trip;p.tripRevision=this.ruleRevision}
    else{p.departurePausedAt??=this.time;p.departureBlocked=true;p.retryTripAt=this.time+2}
   }
   if(!p.nextTrip&&this.time>=(p.retryTripAt??0)){p.nextTrip=this.planDeparture(p);p.tripRevision=this.ruleRevision;p.retryTripAt=this.time+2}
   if(p.nextTrip)p.departStarted??=this.time;
   if(p.nextTrip&&!p.departureBlocked&&this.time-p.departStarted>=3&&!this.entryIsClear(p.nextTrip.path,p.length??10))p.tripRevision=this.ruleRevision-1;
  }
  const parkedBefore=this.parked.length;
  this.parked=this.parked.filter(p=>p.departureBlocked||!p.nextTrip||this.time-p.departStarted<3||!this.depart(p));
  // Departures free their original spaces during this step. Give those to
  // existing waiting drivers before a newly generated trip can reserve them.
  if(this.parked.length<parkedBefore)for(const car of this.cars)if(car.requiresParking&&car.parkingSlot===null){const slot=this.availableSlot(car.destinationId);if(slot!==null&&slot!==-1)car.parkingSlot=slot}
  this.timer+=dt*this.world.demand;while(this.timer>.32){this.timer-=.32;this.spawn()}
  const snapshot=trafficSnapshot(this.graph,this.cars),carsById=new Map(this.cars.map(c=>[c.id,c]));
  for(const [n,locks] of this.locks){for(const id of locks.keys())if(!carsById.has(id)||!(carsById.get(id).junctionCommit===n||carsById.get(id).priorJunctions?.some(held=>held.id===n)))locks.delete(id);if(!locks.size)this.locks.delete(n)}// A yielding vehicle stopped before the line must not reserve the crossing
// against traffic that has priority. Do not revoke a vehicle already entering.
for(const [id,locks] of this.locks){
 const j=this.graph.nodes.get(id),cfg=this.world.junctions[id];if(!j||cfg?.mode==='signal')continue;
 for(const [carId,lock] of locks){
  const car=carsById.get(carId),approach=this.graph.edgeById.get(lock.approach),following=this.graph.edgeById.get(lock.exit),rule=approachRule(this.graph,j,approach,cfg);
  if(!car||!['yield','stop'].includes(rule)||car.junctionEntered)continue;
  const arrival=(snapshot.approaches.get(id)||[]).find(entry=>entry.car.id===carId);if(!arrival)continue;
  const stop=j.stopDistance;
  if(arrival.distance<stop-.5)continue;
  const priority=(snapshot.approaches.get(id)||[]).some(other=>other.car.id!==car.id&&other.edge.id!==lock.approach&&other.distance<55&&!['yield','stop'].includes(approachRule(this.graph,j,other.edge,cfg))&&movementsConflict(this.graph,approach,following,lock.inLane??car.lane,lock.lane??car.lane,other.edge,other.next,other.edge.exitLane??other.car.lane,other.next.lane??other.car.lane));
  if(priority){locks.delete(carId);car.junctionCommit=null;car.junctionEntered=false}
 }
 if(!locks.size)this.locks.delete(id);
}
const cars=[...this.cars].sort((a,b)=>(a.junctionWaitingSince??Infinity)-(b.junctionWaitingSince??Infinity)||a.id-b.id),laneCars=snapshot.lanes,reserve=(car,lane)=>snapshot.reserve(snapshot.byId.get(car.id),lane);
let replans=0;for(const c of cars){
 c.lane??=c.edge.lane??0;
 c.waitReason=null;
 // Retry unreachable trips after edits, then clear a permanently stranded car
 // rather than blocking a lane and holding its parking reservation forever.
 if(c.routeBlocked&&this.time-(c.blockedSince??this.time)>=60){c.done=true;this.cleared=(this.cleared??0)+1;continue}
 if(c.routeBlocked&&this.time>=(c.retryRouteAt??Infinity))c.routeRevision=this.ruleRevision-1;
 c.routeRevision??=this.ruleRevision;if(c.routeRevision!==this.ruleRevision&&!c.laneChange&&!c.junctionCommit){if(replans>=6){c.speed=0;continue}replans++;let destination=c.destinationNode||c.path.at(-1)?.to,replanned=destination&&route(this.graph,c.edge.from,destination,this.world.junctions,c.lane,c.edge.id,Math.max(0,c.edge.length-c.pos));c.routeRevision=this.ruleRevision;if(replanned?.length){c.path=replanned;c.index=0;c.edge=replanned[0];c.routeBlocked=false;delete c.blockedSince;delete c.retryRouteAt}else{c.routeBlocked=true;c.blockedSince??=this.time;c.retryRouteAt=this.time+5}}if(c.routeBlocked){c.waitReason='No available route';c.speed=0;continue}let e=c.edge,exitLane=e.exitLane??c.lane,needed=Math.abs(exitLane-c.lane),remainingOnEdge=e.length-c.pos;if(needed&&!c.laneChange&&laneChangeDeadline(this.graph,c)<=90*needed+45){let target=c.lane+Math.sign(exitLane-c.lane);if(canChangeLane(this,c,target,laneCars)){c.laneChange={from:c.lane,to:target,distance:0,length:32};reserve(c,target)}}let gap=Infinity,safeGap=17,leader=null,travel=0,sameRun=true;for(let k=c.index;k<c.path.length&&travel<90;k++){let part=c.path[k];if(k>c.index)sameRun=sameRun&&sameRoadRun(this.graph,c.path[k-1],part);let lane=sameRun?c.lane:(part.lane??c.lane),occupiedLanes=sameRun&&c.laneChange?[lane,c.laneChange.to]:[lane];for(const occupiedLane of occupiedLanes)for(const other of laneCars.get(part.trafficIndex*4+occupiedLane)||[]){if(other.id===c.id)continue;let d=travel+other.pos-(k===c.index?c.pos:0);if(d>0&&d<gap){gap=d;leader=other;safeGap=(c.length??10)/2+(other.length??10)/2+5}}travel+=part.length-(k===c.index?c.pos:0)}let limit=Math.min(c.path[c.index+1]?Infinity:e.length,c.pos+Math.max(0,gap-safeGap)),node=this.graph.nodes.get(e.to),isJunction=node.junction,next=c.path[c.index+1];if(!next&&c.requiresParking&&c.parkingSlot===null)limit=Math.min(limit,Math.max(0,e.length-1));if(needed&&!c.laneChange){let remaining=laneChangeDeadline(this.graph,c);limit=Math.min(limit,c.pos+Math.max(0,remaining-(40*needed+10)))}// Decide before the stop line, including space beyond the junction. Once a
// car enters, retain its reservation until its rear has cleared the crossing.
const metrics=pathMetrics(this.graph,c.path);let ahead=e.length-c.pos;for(let k=metrics.junctions[c.index];k>=0&&k<c.path.length;k=c.path.length){
 ahead=metrics.starts[k+1]-metrics.starts[c.index]-c.pos;if(ahead>=90)break;
 let approach=c.path[k],j=this.graph.nodes.get(approach.to),following=c.path[k+1];
 if(j?.junction&&following){
  let cfg=this.world.junctions[j.id]||{mode:'priority',phase:12},origin=this.graph.nodes.get(approach.from),horizontal=Math.abs(j.x-origin.x)>=Math.abs(j.y-origin.y),red=cfg.mode==='signal'&&signalIsRed(this.time,cfg.phase,horizontal,cfg.offset);
  let stop=j.stopDistance;
  if(ahead<=stop+3&&c.speed<3)c.junctionWaitingSince??=this.time;
  let exitSpace=stop+(c.length??10)/2+17,covered=0,blocked=false,exitGap=Infinity;
  for(let n=k+1;n<c.path.length&&covered<exitSpace;n++){
   let part=c.path[n],lane=part.lane??c.lane;
   for(const other of laneCars.get(part.trafficIndex*4+lane)||[])if(other.id!==c.id){
    exitGap=Math.min(exitGap,covered+other.pos-Math.max(17,((c.length??10)+(other.length??10))/2+4));
    // A moving platoon is not a blocked exit. Test the space when this car's
    // rear will clear the box; car-following still enforces the physical gap.
    const arrival=Math.min(2,(ahead+stop+(c.length??10)/2)/Math.max(16,c.speed));
    const prediction=other.speed>=6?other.speed*arrival:0;
    if(covered+other.pos+prediction<exitSpace)blocked=true;
   }
   covered+=part.length;
   if(this.graph.nodes.get(part.to)?.junction)break;
  }
  let locks=this.locks.get(j.id),locked=locks&&[...locks.entries()].some(([id,lock])=>id!==c.id&&movementsConflict(this.graph,approach,following,approach.exitLane??c.lane,following.lane??c.lane,this.graph.edgeById.get(lock.approach),this.graph.edgeById.get(lock.exit),lock.inLane??0,lock.lane??0));
  const contenders=snapshot.approaches.get(j.id)||[];
  const conflicts=other=>other.car.id!==c.id&&movementsConflict(this.graph,approach,following,approach.exitLane??c.lane,following.lane??c.lane,other.edge,other.next,other.edge.exitLane??other.car.lane,other.next.lane??other.car.lane);
  let roundaboutEntry=cfg.mode!=='signal'&&!approach.roundabout&&contenders.some(o=>o.edge.roundabout&&o.distance<55+o.car.speed*.5&&conflicts(o));
  let rule=cfg.mode==='signal'?'auto':approachRule(this.graph,j,approach,cfg);
  const nearby=contenders.filter(o=>o.distance<stop+Math.max(18,o.car.speed*1.3)&&conflicts(o));
  const otherRule=o=>approachRule(this.graph,j,o.edge,cfg);
  let signedYield=(rule==='yield'||rule==='stop')&&nearby.some(o=>!['yield','stop'].includes(otherRule(o))||(rule==='stop'&&otherRule(o)==='yield'));
  const oldest=(a,b)=>(a.car.junctionWaitingSince??Infinity)-(b.car.junctionWaitingSince??Infinity)||a.car.id-b.car.id;
  // At equal priority: give way to the right; left turns yield to oncoming
  // straight traffic. Resolve a stationary circular wait by arrival order.
  const own={car:c,edge:approach,next:following,distance:ahead};
  const deadlock=c.junctionWaitingSince!=null&&this.time-c.junctionWaitingSince>=2&&nearby.length&&nearby.every(o=>o.car.speed<1&&o.car.junctionWaitingSince!=null&&this.time-o.car.junctionWaitingSince>=2&&otherRule(o)===rule);
  let yieldToMain=nearby.some(o=>{
   if(cfg.mode==='signal')return turnType(approach,following)==='left'&&approach.hx*o.edge.hx+approach.hy*o.edge.hy<-.65&&turnType(o.edge,o.next)!=='left'&&signalState(this.time,cfg.phase,Math.abs(o.edge.hx)>=Math.abs(o.edge.hy),cfg.offset)==='green';
   const priority=otherRule(o);if(priority==='stop'||priority==='yield')return false;
   if(rule==='yield'||rule==='stop')return true;
   if(rule==='auto'&&priority==='priority')return true;
   if(rule==='priority'&&priority==='auto')return false;
   if(deadlock)return oldest(o,own)<0;
   const opposing=approach.hx*o.edge.hx+approach.hy*o.edge.hy<-.65;
   if(opposing)return turnType(approach,following)==='left'&&turnType(o.edge,o.next)!=='left';
   if(rule==='priority')return false;
   const onRight=approach.hx*(-o.edge.hy)-approach.hy*(-o.edge.hx)>.25;
   return onRight;
  });
  if(rule==='stop'&&!signedYield)yieldToMain=nearby.some(o=>otherRule(o)==='stop'&&oldest(o,own)<0);
  let rightOnRed=cfg.mode==='signal'&&cfg.rightOnRed===true&&red&&signalState(this.time,cfg.phase,horizontal,cfg.offset)==='red'&&signalState(this.time,cfg.phase,!horizontal,cfg.offset)==='green'&&turnType(approach,following)==='right'&&(approach.exitLane??c.lane)===approach.lanes-1&&!c.laneChange;
  if(rightOnRed)signedYield=contenders.some(o=>conflicts(o)&&o.distance<stop+Math.max(20,o.car.speed*1.5)&&(o.car.junctionCommit===j.id||signalState(this.time,cfg.phase,Math.abs(o.edge.hx)>=Math.abs(o.edge.hy),cfg.offset)==='green'));
  let stopping=false;
  if((rule==='stop'||rightOnRed)&&c.junctionCommit!==j.id){
   if(c.stopJunction!==j.id||c.stopApproach!==approach.id){c.stopJunction=j.id;c.stopApproach=approach.id;c.stopSince=null}
   if(ahead<=stop+0.5&&c.speed<0.5)c.stopSince??=this.time;
   stopping=c.stopSince===null||this.time-c.stopSince<1;
  }
  let outNode=this.graph.nodes.get(following.to),ix=j.x-origin.x,iy=j.y-origin.y,ox=outNode.x-j.x,oy=outNode.y-j.y,rightTurn=(ix*oy-iy*ox)>0.25*Math.hypot(ix,iy)*Math.hypot(ox,oy);
  let roundaboutExit=approach.roundabout&&!following.roundabout;
  let exitBlocked=blocked&&!((rightTurn||roundaboutExit)&&exitGap>2);
  let mustWait=(red&&!rightOnRed)||locked||exitBlocked||roundaboutEntry||yieldToMain||signedYield||stopping;
  if(c.junctionCommit!==j.id){
   if(mustWait){let junctionLimit=c.pos+Math.max(0,ahead-stop);if(junctionLimit<limit){limit=junctionLimit;c.waitReason=red&&!rightOnRed?(signalState(this.time,cfg.phase,horizontal,cfg.offset)==='amber'?'Amber light':'Red light'):stopping?(rightOnRed?'Stopping before right on red':'Stopping at stop sign'):exitBlocked?'Exit is blocked':roundaboutEntry?'Waiting for roundabout traffic':locked?'Waiting for crossing traffic':(signedYield||yieldToMain)?'Giving way to traffic':'Waiting at junction'}}
   else if(ahead<=stop+3){if(c.junctionCommit&&c.junctionEntered){(c.priorJunctions??=[]).push({id:c.junctionCommit,remaining:c.junctionClearance});c.junctionEntered=false}c.junctionCommit=j.id;c.stopJunction=null;c.stopSince=null;c.junctionClearance=stop+(c.length??10)/2;{if(!locks)this.locks.set(j.id,locks=new Map());locks.set(c.id,{approach:approach.id,exit:following.id,inLane:approach.exitLane??c.lane,lane:following.lane??c.lane})}}
  }
  break;
 }
 if(following)ahead+=following.length;
}
let cruise=(c.length??10)>12?40:48;
 if(e.roundabout)cruise=Math.min(cruise,30);
 // Brake ahead of a real turn, including bends made from short pieces.
 for(let k=metrics.bends[c.index];k>=0&&k<c.path.length-1;k=metrics.bends[k+1]){
  const bendDistance=metrics.starts[k+1]-metrics.starts[c.index]-c.pos;if(bendDistance>=75)break;
  const part=c.path[k],after=c.path[k+1],angle=Math.acos(Math.max(-1,Math.min(1,part.hx*after.hx+part.hy*after.hy)));
  const corner=Math.max(18,Math.min(38,Math.sqrt(650/angle)));cruise=Math.min(cruise,Math.sqrt(corner*corner+2*28*Math.max(0,bendDistance-16)));
 }
 let remaining=Math.max(0,limit-c.pos),target=Math.min(cruise,Math.sqrt(2*28*remaining),leader?followingSpeed(c,leader,gap):Infinity),speed=Math.max(0,Math.min(c.speed+35*dt,Math.max(target,c.speed-55*dt)));c.speed=Math.min(speed,remaining/dt);if(c.speed<3&&leader&&gap<safeGap+3)c.waitReason='Waiting for vehicle ahead';if(c.speed<3&&!c.waitReason){c.waitReason=leader&&gap<safeGap+12?'Waiting for vehicle ahead':needed?'Waiting to change lanes':!next&&c.requiresParking&&c.parkingSlot===null?'Waiting for parking':'Waiting for space'}c.acceleration=(c.speed-(snapshot.byId.get(c.id)?.speed??0))/dt;let moved=c.speed*dt;c.pos+=moved;
 // Break persistent circular waits only after two minutes without meaningful
 // movement; normal signal phases and temporary queues keep their vehicles.
 c.progressAt??=this.time;c.progressDistance=(c.progressDistance??0)+moved;
 if(c.progressDistance>=3){c.progressDistance=0;c.progressAt=this.time}
 if(this.time-c.progressAt>=120){c.done=true;this.cleared=(this.cleared??0)+1;continue}
 if(c.priorJunctions?.length){
 for(const held of c.priorJunctions)held.remaining-=moved;
 for(const held of c.priorJunctions)if(held.remaining<=0){let locks=this.locks.get(held.id);if(locks){locks.delete(c.id);if(!locks.size)this.locks.delete(held.id)}}
 c.priorJunctions=c.priorJunctions.filter(held=>held.remaining>0);
}
if(c.junctionCommit&&c.junctionEntered){c.junctionClearance-=moved;if(c.junctionClearance<=0){let held=this.locks.get(c.junctionCommit);if(held){held.delete(c.id);if(!held.size)this.locks.delete(c.junctionCommit)}c.junctionCommit=null;c.junctionEntered=false}}if(c.laneChange){c.laneChange.distance+=moved;if(c.laneChange.distance>=c.laneChange.length){c.lane=c.laneChange.to;c.laneChange=null}}while(!c.done&&c.pos>=e.length-1e-7){next=c.path[c.index+1];isJunction=this.graph.nodes.get(e.to).junction;if(!next&&c.requiresParking&&c.parkingSlot===null){c.pos=e.length-1;c.speed=0;break}let continuous=next&&sameRoadRun(this.graph,e,next),carry=continuous&&next.lane===e.lane&&next.exitLane===e.exitLane;if((c.routeRevision!==this.ruleRevision&&!c.laneChange&&!c.junctionCommit)||(!carry&&(c.laneChange||c.lane!==(e.exitLane??c.lane)))){c.pos=Math.max(0,e.length-.02);c.speed=0;break}if(next){if(isJunction&&c.junctionCommit!==e.to){c.pos=Math.max(0,e.length-.02);c.speed=0;break}if(isJunction&&c.junctionCommit===e.to){c.junctionEntered=true;c.junctionWaitingSince=null}c.edge=next;c.index++;if(!continuous)c.lane=next.lane??Math.min(c.lane,next.lanes-1);c.pos=Math.max(0,c.pos-e.length);e=c.edge}else{let destination=this.world.buildings.find(b=>b.id===c.destinationId);if(destination&&c.parkingSlot!==null&&c.parkingSlot!==undefined){let from=this.graph.nodes.get(e.from),to=this.graph.nodes.get(e.to),dx=to.x-from.x,dy=to.y-from.y,len=Math.hypot(dx,dy)||1,road=this.graph.roadById.get(e.road),f=e.lanes,back=road?(road[e.direction===0?'laneBtoA':'laneAtoB']??(road.oneway?0:road.lanes??1)):1,offset=road?travelLaneCenter(road,e.direction,c.lane):(back-f)*4+(c.lane+.5)*8;this.parked.push({id:c.id,destinationId:c.destinationId,parkingSlot:c.parkingSlot,arrived:this.time,color:c.color,length:c.length,start:{x:to.x-dy/len*offset,y:to.y+dx/len*offset},angle:Math.atan2(dy,dx)})}c.done=true;this.completed++}}}this.cars=this.cars.filter(c=>!c.done)}}

function inferRoadGroups(w){let eligible=w.roads.filter(r=>!r.groupId&&distance(r.a,r.b)<70),key=p=>`${Math.round(p.x*10)},${Math.round(p.y*10)},${rLevel(p)}`;function rLevel(p){return Math.round((p.z??0)*100)/100}let endpoints=new Map();for(const r of eligible)for(const p of [r.a,r.b]){let k=key({...p,z:r.z??0});(endpoints.get(k)||endpoints.set(k,[]).get(k)).push(r)}let seen=new Set();for(const seed of eligible){if(seen.has(seed))continue;let component=[],q=[seed];seen.add(seed);while(q.length){let r=q.pop();component.push(r);for(const p of [r.a,r.b]){let k=key({...p,z:r.z??0}),connected=endpoints.get(k)||[];if(connected.length!==2)continue;for(const n of connected)if(!seen.has(n)&&n.z===r.z&&n.laneAtoB===r.laneAtoB&&n.laneBtoA===r.laneBtoA&&n.oneway===r.oneway){seen.add(n);q.push(n)}}}if(component.length>1){let id=`joined-${component[0].id}`;for(const r of component)r.groupId=id}}}
const addPath=(w,pts,o={})=>{let groupId=o.groupId||`demo-${w.roads.length}-${Math.random().toString(36).slice(2,7)}`;for(let i=1;i<pts.length;i++){let a=pts[i-1],b=pts[i];if(distance(a,b)<1)continue;w.roads.push({id:`${groupId}-${i}`,groupId,a:{x:a.x,y:a.y},b:{x:b.x,y:b.y},z:(o.z??0)+(((o.zEnd??o.z??0)-(o.z??0))*(i-1)/(pts.length-1)),zEnd:(o.z??0)+(((o.zEnd??o.z??0)-(o.z??0))*i/(pts.length-1)),lanes:o.lanes??1,laneAtoB:o.forward??o.lanes??1,laneBtoA:o.backward??(o.oneway?0:o.lanes??1),oneway:o.oneway??false,roundabout:o.roundabout??false,ramp:o.ramp??false})}}
const curve=(a,c,b,n=20)=>Array.from({length:n+1},(_,i)=>{let t=i/n,u=1-t;return {x:u*u*a.x+2*u*t*c.x+t*t*b.x,y:u*u*a.y+2*u*t*c.y+t*t*b.y}});
const cubic=(a,c,d,b,n=28)=>Array.from({length:n+1},(_,i)=>{let t=i/n,u=1-t;return {x:u*u*u*a.x+3*u*u*t*c.x+3*u*t*t*d.x+t*t*t*b.x,y:u*u*u*a.y+3*u*u*t*c.y+3*u*t*t*d.y+t*t*t*b.y}});
function demoTerrain(w,id){if(id==='bay')w.terrain=[{kind:'water',points:[{x:0,y:0},{x:375,y:0},{x:420,y:50},{x:390,y:155},{x:300,y:215},{x:175,y:225},{x:0,y:194}]},{kind:'mountain',points:[{x:0,y:710},{x:140,y:675},{x:285,y:730},{x:360,y:860},{x:260,y:1024},{x:0,y:1024}]}];else if(id==='clover')w.terrain=[{kind:'water',points:[{x:0,y:0},{x:260,y:0},{x:300,y:100},{x:200,y:160},{x:0,y:140}]},{kind:'forest',points:[{x:710,y:30},{x:970,y:20},{x:1000,y:180},{x:880,y:235},{x:760,y:150}]},{kind:'forest',points:[{x:20,y:760},{x:210,y:710},{x:340,y:820},{x:220,y:1010},{x:0,y:970}]},{kind:'mountain',points:[{x:780,y:760},{x:1024,y:700},{x:1024,y:1024},{x:730,y:1024},{x:680,y:900}]}];else w.terrain=terrainFor(w.seed)}
export function demoWorld(id){let seed={ramp:2101,bay:3107,editor:4211,stack:5183,clover:6209,magic:7297,district:8351,'lane-lab':9431,'depth-lab':10111,starter:1337}[id]||1337,w=emptyWorld(seed);w.demoId=id;demoTerrain(w,id);const P=(x,y)=>({x,y}),line=(a,b,o={})=>addPath(w,[P(...a),P(...b)],o),bend=(a,c,b,o={})=>addPath(w,curve(P(...a),P(...c),P(...b)),o),round=(cx,cy,r,o={})=>addPath(w,Array.from({length:25},(_,i)=>P(cx+r*Math.cos(-i*Math.PI/12),cy+r*Math.sin(-i*Math.PI/12))),{...o,oneway:true,forward:o.lanes||1,backward:0,roundabout:true});
if(id==='ramp'){
 w.terrain=[{kind:'field',color:'#f2e8bf',points:[{x:0,y:700},{x:350,y:800},{x:450,y:1024},{x:0,y:1024}]}];
 const A=t=>[730*t,780-780*t],B=t=>[1024*t,360+620*t];
 line(A(0),A(1),{lanes:2});line(B(0),B(1),{z:1,lanes:2});
 const ramp=(a,c,d,b,z,zEnd)=>addPath(w,cubic(P(...a),P(...c),P(...d),P(...b),44),{z,zEnd,oneway:true,forward:1,backward:0});
 ramp(A(.68),[420,340],[270,350],B(.13),0,1);
 ramp(B(.60),[750,680],[590,570],A(.37),1,0);
 w.buildings=[{id:'rh1',type:'home',x:150,y:580},{id:'rh2',type:'home',x:195,y:535},{id:'rs1',type:'shop',x:540,y:170},{id:'ri1',type:'industry',x:590,y:120}];
}
if(id==='bay'){
 // Coastal roundabout, diagonal highway, local streets and curved ramps.
 const freeway=y=>[900-y*.4,y];
 let west=cubic(P(0,305),P(95,360),P(220,430),P(315,430),36);addPath(w,west);round(390,430,75);line([465,430],[1024,430]);
 line(freeway(0),freeway(1024),{z:1,lanes:2});
 const ramp=(a,b,c,d,z,zEnd)=>addPath(w,cubic(...[a,b,c,d].map(p=>P(...p)),40),{z,zEnd,oneway:true,forward:1,backward:0});
 let ne=[390+75*Math.cos(Math.PI/6),392.5],se=[390+75*Math.cos(Math.PI/6),467.5];
 ramp(ne,[570,390],[755,300],freeway(180),0,1);
 ramp(freeway(800),[630,720],[600,540],se,1,0);
 ramp(freeway(260),[900,220],[1000,370],[840,430],1,0);
 ramp([840,430],[980,550],[790,690],freeway(720),0,1);
 line([390,355],[390,250]);line([390,250],[610,250]);line([610,250],[610,170]);
 bend([390,505],[390,680],[560,750]);line([560,750],[650,800]);bend([650,800],[770,890],[900,850]);bend([900,850],[985,850],[985,800]);line([985,800],[985,585]);
 line([800,585],[985,585]);line([985,585],[985,430]);
 w.buildings=[];let add=(type,x,y,scale=1)=>w.buildings.push({id:'bay-building-'+w.buildings.length,type,x,y,scale});
 // Space the row by distance along the street, rather than by Bézier parameter:
 // parameter steps get shorter at the roundabout and make the last roofs overlap.
 const lengths=[0];for(let i=1;i<west.length;i++)lengths.push(lengths[i-1]+distance(west[i-1],west[i]));
 for(let i=0;i<8;i++){
  let target=lengths.at(-1)*(.03+.86*i/7),k=Math.max(1,lengths.findIndex(d=>d>=target));
  let a=west[k-1],b=west[k],t=(target-lengths[k-1])/(lengths[k]-lengths[k-1]),dx=b.x-a.x,dy=b.y-a.y,l=Math.hypot(dx,dy)||1;
  add('home',a.x+dx*t+dy/l*38,a.y+dy*t-dx/l*38);
 }
 for(let i=0;i<5;i++)add('home',420+i*39,220);
 add('shop',175,451);add('industry',875,785,1.8);
}
if(id==='editor'){
 w.terrain=[{kind:'field',color:'#c7df98',points:[{x:0,y:800},{x:170,y:830},{x:300,y:1024},{x:0,y:1024}]}];
 const v=y=>[500,y],d=x=>[x,780-x*.5];
 line(v(0),v(1024),{lanes:2});line(d(0),d(1024),{z:1,lanes:2});
 const ramp=(a,c,d,b,z,zEnd)=>addPath(w,cubic(P(...a),P(...c),P(...d),P(...b),55),{z,zEnd,oneway:true,forward:1,backward:0});
 ramp(v(140),[500,300],[910,270],d(150),0,1);
 ramp(d(900),[830,330],[250,660],v(900),1,0);
 ramp(v(280),[485,450],[270,510],d(80),0,1);
 ramp(d(950),[780,385],[515,615],v(760),1,0);
 w.buildings=[{id:'eh1',type:'home',x:465,y:90},{id:'eh2',type:'home',x:465,y:145},{id:'es1',type:'shop',x:535,y:930},{id:'es2',type:'shop',x:535,y:875}];
}
if(id==='stack'){
 // Two diagonal highways with four flowing, grade-separated connectors.
 w.terrain=[{kind:'mountain',points:[{x:-120,y:-80},{x:1050,y:-80},{x:1050,y:220},{x:800,y:175},{x:570,y:255},{x:250,y:310},{x:0,y:280}]}];
 const A=t=>[1024*t,300+460*t],B=t=>[1024*t,1024*(1-t)];
 line(A(0),A(1),{z:1,lanes:2});line(B(0),B(1),{lanes:2});
 const routeRamp=(segments,z,zEnd)=>{let points=[];for(const segment of segments){let [a,c,d,b]=segment.map(p=>P(...p)),part=cubic(a,c,d,b,18);if(points.length)part.shift();points.push(...part)}addPath(w,points,{z,zEnd,oneway:true,forward:1,backward:0})};
 routeRamp([[A(.2),[250,420],[390,390],[400,480]],[[400,480],[400,540],[400,610],[400,650]],[[400,650],[400,725],[300,720],B(.25)]],1,0);
 routeRamp([[B(.75),[700,300],[560,285],[560,390]],[[560,390],[560,460],[560,590],[560,650]],[[560,650],[565,710],[735,625],A(.8)]],0,1);
 routeRamp([[A(.2),[260,430],[255,535],[340,540]],[[340,540],[430,540],[570,540],[650,540]],[[650,540],[730,540],[675,340],B(.75)]],1,0);
 routeRamp([[B(.25),[290,700],[245,580],[340,580]],[[340,580],[430,580],[570,580],[650,580]],[[650,580],[730,580],[745,640],A(.8)]],0,1);
 line([140,315],[850,210]);line([850,210],B(.84));
 line([600,900],[960,900]);line([960,900],B(.94));
 w.buildings=[];let add=(type,x,y)=>w.buildings.push({id:'stack-'+w.buildings.length,type,x,y});
 for(let i=0;i<8;i++)add('industry',190+i*72,285-i*10);
 add('shop',650,930);add('shop',770,930);add('home',900,930);
}
if(id==='clover'){
 // Two intersecting highways, four loop ramps and four outside slip roads.
 // Each highway has two outside connections so traffic can use the whole interchange.
 w.ground='#eae3df';
 w.terrain=[
  {kind:'water',color:'#85cce3',points:[{x:-70,y:-40},{x:610,y:-40},{x:575,y:70},{x:450,y:145},{x:340,y:165},{x:215,y:245},{x:100,y:250},{x:-70,y:170}]},
  {kind:'field',color:'#d1e17e',points:[{x:825,y:90},{x:1080,y:80},{x:1080,y:1024},{x:920,y:1024},{x:830,y:860},{x:860,y:700},{x:800,y:500},{x:795,y:280}]},
  {kind:'field',color:'#d4e88b',points:[{x:-50,y:510},{x:115,y:560},{x:235,y:740},{x:310,y:1024},{x:-50,y:1080}]}
 ];
 const H=x=>[x,440+.14*x],D=y=>[830-.62*y,y];
 line(H(0),H(1024),{z:1,lanes:2});
 line(D(0),D(1024),{lanes:2});
 const ramp=(a,c,d,b,z,zEnd)=>addPath(w,cubic(P(...a),P(...c),P(...d),P(...b),56),{z,zEnd,oneway:true,forward:1,backward:0,ramp:true});
 // Tight inner loops, running clockwise between the two carriageways.
 ramp(H(355),[135,485],[225,220],D(330),1,0);
 ramp(D(320),[790,220],[910,505],H(700),0,1);
 ramp(D(700),[175,860],[105,535],H(355),0,1);
 ramp(H(700),[955,585],[650,895],D(700),1,0);
 // Wider outside connectors provide the remaining four turning movements.
 ramp(D(180),[610,240],[170,200],H(55),0,1);
 ramp(H(960),[800,520],[930,285],D(180),1,0);
 ramp(H(55),[80,555],[115,765],D(850),1,0);
 ramp(D(850),[490,830],[865,760],H(960),0,1);
 w.buildings=[];
 w.ports=[
  {id:'clover-west',x:H(0)[0],y:H(0)[1],z:1},
  {id:'clover-east',x:H(1024)[0],y:H(1024)[1],z:1},
  {id:'clover-north',x:D(0)[0],y:D(0)[1],z:0},
  {id:'clover-south',x:D(1024)[0],y:D(1024)[1],z:0}
 ];
}

if(id==='magic'){
 w.ground='#ead7a5';w.terrain=[{kind:'field',color:'#f3efe7',points:[{x:-50,y:-30},{x:300,y:-30},{x:350,y:220},{x:300,y:420},{x:390,y:650},{x:390,y:1050},{x:-50,y:1050}]},{kind:'mountain',points:[{x:880,y:0},{x:1024,y:0},{x:1024,y:310},{x:910,y:220}]}];
 const centers=Array.from({length:5},(_,i)=>{let a=-Math.PI/2+i*2*Math.PI/5;return [520+145*Math.cos(a),500+145*Math.sin(a)]}),r=48;
 const nearest=(c,target)=>{let angle=Math.atan2(target[1]-c[1],target[0]-c[0]),step=Math.round(angle/(Math.PI/12));return [c[0]+r*Math.cos(step*Math.PI/12),c[1]+r*Math.sin(step*Math.PI/12)]};
 centers.forEach(c=>round(c[0],c[1],r));
 for(let i=0;i<5;i++){let a=centers[i],b=centers[(i+1)%5];line(nearest(a,b),nearest(b,a))}
 const outer=[[520,120],[880,370],[800,830],[240,830],[135,370]];
 for(let i=0;i<5;i++)line(nearest(centers[i],outer[i]),outer[i]);
 line([135,370],[40,370]);line([240,830],[40,830]);line([800,830],[980,830]);line([880,370],[980,370]);line([520,120],[520,0]);
 line([40,240],[135,240]);line([135,240],[135,370]);line([40,240],[40,830]);line([40,830],[240,830]);
 line([880,370],[880,240]);line([880,240],[980,240]);line([980,240],[980,830]);
 w.buildings=[];let add=(type,x,y)=>w.buildings.push({id:'magic-'+w.buildings.length,type,x,y});
 for(let i=0;i<8;i++){add('home',75,280+i*55);add('industry',945,280+i*55)}line([135,235],[310,235]);line([310,235],[310,370]);line([310,370],[135,370]);line([135,235],[135,830]);for(let row=0;row<5;row++){let y=420+row*72;line([135,y],[310,y]);for(let col=0;col<5;col++){add('home',155+col*31,y-25);add('home',155+col*31,y+25)}}for(let i=0;i<5;i++)add('home',155+i*31,205);line([310,708],[310,830]);line([310,830],[240,830]);
 add('shop',485,80);add('shop',555,80);add('shop',285,800);add('shop',760,800);
}
if(id==='district'){
 w.terrain=[{kind:'field',color:'#c1df96',points:[{x:90,y:0},{x:1024,y:0},{x:1024,y:420},{x:800,y:450},{x:590,y:340},{x:440,y:370},{x:290,y:280},{x:80,y:370}]},{kind:'field',color:'#c9e5a1',points:[{x:0,y:620},{x:340,y:650},{x:360,y:1024},{x:0,y:1024}]},{kind:'mountain',points:[{x:940,y:700},{x:1024,y:620},{x:1024,y:1024},{x:790,y:1024},{x:830,y:840}]}];
 round(400,550,75);line([0,550],[325,550]);line([475,550],[1024,550]);line([400,475],[400,0]);line([400,625],[400,1024]);
 line([130,0],[130,550]);line([0,720],[400,720]);line([0,865],[400,865]);line([220,550],[220,1024]);line([80,550],[80,1024]);
 const arc=cubic(P(400,260),P(475,120),P(670,115),P(700,310),40);addPath(w,arc);bend([700,310],[710,440],[700,550]);
 const outer=cubic(P(450,145),P(585,45),P(755,125),P(770,340),36);addPath(w,outer);line([450,145],[400,145]);line([770,340],[770,550]);
 line([800,0],[800,1024],{z:1,lanes:2});
 const ramp=(a,c,d,b,z,zEnd)=>addPath(w,cubic(P(...a),P(...c),P(...d),P(...b),36),{z,zEnd,oneway:true,forward:1,backward:0});
 ramp([600,550],[680,535],[790,405],[800,300],0,1);ramp([800,760],[775,720],[680,600],[600,550],1,0);
 ramp([800,350],[900,340],[935,460],[880,550],1,0);ramp([880,550],[950,630],[900,720],[800,700],0,1);
 w.buildings=[];let add=(type,x,y,scale=1,variant='')=>w.buildings.push({id:'district-'+w.buildings.length,type,x,y,scale,variant});
 for(let k=3;k<arc.length-3;k+=4){let p=arc[k],p0=arc[k-1],p1=arc[k+1],dx=p1.x-p0.x,dy=p1.y-p0.y,l=Math.hypot(dx,dy)||1;add('home',p.x+dy/l*34,p.y-dx/l*34);add('home',p.x-dy/l*35,p.y+dx/l*35)}
 for(let k=3;k<outer.length-3;k+=4){let p=outer[k],p0=outer[k-1],p1=outer[k+1],dx=p1.x-p0.x,dy=p1.y-p0.y,l=Math.hypot(dx,dy)||1;add('home',p.x+dy/l*35,p.y-dx/l*35)}
 for(let y of [685,755,830,900])for(let x of [40,110,150,190,250,290,330,370])add('home',x,y);
 add('shop',180,330,2.6);add('shop',300,590,1.4,'purple');add('shop',300,680);add('industry',930,590,1.7);add('industry',840,645,1.7);
}
if(id==='lane-lab'){
 w.terrain=[{kind:'field',color:'#c8dfa0',points:[{x:0,y:0},{x:1024,y:0},{x:1024,y:325},{x:700,y:340},{x:440,y:265},{x:0,y:335}]},{kind:'water',points:[{x:0,y:720},{x:180,y:730},{x:240,y:1024},{x:0,y:1024}]}];
 line([0,450],[1024,450],{oneway:true,forward:3,backward:0});
 addPath(w,cubic(P(720,450),P(825,450),P(800,650),P(900,650),40),{oneway:true,forward:1,backward:0,ramp:true});
 addPath(w,cubic(P(120,650),P(220,650),P(180,450),P(280,450),40),{oneway:true,forward:1,backward:0,ramp:true});
 line([900,650],[1024,650]);line([120,650],[0,650]);
 w.ports=[{id:'lab-west',side:'W',slot:3,x:0,y:450},{id:'lab-east',side:'E',slot:3,x:1024,y:450}];
 w.buildings=[{id:'lab-a',type:'home',x:30,y:615},{id:'lab-b',type:'home',x:75,y:615},{id:'lab-c',type:'shop',x:920,y:615},{id:'lab-d',type:'shop',x:975,y:615},{id:'lab-e',type:'industry',x:950,y:690}];
}
if(id==='depth-lab'){
 // The sloping road is above the ground road at their crossing, even though
 // both occupy the same coarse rendering layer there.
 w.terrain=[];
 line([0,400],[1024,400]);
 line([512,0],[512,100]);
 line([512,100],[512,900],{z:0,zEnd:1});
 line([512,900],[512,1024],{z:1});
 w.ports=[{id:'depth-west',x:0,y:400},{id:'depth-east',x:1024,y:400},{id:'depth-north',x:512,y:0},{id:'depth-south',x:512,y:1024,z:1}];
 w.buildings=[];
}
if(id==='starter'){let x=starterWorld(seed);x.demoId='starter';x.name='Little Bay';return x}if(id==='blank'){w.name='Open Canvas';w.demoId='blank';return w}w.name=({ramp:'Ramp Junction',bay:'Coastal Interchange',editor:'Ramp Workshop',stack:'Stacked Interchange',clover:'Cloverleaf',magic:'Five Roundabouts',district:'Garden District','lane-lab':'Lane & Ramp Lab','depth-lab':'Elevation Crossing Lab'}[id]||mapName(seed));return w}

