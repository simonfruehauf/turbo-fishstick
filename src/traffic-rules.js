// Shared by the driver model and the map: a displayed rule is a driving rule.
export function signalState(time,phase=12,horizontal=true,offset=0){
 const green=Math.max(5,Number(phase)||12),half=green+3,cycle=half*2;
 const elapsed=((time+(Number(offset)||0))%cycle+cycle)%cycle;
 const own=horizontal?elapsed:(elapsed+half)%cycle;
 return own<green?'green':own<green+2?'amber':'red';
}

export function prepareTrafficGraph(graph){
 graph.junctionNodes=[];
 for(const [trafficIndex,edge] of graph.edges.entries()){
  edge.trafficIndex=trafficIndex;
  const a=graph.nodes.get(edge.from),b=graph.nodes.get(edge.to);
  edge.hx=(b.x-a.x)/edge.length;edge.hy=(b.y-a.y)/edge.length;
  const road=graph.roadById.get(edge.road);
  edge.group=road?.groupId??road?.id;
 }
 for(const node of graph.nodes.values()){
  if(!node.junction)continue;
  graph.junctionNodes.push(node);
  node.width=Math.max(26,...node.incoming.concat(node.out).map(e=>{
   const r=graph.roadById.get(e.road);
   return 14+8*((r?.laneAtoB??r?.lanes??1)+(r?.laneBtoA??(r?.oneway?0:r?.lanes??1)));
  }));
  node.stopDistance=node.width/2+8;
 }
 return graph;
}

export function turnType(incoming,outgoing){
 if(!incoming||!outgoing)return 'straight';
 const cross=incoming.hx*outgoing.hy-incoming.hy*outgoing.hx;
 return cross>.25?'right':cross<-.25?'left':'straight';
}

export function approachRule(graph,node,edge,config={}){
 const explicit=config.approaches?.[edge.id];
 if(explicit&&explicit!=='auto')return explicit;
 if(edge.roundabout)return 'priority';
 if(node.incoming.some(e=>e.roundabout))return 'yield';
 if(node.mergeGroups.size)return node.mergeGroups.has(edge.group)?'priority':'yield';
 return 'auto';
}

const pathCaches=new WeakMap();
export function pathMetrics(graph,path){
 let cached=pathCaches.get(path);if(cached?.graph===graph)return cached;
 const starts=new Float64Array(path.length+1),junctions=new Int32Array(path.length),bends=new Int32Array(path.length),runEnds=new Int32Array(path.length);
 for(let i=0;i<path.length;i++)starts[i+1]=starts[i]+path[i].length;
 let junction=-1,bend=-1;
 for(let i=path.length-1;i>=0;i--){
  const edge=path[i],next=path[i+1],node=graph.nodes.get(edge.to);
  if(next&&node?.junction)junction=i;
  if(next&&Math.acos(Math.max(-1,Math.min(1,edge.hx*next.hx+edge.hy*next.hy)))>.3)bend=i;
  junctions[i]=junction;bends[i]=bend;
  const continues=next&&!node?.junction&&edge.group===next.group&&edge.direction===next.direction&&edge.lanes===next.lanes&&edge.lane===next.lane&&edge.exitLane===next.exitLane;
  runEnds[i]=continues?runEnds[i+1]:i;
 }
 cached={graph,starts,junctions,bends,runEnds};pathCaches.set(path,cached);return cached;
}

// One snapshot and two indexes per step. Drivers read the same positions even
// when a leader crosses an edge boundary earlier in the update loop.
export function trafficSnapshot(graph,cars){
 const lanes=new Map(),approaches=new Map(),byId=new Map();
 const reserve=(car,lane)=>{
  const key=car.edge.trafficIndex*4+lane;
  let bucket=lanes.get(key);if(!bucket)lanes.set(key,bucket=[]);
  if(!bucket.some(other=>other.id===car.id))bucket.push(car);
 };
 for(const source of cars){
  const car={id:source.id,edge:source.edge,path:source.path,index:source.index,pos:source.pos,speed:source.speed,length:source.length,lane:source.lane??source.edge.lane??0,laneChange:source.laneChange,junctionCommit:source.junctionCommit,junctionWaitingSince:source.junctionWaitingSince};byId.set(car.id,car);
  reserve(car,car.lane);if(car.laneChange)reserve(car,car.laneChange.to);
  const metrics=pathMetrics(graph,car.path),k=metrics.junctions[car.index];
  if(k<0)continue;
  const edge=car.path[k],ahead=metrics.starts[k+1]-metrics.starts[car.index]-car.pos;
  if(ahead<=110){let bucket=approaches.get(edge.to);if(!bucket)approaches.set(edge.to,bucket=[]);bucket.push({car,edge,next:car.path[k+1],distance:ahead})}
 }
 // Only the first vehicle in each incoming lane can claim right of way.
 // A second car behind a stop line must not keep another approach yielding.
 for(const [id,bucket] of approaches){
  const heads=new Map();
  for(const entry of bucket){const key=entry.edge.trafficIndex*4+(entry.edge.exitLane??entry.car.lane),head=heads.get(key);if(!head||entry.distance<head.distance)heads.set(key,entry)}
  approaches.set(id,[...heads.values()]);
 }
 return {lanes,approaches,byId,reserve};
}

// Smaller gaps in a queue, increasing space at speed, and enough braking room
// if the leader slows. All distances include both vehicles' actual lengths.
export function followingSpeed(car,leader,gap){
 const clearance=((car.length??10)+(leader.length??10))/2+5;
 const usable=Math.max(0,gap-clearance);
 return Math.min(Math.max(0,(usable-2)/.65),Math.sqrt(Math.max(0,leader.speed*leader.speed+2*32*usable)));
}
