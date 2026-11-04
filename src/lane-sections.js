import { LANE_WIDTH, laneCount, laneDivider } from './lane-geometry.js';
import { distance, nodeKey, projection, validateRoad } from './model.js';

// A road is stored as short directed segments. Build the editable section that
// contains the hit, stopping at junctions or a gap in the stored road group.
export function laneSection(world, graph, hitRoad, hitPoint) {
  const group = hitRoad.groupId ?? hitRoad.id;
  const roads = world.roads.filter(r => (r.groupId ?? r.id) === group);
  const fragments = [];
  const nodeAt = (p, z) => graph.nodes.get(nodeKey(p, z));
  const at = (r, t) => ({ x: r.a.x + (r.b.x-r.a.x)*t, y: r.a.y + (r.b.y-r.a.y)*t });
  const level = (r, t) => (r.z ?? 0) + ((r.zEnd ?? r.z ?? 0)-(r.z ?? 0))*t;
  for (const road of roads) {
    const cuts = [0, 1];
    if ((road.z ?? 0) === (road.zEnd ?? road.z ?? 0)) {
      for (const node of graph.nodes.values()) {
        if (!node.junction || Math.abs(node.z-(road.z ?? 0)) > 1e-6) continue;
        const q = projection(node, road.a, road.b);
        if (distance(q, node) < .05 && q.t > 1e-6 && q.t < 1-1e-6) cuts.push(q.t);
      }
    }
    cuts.sort((a,b)=>a-b);
    const unique = cuts.filter((v,i)=>i===0 || v-cuts[i-1]>1e-6);
    for (let i=1;i<unique.length;i++) {
      const lo=unique[i-1], hi=unique[i], a=at(road,lo), b=at(road,hi);
      fragments.push({road,lo,hi,a,b,z0:level(road,lo),z1:level(road,hi)});
    }
  }
  const hitT = projection(hitPoint,hitRoad.a,hitRoad.b).t;
  const seed = fragments.find(f=>f.road.id===hitRoad.id && hitT>=f.lo-1e-6 && hitT<=f.hi+1e-6);
  if (!seed) return [];
  const start = new Map(), end = new Map();
  const add=(map,key,value)=>{if(!map.has(key))map.set(key,[]);map.get(key).push(value)};
  for (const f of fragments) { add(start,nodeKey(f.a,f.z0),f); add(end,nodeKey(f.b,f.z1),f); }
  const sameLayout=(a,b)=>(a.road.laneAtoB??a.road.lanes??1)===(b.road.laneAtoB??b.road.lanes??1)
    && (a.road.laneBtoA??(a.road.oneway?0:a.road.lanes??1))===(b.road.laneBtoA??(b.road.oneway?0:b.road.lanes??1));
  const picked=new Set([seed]);
  for (const backwards of [true,false]) {
    let current=seed;
    while (true) {
      const point=backwards?current.a:current.b, z=backwards?current.z0:current.z1;
      if (nodeAt(point,z)?.junction) break;
      const options=(backwards?end:start).get(nodeKey(point,z)) ?? [];
      const candidates=options.filter(f=>f!==current&&!picked.has(f)&&sameLayout(current,f));
      if (candidates.length!==1) break;
      current=candidates[0]; picked.add(current);
    }
  }
  return [...picked];
}

// Snap a selected range to a real junction, so a lane that visually reaches
// an intersection also reaches its graph node. Keep ordinary cut points exact.
export function snapLaneRangeToJunction(section, graph, hit, radius) {
  let best = null;
  for (const span of section) for (const [point, z] of [[span.a,span.z0],[span.b,span.z1]]) {
    const node = graph.nodes.get(nodeKey(point,z));
    if (!node?.junction) continue;
    const d = distance(hit.point,point);
    if (d <= radius && (!best || d < best.d)) best={road:span.road,point:{...point},d,snapped:true};
  }
  return best ?? hit;
}

// Restrict an editable section to the part between two hits. Distances are
// measured along its segments, so dragging works on curves as well as lines.
export function laneRange(section, start, end) {
  if (!section.length) return [];
  const starts=new Map(),ends=new Map();
  for(const span of section){starts.set(nodeKey(span.a,span.z0),span);ends.set(nodeKey(span.b,span.z1),span)}
  const first=section.find(span=>!ends.has(nodeKey(span.a,span.z0)))??section[0];
  const ordered=[],seen=new Set();let current=first,total=0;
  while(current&&!seen.has(current)){
    seen.add(current);let length=distance(current.a,current.b);ordered.push({span:current,start:total,length});total+=length;
    current=starts.get(nodeKey(current.b,current.z1));
  }
  if(ordered.length!==section.length||!total)return [];
  const station=hit=>{
    const t=projection(hit.point,hit.road.a,hit.road.b).t;
    const item=ordered.find(({span})=>span.road.id===hit.road.id&&t>=span.lo-1e-6&&t<=span.hi+1e-6);
    return item?item.start+item.length*Math.max(0,Math.min(1,(t-item.span.lo)/(item.span.hi-item.span.lo))):null;
  };
  const a=station(start),b=station(end);if(a===null||b===null)return [];
  let lo=Math.min(a,b),hi=Math.max(a,b),closed=nodeKey(first.a,first.z0)===nodeKey(ordered.at(-1).span.b,ordered.at(-1).span.z1);
  const ranges=closed&&hi-lo>total/2?[[0,lo],[hi,total]]:[[lo,hi]],result=[];
  for(const item of ordered)for(const [rangeStart,rangeEnd] of ranges){
    let left=Math.max(item.start,rangeStart),right=Math.min(item.start+item.length,rangeEnd);
    if(right-left<1e-4)continue;
    const s=item.span,spanT=f=>s.lo+(s.hi-s.lo)*(f-item.start)/item.length,from=spanT(left),to=spanT(right);
    const at=t=>({x:s.road.a.x+(s.road.b.x-s.road.a.x)*t,y:s.road.a.y+(s.road.b.y-s.road.a.y)*t});
    const z=t=>(s.road.z??0)+((s.road.zEnd??s.road.z??0)-(s.road.z??0))*t;
    result.push({road:s.road,lo:from,hi:to,a:at(from),b:at(to),z0:z(from),z1:z(to)});
  }
  return result;
}

// Split only the segments touched by a section. Preserve untouched road IDs;
// segments with a changed layout receive a common group for continuous drawing.
export function applyLaneSection(world, section, forward, reverse, makeId) {
  const byRoad=new Map();
  for (const span of section) {
    if (!byRoad.has(span.road.id)) byRoad.set(span.road.id,[]);
    byRoad.get(span.road.id).push(span);
  }
  const result=[];
  for (const road of world.roads) {
    const spans=byRoad.get(road.id);
    if (!spans) { result.push(road); continue; }
    const cuts=[0,1,...spans.flatMap(s=>[s.lo,s.hi])].sort((a,b)=>a-b)
      .filter((v,i,arr)=>i===0||v-arr[i-1]>1e-6);
    for (let i=1;i<cuts.length;i++) {
      const lo=cuts[i-1],hi=cuts[i],mid=(lo+hi)/2;
      const changed=spans.some(s=>mid>=s.lo-1e-6&&mid<=s.hi+1e-6);
      const at=t=>({x:road.a.x+(road.b.x-road.a.x)*t,y:road.a.y+(road.b.y-road.a.y)*t});
      const z=t=>(road.z??0)+((road.zEnd??road.z??0)-(road.z??0))*t;
      const piece={...road,id:i===1?road.id:makeId(),groupId:road.groupId??road.id,a:at(lo),b:at(hi),z:z(lo),zEnd:z(hi)};
      if(changed){piece.laneAtoB=forward;piece.laneBtoA=reverse;piece.lanes=Math.max(forward,reverse);piece.oneway=!forward||!reverse;}
      result.push(piece);
    }
  }
  world.roads=result;
}

// Only widening can introduce a new terrain overlap; allow narrowing roads
// that were already authored close to terrain.
export function validateLaneSection(world, section, forward, reverse) {
  const lanes=forward+reverse,clearance=(14+8*lanes)/2;
  for(const span of section){
    const road=span.road,oldForward=road.laneAtoB??road.lanes??1,oldReverse=road.laneBtoA??(road.oneway?0:road.lanes??1);
    if(lanes<=oldForward+oldReverse||distance(span.a,span.b)<2)continue;
    const error=validateRoad(world,span.a,span.b,span.z0,span.z1,clearance);
    if(error)return error;
  }
  return null;
}

// Lane numbers run outward from the center divider in each travel direction.
// Screen-space road endpoints let the same hit test work on elevated roads.
export function pickLane(road, point, screenA, screenB, adding=false) {
  const dx=screenB.x-screenA.x,dy=screenB.y-screenA.y,len=Math.hypot(dx,dy)||1;
  const q=projection(point,screenA,screenB),normal={x:-dy/len,y:dx/len};
  const offset=(point.x-q.x)*normal.x+(point.y-q.y)*normal.y;
  const divider=laneDivider(road),direction=offset>=divider?0:1,count=laneCount(road,direction);
  if(!adding&&count===0)return null;
  const index=Math.max(0,Math.min(adding?count:count-1,Math.floor(Math.abs(offset-divider)/LANE_WIDTH)));
  return {direction,index,divider,normal,count};
}

// Lane rules refer to graph edges. Remap them after road pieces are split and
// shift indices on the edited side, so the selected lane—not merely the outer
// lane—is the one removed from existing turn rules.
export function remapLaneConnections(oldGraph,newGraph,junctions,section,edit=null){
  const spans=new Map();for(const part of section){if(!spans.has(part.road.id))spans.set(part.road.id,[]);spans.get(part.road.id).push(part)}
  for(const [nodeId,cfg] of Object.entries(junctions)){
    if(!Array.isArray(cfg.connectors))continue;
    const node=newGraph.nodes.get(nodeId);
    const find=(old,incoming)=>{
      if(!node||!old)return null;
      const candidates=incoming?node.incoming:node.out,exact=candidates.find(e=>e.id===old.id);
      if(exact)return exact;
      const a=oldGraph.nodes.get(old.from),b=oldGraph.nodes.get(old.to),oldRoad=oldGraph.roadById.get(old.road);
      if(!a||!b||!oldRoad)return null;
      const matches=candidates.filter(edge=>{
        const x=newGraph.nodes.get(edge.from),y=newGraph.nodes.get(edge.to);if(!x||!y||edge.direction!==old.direction)return false;
        const near=incoming?y:x,far=incoming?x:y,t=Math.min(.5,distance(a,b)*.5/(edge.length||1));
        const sample={x:near.x+(far.x-near.x)*t,y:near.y+(far.y-near.y)*t};
        return distance(sample,projection(sample,a,b))<.05;
      });
      return matches.find(e=>e.road===old.road)||matches.find(e=>{const r=newGraph.roadById.get(e.road);return (r?.groupId??r?.id)===(oldRoad.groupId??oldRoad.id)})||null;
    };
    const shift=(old,next,lane,incoming)=>{
      if(!edit||!old||!next||old.direction!==edit.direction||!spans.has(old.road))return lane;
      const near=newGraph.nodes.get(incoming?next.to:next.from),far=newGraph.nodes.get(incoming?next.from:next.to),road=oldGraph.roadById.get(old.road);
      if(!near||!far||!road)return lane;
      const f=Math.min(.1,.05/(next.length||1)),sample={x:near.x+(far.x-near.x)*f,y:near.y+(far.y-near.y)*f},t=projection(sample,road.a,road.b).t;
      if(!spans.get(old.road).some(s=>t>=s.lo-1e-8&&t<=s.hi+1e-8))return lane;
      if(edit.action==='remove'){if(lane===edit.index)return null;return lane>edit.index?lane-1:lane}
      return lane>=edit.index?lane+1:lane;
    };
    cfg.connectors=cfg.connectors.flatMap(key=>{
      const m=key.match(/^(.*)#(\d+)>(.*)#(\d+)$/);if(!m)return [];
      const oldIn=oldGraph.edgeById.get(m[1]),oldOut=oldGraph.edgeById.get(m[3]),a=find(oldIn,true),b=find(oldOut,false),la=shift(oldIn,a,+m[2],true),lb=shift(oldOut,b,+m[4],false);
      return a&&b&&la!==null&&lb!==null&&la<a.lanes&&lb<b.lanes?[`${a.id}#${la}>${b.id}#${lb}`]:[];
    });
  }
}
