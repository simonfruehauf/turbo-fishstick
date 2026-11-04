import {laneCount} from './lane-geometry.js';

const caches=new WeakMap();
const halfWidth=road=>7+4*(laneCount(road,0)+laneCount(road,1));
const projected=node=>({x:node.x,y:node.y-node.z*28});

export function flareHalfWidth(flare,distance){
 const t=Math.max(0,Math.min(1,distance/flare.reach)),ease=t*t*(3-2*t);
 return flare.wide+(flare.narrow-flare.wide)*ease;
}

function traceApproach(graph,node,arm,reach){
 const points=[projected(node)];let previous=node,current=arm.other,road=arm.road,walked=0,widthAtSetback=arm.width;
 const setback=(node.width??30)/2+12;
 for(let i=0;current&&i<100;i++){
  if(Math.abs(current.z-node.z)>.001)break;
  const a=points.at(-1),b=projected(current),dx=b.x-a.x,dy=b.y-a.y,length=Math.hypot(dx,dy);if(length<.001)break;
  if(walked<=setback)widthAtSetback=halfWidth(road);
  points.push(b);walked+=length;
  if(current.junction||walked>=reach)return {points,length:walked,junction:current.junction,widthAtSetback};
  const choices=new Map();
  for(const edge of [...current.incoming,...current.out]){
   const next=graph.nodes.get(edge.from===current.id?edge.to:edge.from),candidate=graph.roadById.get(edge.road);
   if(!next||!candidate||candidate.roundabout||next.id===previous.id||Math.abs(next.z-node.z)>.001)continue;
   const nx=next.x-current.x,ny=next.y-current.y,nl=Math.hypot(nx,ny);
   if(nl>.001&&(dx*nx+dy*ny)/(length*nl)>.7)choices.set(next.id,{next,road:candidate});
  }
  if(choices.size!==1)break;
  const continuation=choices.values().next().value;previous=current;current=continuation.next;road=continuation.road;
 }
 return {points,length:walked,junction:false,widthAtSetback};
}

// Widen only the narrow approach: a smooth flare meets the wider carriageway
// at the node, rather than leaving a perpendicular step opposite a T's branch.
// Filling a taper across both arms would leave that step inside the road union.
export function junctionWidthFlares(graph,node){
 if(!node.junction||node.mergeGroups?.size)return [];
 let cache=caches.get(graph);if(!cache)caches.set(graph,cache=new Map());
 if(cache.has(node.id))return cache.get(node.id);
 const arms=new Map(),result=[];
 for(const edge of [...node.incoming,...node.out]){
  const id=edge.from===node.id?edge.to:edge.from,other=graph.nodes.get(id),road=graph.roadById.get(edge.road);
  if(!other||!road||arms.has(id))continue;
  if(road.roundabout){cache.set(node.id,result);return result}
  const dx=other.x-node.x,dy=other.y-node.y,length=Math.hypot(dx,dy);if(length<.001)continue;
  arms.set(id,{other,road,group:road.groupId??road.id,dx:dx/length,dy:dy/length,width:halfWidth(road)});
 }
 for(const arm of arms.values())arm.trace=traceApproach(graph,node,arm,80);
 for(const arm of arms.values()){
  const opposite=[...arms.values()].filter(other=>other!==arm&&arm.dx*other.dx+arm.dy*other.dy<-.94).sort((a,b)=>(arm.dx*a.dx+arm.dy*a.dy)-(arm.dx*b.dx+arm.dy*b.dy))[0];
  if(!opposite)continue;
  const narrow=Math.min(arm.width,arm.trace.widthAtSetback),wide=Math.max(arm.width,arm.trace.widthAtSetback,opposite.width,opposite.trace.widthAtSetback);
  if(wide<=narrow+.01)continue;
  const desired=Math.min(80,Math.max(32,wide+(wide-narrow)*8)),trace=arm.trace;
  const reach=Math.min(desired,trace.length*(trace.junction ? .45 : .8));if(reach<8)continue;
  const flare={otherId:arm.other.id,narrow,wide,reach,sections:[]};
  // Sample the real approach, including short curve/attachment pieces. The
  // smoothstep has zero width slope at each end, so no new curb kinks appear.
  let segment=1,along=0;
  for(let i=0,count=Math.max(8,Math.ceil(reach/3));i<=count;i++){
   const distance=reach*i/count;
   while(segment<trace.points.length-1&&along+Math.hypot(trace.points[segment].x-trace.points[segment-1].x,trace.points[segment].y-trace.points[segment-1].y)<distance){along+=Math.hypot(trace.points[segment].x-trace.points[segment-1].x,trace.points[segment].y-trace.points[segment-1].y);segment++}
   const a=trace.points[segment-1],b=trace.points[segment],length=Math.hypot(b.x-a.x,b.y-a.y),t=Math.min(1,(distance-along)/length);
   flare.sections.push({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,dx:(b.x-a.x)/length,dy:(b.y-a.y)/length,width:flareHalfWidth(flare,distance)});
  }
  result.push(flare);
 }
 cache.set(node.id,result);return result;
}
