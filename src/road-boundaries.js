// Identify projected road overlaps that are parallel but not junctions. The
// resulting masks restore the boundary of the upper road without changing the
// road graph or drawing separators through connected intersections.
const lanes=(road,dir)=>road[dir===0?'laneAtoB':'laneBtoA']??(road.oneway&&dir===1?0:road.lanes??1);
const groupOf=road=>road.groupId??road.id;
const pairKey=(a,b)=>[a,b].sort().join('\u0000');
export function nearbyRoadBoundaries(roads,graph){
 const segments=roads.map(road=>{
  const z0=road.z??0,z1=road.zEnd??z0;
  const a={x:road.a.x,y:road.a.y-z0*28},b={x:road.b.x,y:road.b.y-z1*28};
  const dx=b.x-a.x,dy=b.y-a.y,length=Math.hypot(dx,dy);
  return {road,a,b,length,ux:dx/(length||1),uy:dy/(length||1),height:(z0+z1)/2,half:(14+8*(lanes(road,0)+lanes(road,1)))/2,group:groupOf(road)};
 });
 const joins=new Map();
 for(const node of graph.nodes.values()){
  if(node.degree<2||node.roads.size<2)continue;
  const ids=[...node.roads];for(let i=0;i<ids.length;i++)for(let j=i+1;j<ids.length;j++){
   const key=pairKey(ids[i],ids[j]);if(!joins.has(key))joins.set(key,[]);joins.get(key).push(node);
  }
 }
 const result=new Map();
 for(let i=0;i<segments.length;i++)for(let j=i+1;j<segments.length;j++){
  let a=segments[i],b=segments[j];
  if(a.group===b.group||a.length<.01||b.length<.01||Math.abs(a.height-b.height)>.35)continue;
  if(Math.abs(a.ux*b.ux+a.uy*b.uy)<.9)continue;
  if(b.height>a.height||(b.height===a.height&&b.group>a.group))[a,b]=[b,a];
  const dx=b.a.x-a.a.x,dy=b.a.y-a.a.y,nx=-a.uy,ny=a.ux;
  const across=Math.abs(dx*nx+dy*ny);
  if(across>=a.half+b.half||across<=Math.abs(a.half-b.half)+1)continue;
  const t0=dx*a.ux+dy*a.uy,t1=(b.b.x-a.a.x)*a.ux+(b.b.y-a.a.y)*a.uy;
  const lo=Math.max(0,Math.min(t0,t1)),hi=Math.min(a.length,Math.max(t0,t1));
  if(hi-lo<.2)continue;
  const x=a.a.x+a.ux*(lo+hi)/2,y=a.a.y+a.uy*(lo+hi)/2;
  const shared=joins.get(pairKey(a.group,b.group))||[];
  if(shared.some(n=>Math.hypot(n.x-x,n.y-(n.z??0)*28-y)<Math.max(a.half,b.half)+35))continue;
  // Draw at or above both pieces so the boundary survives layer compositing.
  const level=Math.min(2,Math.max(0,Math.ceil(Math.max(a.height,b.height)-1e-7)));
  const key=`${a.group}\u0000${b.group}\u0000${level}`;
  if(!result.has(key))result.set(key,{group:a.group,other:b.group,level,clips:[],joins:shared});
  result.get(key).clips.push({a:b.a,b:b.b,half:b.half});
 }
 return [...result.values()];
}
