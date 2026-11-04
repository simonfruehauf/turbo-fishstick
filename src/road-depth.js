const CELL=64;
const count=(road,dir)=>road[dir===0?'laneAtoB':'laneBtoA']??(road.oneway&&dir===1?0:road.lanes??1);
const key=(x,y)=>`${x},${y}`;

// The static renderer already splits roads at layer boundaries. Index their
// projected footprints so a car only checks nearby roads in its own layer.
export function buildRoadDepthIndex(layers){
 return layers.map(roads=>{
  const cells=new Map();
  for(const road of roads){
   const z0=road.z??0,z1=road.zEnd??z0;
   const a={x:road.a.x,y:road.a.y-z0*28},b={x:road.b.x,y:road.b.y-z1*28};
   const half=(14+8*(count(road,0)+count(road,1))+3)/2,margin=half+14;
   const item={road,a,b,z0,z1,half};
   const x0=Math.floor((Math.min(a.x,b.x)-margin)/CELL),x1=Math.floor((Math.max(a.x,b.x)+margin)/CELL);
   const y0=Math.floor((Math.min(a.y,b.y)-margin)/CELL),y1=Math.floor((Math.max(a.y,b.y)+margin)/CELL);
   for(let x=x0;x<=x1;x++)for(let y=y0;y<=y1;y++){
    const k=key(x,y);if(!cells.has(k))cells.set(k,[]);cells.get(k).push(item);
   }
  }
  return cells;
 });
}

// A ramp that joins this road should not hide cars at the mouth of the merge.
// Compare unprojected geometry and elevation so genuine overpasses still hide
// vehicles, including crossings that happen close to a ramp endpoint.
function nearConnectedMouth(item, ownRoad, point){
 if(!ownRoad)return false;
 const dx=ownRoad.b.x-ownRoad.a.x,dy=ownRoad.b.y-ownRoad.a.y,len2=dx*dx+dy*dy;
 if(len2<.001)return false;
 for(const [end,z] of [[item.road.a,item.z0],[item.road.b,item.z1]]){
  const t=((end.x-ownRoad.a.x)*dx+(end.y-ownRoad.a.y)*dy)/len2;
  if(t<-.001||t>1.001)continue;
  const x=ownRoad.a.x+dx*t,y=ownRoad.a.y+dy*t;
  const ownZ=(ownRoad.z??0)+((ownRoad.zEnd??ownRoad.z??0)-(ownRoad.z??0))*t;
  if(Math.hypot(x-end.x,y-end.y)>.1||Math.abs(z-ownZ)>.01)continue;
  const ownHalf=(14+8*(count(ownRoad,0)+count(ownRoad,1)))/2;
  if(Math.hypot(point.x-x,point.y-(y-z*28))<=item.half+ownHalf+12)return true;
 }
 return false;
}

// Nearby pieces of a curve or merging ramp can have different heights without
// being above the vehicle. Only mask a vehicle when the projected road
// centerlines actually cross at different heights.
function crossesAbove(item, road){
 const z0=road.z??0,z1=road.zEnd??z0;
 const a={x:road.a.x,y:road.a.y-z0*28},b={x:road.b.x,y:road.b.y-z1*28};
 const ax=b.x-a.x,ay=b.y-a.y,bx=item.b.x-item.a.x,by=item.b.y-item.a.y;
 const den=ax*by-ay*bx,alen=Math.hypot(ax,ay);
 if(alen<.001||Math.abs(den)<.001)return false;
 const dx=item.a.x-a.x,dy=item.a.y-a.y;
 const t=(dx*by-dy*bx)/den,u=(dx*ay-dy*ax)/den;
 const margin=(item.half+14)/alen;
 if(t < -margin||t > 1+margin||u<0||u>1)return false;
 const ownHeight=z0+(z1-z0)*Math.max(0,Math.min(1,t));
 return item.z0+(item.z1-item.z0)*u-ownHeight>.08;
}

export function roadsAboveVehicle(index,layer,point,height,ownRoad,ownGroup,ownRoadGeometry=null){
 const cells=index[layer];if(!cells)return [];
 const candidates=cells.get(key(Math.floor(point.x/CELL),Math.floor(point.y/CELL)))||[],result=[];
 for(const item of candidates){
  if(item.road.id===ownRoad)continue;
  if(nearConnectedMouth(item,ownRoadGeometry,point))continue;
  if(ownRoadGeometry&&!crossesAbove(item,ownRoadGeometry))continue;
  const dx=item.b.x-item.a.x,dy=item.b.y-item.a.y,len2=dx*dx+dy*dy;
  if(len2<.001)continue;
  const t=Math.max(0,Math.min(1,((point.x-item.a.x)*dx+(point.y-item.a.y)*dy)/len2));
  const x=item.a.x+dx*t,y=item.a.y+dy*t;
  if(Math.hypot(point.x-x,point.y-y)>item.half+12)continue;
  const roadHeight=item.z0+(item.z1-item.z0)*t,group=item.road.groupId??item.road.id;
  if(roadHeight-height<=(group===ownGroup ? .16 : .04))continue;
  result.push(item);
 }
 return result;
}

export function roadFootprint(item){
 const dx=item.b.x-item.a.x,dy=item.b.y-item.a.y,length=Math.hypot(dx,dy);
 if(length<.001)return [];
 const nx=-dy/length*item.half,ny=dx/length*item.half;
 return [
  {x:item.a.x+nx,y:item.a.y+ny},{x:item.b.x+nx,y:item.b.y+ny},
  {x:item.b.x-nx,y:item.b.y-ny},{x:item.a.x-nx,y:item.a.y-ny}
 ];
}

function projected(road){
 const z0=road.z??0,z1=road.zEnd??z0;
 return {road,z0,z1,a:{x:road.a.x,y:road.a.y-z0*28},b:{x:road.b.x,y:road.b.y-z1*28}};
}

// Find crossings whose roads have different heights but share a coarse layer.
// These need a local redraw so the higher road's border covers the lower road.
export function projectedOverpasses(layers){
 return layers.map(roads=>{
  const parts=roads.map(projected),patches=[],seen=new Set();
  for(let i=0;i<parts.length;i++)for(let j=i+1;j<parts.length;j++){
   const a=parts[i],b=parts[j];if(a.road.id===b.road.id)continue;
   if(Math.max(a.a.x,a.b.x)<Math.min(b.a.x,b.b.x)||Math.max(b.a.x,b.b.x)<Math.min(a.a.x,a.b.x)||Math.max(a.a.y,a.b.y)<Math.min(b.a.y,b.b.y)||Math.max(b.a.y,b.b.y)<Math.min(a.a.y,a.b.y))continue;
   const ax=a.b.x-a.a.x,ay=a.b.y-a.a.y,bx=b.b.x-b.a.x,by=b.b.y-b.a.y,den=ax*by-ay*bx;
   if(Math.abs(den)<1e-6)continue;
   const dx=b.a.x-a.a.x,dy=b.a.y-a.a.y,ta=(dx*by-dy*bx)/den,tb=(dx*ay-dy*ax)/den;
   if(ta<0||ta>1||tb<0||tb>1)continue;
   const za=a.z0+(a.z1-a.z0)*ta,zb=b.z0+(b.z1-b.z0)*tb;
   if(Math.abs(za-zb)<.08)continue;
   const upper=za>zb?a:b,lower=za>zb?b:a,high=Math.max(za,zb),low=Math.min(za,zb);
   const x=a.a.x+ax*ta,y=a.a.y+ay*ta,group=upper.road.groupId??upper.road.id,other=lower.road.groupId??lower.road.id;
   const id=`${group}|${other}|${Math.round(x)}|${Math.round(y)}`;if(seen.has(id))continue;seen.add(id);
   const sine=Math.abs(den)/(Math.hypot(ax,ay)*Math.hypot(bx,by));
   const upperWidth=14+8*(count(upper.road,0)+count(upper.road,1)),lowerWidth=14+8*(count(lower.road,0)+count(lower.road,1));
   patches.push({x,y,group,other,high,low,radius:Math.min(120,(upperWidth+lowerWidth+12)/(2*Math.max(.3,sine)))});
  }
  return patches.sort((a,b)=>a.high-b.high);
 });
}
