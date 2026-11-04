import { distance, projection } from './model.js';

const lanes=road=>(road.laneAtoB??road.lanes??1)+(road.laneBtoA??(road.oneway?0:road.lanes??1));

// Orient a building's frontage towards the same ground road used for access.
export function buildingAccess(building,roads,maxDistance=100){
 if(building.parkingDisconnected)return null;
 let best=null;
 for(const road of roads){
  if(building.accessRoadId&&road.id!==building.accessRoadId)continue;
  if((road.z??0)!==0||(road.zEnd??road.z??0)!==0)continue;
  const anchor=building.accessPoint||building,point=projection(anchor,road.a,road.b),d=distance(anchor,point);
  if(d<maxDistance&&(!best||d<best.distance))best={road,point,distance:d};
 }
 if(!best)return null;
 const {road,point,distance:d}=best,angle=Math.atan2(road.b.y-road.a.y,road.b.x-road.a.x);
 const nx=-Math.sin(angle),ny=Math.cos(angle),side=(point.x-building.x)*nx+(point.y-building.y)*ny>=0?1:-1;
 const anchor=building.accessPoint||building,dx=d?(anchor.x-point.x)/d:nx,dy=d?(anchor.y-point.y)/d:ny;
 const edge=(14+lanes(road)*8)/2-1,scale=building.scale||1;
 const depth=(building.type==='home'?8.5:building.type==='shop'?13.5:building.type==='parking'?22:14.5)*scale;
 return {...best,angle,side,dx,dy,edge,depth,
  start:{x:point.x+dx*edge,y:point.y+dy*edge},
  end:building.accessPoint||{x:building.x-dx*depth,y:building.y-dy*depth},
  drivewayLength:Math.max(0,d-edge-(building.accessPoint?0:depth))};
}

export function buildingVariant(id){
 let hash=2166136261;
 for(const char of String(id))hash=Math.imul(hash^char.charCodeAt(0),16777619);
 return hash>>>0;
}

// Left and right are relative to the road's drawn direction, in map coordinates.
export function roadsideSide(road, point){
 const cross=(road.b.x-road.a.x)*(point.y-road.a.y)-(road.b.y-road.a.y)*(point.x-road.a.x);
 return cross<0?'left':'right';
}
export function roadsideAllowed(building, roads){
 const access=buildingAccess(building, roads);
 return !access || !!((access.road.roadside??3) & (roadsideSide(access.road,building)==='left'?2:1));
}
export function affectedBuildings(buildings, roads, selectedIds, mode){
 return buildings.filter(building=>{
  const access=buildingAccess(building,roads);
  if(!access || !selectedIds.has(access.road.id))return false;
  const bit=roadsideSide(access.road,building)==='left'?2:1;
  return ((access.road.roadside??3)&bit)!==0 && (mode&bit)===0;
 });
}
