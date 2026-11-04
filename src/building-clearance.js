import {polygonsOverlap} from './parking-layout.js';
import {laneCount} from './lane-geometry.js';
const sizes={home:[30,17],shop:[42,27],industry:[48,29],parking:[68,40]};
const dot=(a,b)=>a.x*b.x+a.y*b.y;
function distanceToSegment(point,a,b){const dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy,t=l?Math.max(0,Math.min(1,((point.x-a.x)*dx+(point.y-a.y)*dy)/l)):0;return Math.hypot(point.x-a.x-dx*t,point.y-a.y-dy*t)}
function rectangle(x,y,angle,halfWidth,halfHeight){return {x,y,u:{x:Math.cos(angle),y:Math.sin(angle)},v:{x:-Math.sin(angle),y:Math.cos(angle)},halfWidth,halfHeight}}
// Match the building renderer's orientation and scaled dimensions.
export function buildingFootprint(building,roads){
 if(building.polygon?.length>=3)return {points:building.polygon};
 let nearest=null;for(const road of roads){if((road.z??0)!==0||(road.zEnd??road.z??0)!==0)continue;const d=distanceToSegment(building,road.a,road.b);if(d<100&&(!nearest||d<nearest.distance))nearest={road,distance:d}}
 const angle=nearest?Math.atan2(nearest.road.b.y-nearest.road.a.y,nearest.road.b.x-nearest.road.a.x):0;
 const [width,height]=sizes[building.type]||sizes.industry,scale=building.scale||1;
 return rectangle(building.x,building.y,angle,width*scale/2,height*scale/2);
}
export function footprintsOverlap(a,b){
 if(a.points||b.points){const points=r=>r.points||[[-1,-1],[1,-1],[1,1],[-1,1]].map(([x,y])=>({x:r.x+x*r.halfWidth*r.u.x+y*r.halfHeight*r.v.x,y:r.y+x*r.halfWidth*r.u.y+y*r.halfHeight*r.v.y}));return polygonsOverlap(points(a),points(b))}
 const delta={x:b.x-a.x,y:b.y-a.y};
 for(const axis of [a.u,a.v,b.u,b.v]){
  const ra=a.halfWidth*Math.abs(dot(a.u,axis))+a.halfHeight*Math.abs(dot(a.v,axis));
  const rb=b.halfWidth*Math.abs(dot(b.u,axis))+b.halfHeight*Math.abs(dot(b.v,axis));
  if(Math.abs(dot(delta,axis))>=ra+rb-1e-6)return false;
 }
 return true;
}
function roadFootprint(a,b,half){const dx=b.x-a.x,dy=b.y-a.y;return rectangle((a.x+b.x)/2,(a.y+b.y)/2,Math.atan2(dy,dx),Math.hypot(dx,dy)/2,half)}
// Geometry edits replace road arrays; building edits replace or resize their
// array. Reuse orientations during repeated construction-preview frames.
const cachedFootprints=new WeakMap();
export function worldBuildingFootprints(world){
 const old=cachedFootprints.get(world);
 if(old&&old.roads===world.roads&&old.roadCount===world.roads.length&&old.buildings===world.buildings&&old.buildingCount===world.buildings.length)return old.shapes;
 const shapes=world.buildings.map(building=>buildingFootprint(building,world.roads));
 cachedFootprints.set(world,{roads:world.roads,roadCount:world.roads.length,buildings:world.buildings,buildingCount:world.buildings.length,shapes});return shapes;
}
export function roadOverlapsBuildings(world,a,b,z0,z1,halfWidth,shapes=worldBuildingFootprints(world)){
 if(z0>=.85&&z1>=.85)return false;
 // Only the ground-level portion can collide with buildings.
 let lo=0,hi=1;if(z0<.85&&z1>=.85)hi=(.85-z0)/(z1-z0);else if(z0>=.85&&z1<.85)lo=(.85-z0)/(z1-z0);
 const point=t=>({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t}),surface=roadFootprint(point(lo),point(hi),halfWidth);
 return shapes.some(shape=>footprintsOverlap(surface,shape));
}
export function buildingOverlapsWorld(world,building){
 const shape=buildingFootprint(building,world.roads);
 if(worldBuildingFootprints(world).some(other=>footprintsOverlap(shape,other)))return true;
 return world.roads.some(road=>{
  const z0=road.z??0,z1=road.zEnd??z0;if(z0>=.85&&z1>=.85)return false;
  let lo=0,hi=1;if(z0<.85&&z1>=.85)hi=(.85-z0)/(z1-z0);else if(z0>=.85&&z1<.85)lo=(.85-z0)/(z1-z0);
  const point=t=>({x:road.a.x+(road.b.x-road.a.x)*t,y:road.a.y+(road.b.y-road.a.y)*t});
  return footprintsOverlap(shape,roadFootprint(point(lo),point(hi),(14+8*(laneCount(road,0)+laneCount(road,1)))/2));
 });
}
