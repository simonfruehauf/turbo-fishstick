import {buildingOverlapsWorld,roadOverlapsBuildings} from './building-clearance.js';
import {laneCount} from './lane-geometry.js';
const project=(p,a,b)=>{let dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy,t=l?Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/l)):0;return {x:a.x+t*dx,y:a.y+t*dy}};
export function findHomeParking(world,home){
 let nearest=null;for(const road of world.roads){if((road.z??0)!==0||(road.zEnd??0)!==0)continue;const p=project(home,road.a,road.b),d=Math.hypot(home.x-p.x,home.y-p.y);if(d<100&&(!nearest||d<nearest.d))nearest={road,p,d}}
 if(!nearest||nearest.d<.01)return null;
 const {road,p,d}=nearest,nx=(home.x-p.x)/d,ny=(home.y-p.y)/d,tx=(road.b.x-road.a.x)/Math.hypot(road.b.x-road.a.x,road.b.y-road.a.y),ty=(road.b.y-road.a.y)/Math.hypot(road.b.x-road.a.x,road.b.y-road.a.y),edge=(14+8*(laneCount(road,0)+laneCount(road,1)))/2;
 const candidates=[{x:home.x-nx*14.5,y:home.y-ny*14.5,angle:Math.atan2(ny,nx),kind:'front'},...[-1,1].map(side=>({x:home.x+side*22*tx,y:home.y+side*22*ty,angle:Math.atan2(ty,tx),kind:'side'}))];
 for(const spot of candidates){let c=Math.cos(spot.angle),s=Math.sin(spot.angle),poly=[[-6,-4],[6,-4],[6,4],[-6,4]].map(([x,y])=>({x:spot.x+x*c-y*s,y:spot.y+x*s+y*c})),q=project(spot,road.a,road.b),dist=Math.hypot(spot.x-q.x,spot.y-q.y);if(dist<edge+6)continue;
  let otherWorld=spot.kind==='front'?{...world,buildings:world.buildings.filter(b=>b.id!==home.id)}:world;
  if(buildingOverlapsWorld(otherWorld,{type:'parking',x:spot.x,y:spot.y,polygon:poly}))continue;
  let dx=(spot.x-q.x)/dist,dy=(spot.y-q.y)/dist,start={x:q.x+dx*edge,y:q.y+dy*edge};if(roadOverlapsBuildings(otherWorld,start,{x:spot.x,y:spot.y},0,0,4))continue;
  return {...spot,roadId:road.id,roadPoint:q,start};
 }
 return null;
}
