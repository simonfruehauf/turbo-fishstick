import {travelLaneCenter} from './lane-geometry.js';
import {junctionVehiclePose} from './vehicle-pose.js';
import {pathMetrics} from './traffic-rules.js';

function lanePose(graph,edge,position,lane){
 const a=graph.nodes.get(edge.from),b=graph.nodes.get(edge.to),road=graph.roadById.get(edge.road);
 if(!a||!b||!road)return null;
 const t=Math.max(0,Math.min(1,position/edge.length)),dx=b.x-a.x,dy=b.y-a.y,len=Math.hypot(dx,dy)||1,off=travelLaneCenter(road,edge.direction,lane);
 return {x:a.x+dx*t-dy/len*off,y:a.y+dy*t+dx/len*off-(a.z+(b.z-a.z)*t)*28,z:a.z+(b.z-a.z)*t};
}

// Sample the actual remaining lane itinerary, starting at the selected car.
// The renderer caches this until its route, lane, or edge changes.
export function vehicleRoutePoints(graph,car){
 const result=[],metrics=pathMetrics(graph,car.path);
 let runEnd=-1,runTravel=0,entryLane=0,goalLane=0,wait=0,changeProgress=0;
 for(let k=car.index;k<car.path.length;k++){
  const edge=car.path[k],start=k===car.index?Math.max(0,car.pos):0;
  if(k>runEnd){
   runEnd=metrics.runEnds[k];runTravel=0;
   const active=k===car.index&&car.laneChange;
   entryLane=active?active.from:k===car.index?car.lane??edge.lane??0:edge.lane??0;
   goalLane=edge.exitLane??entryLane;
   changeProgress=active?active.distance:0;
   const runLength=metrics.starts[runEnd+1]-metrics.starts[k]-start;
   wait=active?0:Math.max(0,runLength-(90*Math.abs(goalLane-entryLane)+45));
  }
  const count=Math.max(1,Math.ceil((edge.length-start)/8));
  for(let i=0;i<=count;i++){
   const position=start+(edge.length-start)*i/count;
   const steps=Math.max(0,Math.min(Math.abs(goalLane-entryLane),(changeProgress+runTravel+position-start-wait)/32));
   const whole=Math.floor(steps),fraction=steps-whole;
   const visualLane=entryLane+Math.sign(goalLane-entryLane)*(whole+fraction*fraction*(3-2*fraction));
   const pose=lanePose(graph,edge,position,visualLane);if(!pose)continue;
   const turn=Number.isInteger(visualLane)&&junctionVehiclePose(graph,car.path,k,position,visualLane);
   result.push({...pose,...turn,edgeIndex:k,position});
  }
  runTravel+=edge.length-start;
 }
 return result;
}

export function departureLanePoint(graph,trip){
 const edge=trip?.path?.[0];return edge?lanePose(graph,edge,0,edge.lane??0):null;
}

// Keep arrival and departure anchors separate. A rule edit can choose a new
// departure lane, but must never replay the original arrival across the map.
export function parkingAnimation(car,time){
 const arriving=time-car.arrived<3;
 const leaving=!arriving&&car.departStarted!=null&&!!car.nextTrip;
 const motionTime=car.departurePausedAt??time;
 const progress=arriving?(time-car.arrived)/3:leaving?1-(motionTime-car.departStarted)/3:1;
 return {leaving,progress:Math.max(0,Math.min(1,progress))};
}
