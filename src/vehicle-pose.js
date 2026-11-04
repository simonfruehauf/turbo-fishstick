// Render a continuous lane-to-lane turn across an entire junction approach,
// even when the underlying road is split into many short pieces.
import { travelLaneCenter } from './lane-geometry.js';

function sample(graph,edge,distance,lane){
 const a=graph.nodes.get(edge.from),b=graph.nodes.get(edge.to),road=graph.roadById.get(edge.road);
 if(!a||!b||!road||lane>=edge.lanes)return null;
 const t=Math.max(0,Math.min(1,distance/edge.length));
 const ax=a.x,ay=a.y-a.z*28,bx=b.x,by=b.y-b.z*28,dx=bx-ax,dy=by-ay,length=Math.hypot(dx,dy);
 if(length<.001)return null;
 const hx=dx/length,hy=dy/length,offset=travelLaneCenter(road,edge.direction,lane);
 return {x:ax+dx*t-hy*offset,y:ay+dy*t+hx*offset,hx,hy};
}

function available(graph,path,index,backward){
 let sum=0;
 for(let i=index;i>=0&&i<path.length;i+=backward?-1:1){
  const edge=path[i];sum+=edge.length;
  const node=graph.nodes.get(backward?edge.from:edge.to);
  if(node?.junction)break;
 }
 return sum;
}

function sampleFromBoundary(graph,path,boundary,backward,distance,lane){
 for(let i=backward?boundary:boundary+1;i>=0&&i<path.length;i+=backward?-1:1){
  const edge=path[i];
  if(distance<=edge.length+1e-6)return sample(graph,edge,backward?edge.length-distance:distance,lane);
  distance-=edge.length;
 }
 return null;
}

export function junctionVehiclePose(graph,path,index,position,lane){
 if(!path[index]||path.length<2)return null;
 const maxRadius=16;
 let nearest=null,dist=path[index].length-position;
 for(let i=index;i<path.length-1&&dist<=maxRadius;i++){
  if(graph.nodes.get(path[i].to)?.junction){nearest={boundary:i,distance:-dist};break}
  dist+=path[i+1].length;
 }
 dist=position;
 for(let i=index;i>0&&dist<=maxRadius;i--){
  if(graph.nodes.get(path[i].from)?.junction){if(!nearest||dist<Math.abs(nearest.distance))nearest={boundary:i-1,distance:dist};break}
  dist+=path[i-1].length;
 }
 if(!nearest)return null;
 const {boundary,distance}=nearest,incoming=path[boundary],outgoing=path[boundary+1];
 const inLane=incoming.exitLane??incoming.lane??0,outLane=outgoing.lane??inLane;
 if(lane!==(index<=boundary?inLane:outLane))return null;
 const radius=Math.min(maxRadius,available(graph,path,boundary,true)/2,available(graph,path,boundary+1,false)/2);
 if(radius<3||Math.abs(distance)>radius)return null;
 const p0=sampleFromBoundary(graph,path,boundary,true,radius,inLane),p3=sampleFromBoundary(graph,path,boundary,false,radius,outLane);
 if(!p0||!p3)return null;
 const handle=radius*.65,p1={x:p0.x+p0.hx*handle,y:p0.y+p0.hy*handle},p2={x:p3.x-p3.hx*handle,y:p3.y-p3.hy*handle};
 const t=(distance+radius)/(2*radius),u=1-t;
 const x=u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x;
 const y=u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y;
 const dx=3*(u*u*(p1.x-p0.x)+2*u*t*(p2.x-p1.x)+t*t*(p3.x-p2.x));
 const dy=3*(u*u*(p1.y-p0.y)+2*u*t*(p2.y-p1.y)+t*t*(p3.y-p2.y));
 const length=Math.hypot(dx,dy);if(length<.001)return null;
 return {x,y,dx:dx/length,dy:dy/length};
}
