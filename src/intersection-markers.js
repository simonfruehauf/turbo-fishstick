import {travelLaneCenter} from './lane-geometry.js';

// Editor handles fan out around a junction. Their leader lines retain the
// physical lane anchors, while the clickable circles stay 16 screen px apart.
export function intersectionMarkers(graph,nodeId,zoom=1){
 const node=graph.nodes.get(nodeId);if(!node)return [];
 const markers=[],tau=Math.PI*2;
 for(const edge of [...node.incoming,...node.out]){
  const incoming=edge.to===nodeId,road=graph.roadById.get(edge.road);if(!road)continue;
  const sign=incoming?-1:1,reach=node.stopDistance??28;
  for(let lane=0;lane<edge.lanes;lane++){
   const off=travelLaneCenter(road,edge.direction,lane),x=sign*edge.hx*reach-edge.hy*off,y=sign*edge.hy*reach+edge.hx*off;
   markers.push({edge,lane,incoming,anchorX:node.x+x,anchorY:node.y-node.z*28+y,angle:(Math.atan2(y,x)+tau)%tau});
  }
 }
 if(!markers.length)return markers;
 markers.sort((a,b)=>a.angle-b.angle||a.edge.trafficIndex-b.edge.trafficIndex||a.lane-b.lane);
 const gap=16/zoom,radius=Math.max((node.width??30)/2+22,52/zoom,markers.length*gap/tau*1.25),minAngle=2*Math.asin(gap/(2*radius));
 const extras=markers.map((m,i)=>Math.max(0,((markers[(i+1)%markers.length].angle-m.angle+tau)%tau)-minAngle));
 const total=extras.reduce((a,b)=>a+b,0),spare=tau-markers.length*minAngle;
 let angle=markers[0].angle;
 const angles=markers.map((m,i)=>{const value=angle;angle+=minAngle+(total?spare*extras[i]/total:spare/markers.length);return value});
 const shift=markers.reduce((sum,m,i)=>sum+m.angle-angles[i],0)/markers.length;
 return markers.map((marker,i)=>({...marker,x:node.x+Math.cos(angles[i]+shift)*radius,y:node.y-node.z*28+Math.sin(angles[i]+shift)*radius}));
}

// Keep important junctions and endpoints visible, thinning intermediate curve
// handles at low zoom. Drawing and hit-testing consume the exact same list.
export function spacedNodeHandles(nodes,zoom=1){
 const result=[],gap=14/zoom;
 const ordered=[...nodes].sort((a,b)=>Number(!!b.junction||b.degree!==2)-Number(!!a.junction||a.degree!==2));
 const cells=new Map(),key=(x,y)=>`${x},${y}`;
 for(const node of ordered){
  const p={x:node.x,y:node.y-node.z*28},cx=Math.floor(p.x/gap),cy=Math.floor(p.y/gap);
  let close=false;
  for(let x=cx-1;x<=cx+1;x++)for(let y=cy-1;y<=cy+1;y++)for(const other of cells.get(key(x,y))||[])if(Math.hypot(p.x-other.x,p.y-other.y)<gap)close=true;
  if(close&&!node.junction&&node.degree===2)continue;
  const cellKey=key(cx,cy);if(!cells.has(cellKey))cells.set(cellKey,[]);cells.get(cellKey).push(p);result.push(node);
 }
 return result;
}
