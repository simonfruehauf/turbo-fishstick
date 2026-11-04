const distance=(a,b)=>Math.hypot(b.x-a.x,b.y-a.y);

function tangent(anchor,reference,fallback){
 const lane=anchor.laneAnchor;
 let vector=lane?{x:lane.normal.y,y:-lane.normal.x}:anchor.tangent;
 if(!vector)return null;
 let length=Math.hypot(vector.x,vector.y);if(length<1e-6)return null;
 vector={x:vector.x/length,y:vector.y/length};
 if(lane){if(lane.direction===1)vector={x:-vector.x,y:-vector.y};return vector}
 let dot=vector.x*reference.x+vector.y*reference.y;
 if(Math.abs(dot)<1e-6)dot=vector.x*fallback.x+vector.y*fallback.y;
 return dot<0?{x:-vector.x,y:-vector.y}:vector;
}

// Free curves keep the familiar quadratic shape. Snapped ends replace the
// relevant cubic handle with a tangent to the road or selected travel lane.
export function curveControls(start,handle,end,align=true){
 const c1={x:start.x+(handle.x-start.x)*2/3,y:start.y+(handle.y-start.y)*2/3};
 const c2={x:end.x+(handle.x-end.x)*2/3,y:end.y+(handle.y-end.y)*2/3};
 if(!align)return {c1,c2};
 const chord={x:end.x-start.x,y:end.y-start.y},span=Math.hypot(chord.x,chord.y);
 const first=tangent(start,{x:handle.x-start.x,y:handle.y-start.y},chord);
 const last=tangent(end,{x:end.x-handle.x,y:end.y-handle.y},chord);
 const magnitude=point=>Math.min(span*.75,Math.max(span*.18,distance(point,handle)*.6));
 if(first){let d=magnitude(start);c1.x=start.x+first.x*d;c1.y=start.y+first.y*d}
 if(last){let d=magnitude(end);c2.x=end.x-last.x*d;c2.y=end.y-last.y*d}
 return {c1,c2};
}

export function sampleCurve(start,handle,end,align=true){
 const {c1,c2}=curveControls(start,handle,end,align);
 const length=distance(start,c1)+distance(c1,c2)+distance(c2,end);
 const count=Math.max(4,Math.min(64,Math.ceil(length/6)));
 const points=Array.from({length:count+1},(_,i)=>{
  let t=i/count,u=1-t;
  return {x:u*u*u*start.x+3*u*u*t*c1.x+3*u*t*t*c2.x+t*t*t*end.x,y:u*u*u*start.y+3*u*u*t*c1.y+3*u*t*t*c2.y+t*t*t*end.y};
 });
 const result=[points[0]];
 for(let i=1;i<points.length-1;i++)if(distance(result.at(-1),points[i])>=3)result.push(points[i]);
 while(result.length>1&&distance(result.at(-1),end)<3)result.pop();
 result.push(points.at(-1));return result;
}

// Project the same sampled path and arc-length elevation used by construction.
export function projectCurve(points,z,zEnd){
 const lengths=[0];for(let i=1;i<points.length;i++)lengths.push(lengths.at(-1)+distance(points[i-1],points[i]));
 const total=lengths.at(-1);
 return points.map((p,i)=>({x:p.x,y:p.y-(z+(zEnd-z)*(total?lengths[i]/total:0))*28}));
}
