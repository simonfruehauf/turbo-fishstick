// Shared Catmull-Rom outline for terrain rendering and placement checks.
export function closedSplineSegments(points, scale=1, offset=0) {
  if(points.length<3)return [];
  const ox=typeof offset==='number'?offset:offset.x,oy=typeof offset==='number'?offset:offset.y;
  const cx=points.reduce((s,p)=>s+p.x,0)/points.length,cy=points.reduce((s,p)=>s+p.y,0)/points.length;
  const pts=points.map(p=>({x:cx+(p.x-cx)*scale+ox,y:cy+(p.y-cy)*scale+oy}));
  return pts.map((p1,i)=>{
    const p0=pts[(i-1+pts.length)%pts.length],p2=pts[(i+1)%pts.length],p3=pts[(i+2)%pts.length];
    return {start:p1,c1:{x:p1.x+(p2.x-p0.x)/6,y:p1.y+(p2.y-p0.y)/6},c2:{x:p2.x-(p3.x-p1.x)/6,y:p2.y-(p3.y-p1.y)/6},end:p2};
  });
}

export function sampleClosedSpline(points, scale=1) {
  const result=[];
  for(const {start,c1,c2,end} of closedSplineSegments(points,scale)){
    const steps=Math.max(4,Math.min(32,Math.ceil(Math.hypot(end.x-start.x,end.y-start.y)/10)));
    for(let i=0;i<steps;i++){
      const t=i/steps,u=1-t;
      result.push({x:u*u*u*start.x+3*u*u*t*c1.x+3*u*t*t*c2.x+t*t*t*end.x,y:u*u*u*start.y+3*u*u*t*c1.y+3*u*t*t*c2.y+t*t*t*end.y});
    }
  }
  return result;
}

// Recognize a complete circular road without rounding unrelated curved roads.
// Points may be unequally spaced, as long as they describe a closed ring.
export function circleFromClosedPath(points) {
  if (points.length < 7) return null;
  const first = points[0], last = points.at(-1);
  if (Math.hypot(first.x - last.x, first.y - last.y) > .1) return null;
  const unique = points.slice(0, -1);
  let second = unique.reduce((best, p) => Math.hypot(p.x-first.x,p.y-first.y) > Math.hypot(best.x-first.x,best.y-first.y) ? p : best, unique[0]);
  let third = unique.reduce((best, p) => Math.abs((second.x-first.x)*(p.y-first.y)-(second.y-first.y)*(p.x-first.x)) > Math.abs((second.x-first.x)*(best.y-first.y)-(second.y-first.y)*(best.x-first.x)) ? p : best, unique[0]);
  const d = 2 * (first.x * (second.y-third.y) + second.x * (third.y-first.y) + third.x * (first.y-second.y));
  if (Math.abs(d) < .001) return null;
  const a = first.x**2+first.y**2, b = second.x**2+second.y**2, c = third.x**2+third.y**2;
  const x = (a*(second.y-third.y)+b*(third.y-first.y)+c*(first.y-second.y))/d;
  const y = (a*(third.x-second.x)+b*(first.x-third.x)+c*(second.x-first.x))/d;
  const radius = Math.hypot(first.x-x, first.y-y);
  if (radius < 1 || unique.some(p => Math.abs(Math.hypot(p.x-x,p.y-y)-radius) > .1)) return null;
  return { x, y, radius };
}

export function pointOnCircularEdge(circle, a, b, t) {
  const start = Math.atan2(a.y-circle.y, a.x-circle.x);
  const end = Math.atan2(b.y-circle.y, b.x-circle.x);
  const delta = Math.atan2(Math.sin(end-start), Math.cos(end-start));
  const angle = start + delta*t;
  return { x: circle.x+Math.cos(angle)*circle.radius, y: circle.y+Math.sin(angle)*circle.radius };
}

// A positive offset follows the left normal of the original road path.
export function circularOffsetRadius(circle, a, b, offset) {
  const cross = (a.x-circle.x)*(b.y-circle.y)-(a.y-circle.y)*(b.x-circle.x);
  return circle.radius - Math.sign(cross)*offset;
}

// The stop marking covers only traffic approaching the node. Reverse lanes
// occupy the opposite side of a two-way road and must stay clear.
export function approachStopLine(node, other, incoming, reverse, junctionWidth) {
  const dx = node.x-other.x, dy = node.y-other.y, length = Math.hypot(dx,dy);
  if (length < .001 || incoming < 1) return null;
  const ux = dx/length, uy = dy/length, nx = -uy, ny = ux;
  const setback = junctionWidth/2+2;
  const center = {x:node.x-ux*setback, y:node.y-uy*setback};
  const half = (14+8*(incoming+reverse))/2-2;
  const from = reverse ? (reverse-incoming)*4 : -half;
  return {
    a:{x:center.x+nx*from,y:center.y+ny*from},
    b:{x:center.x+nx*half,y:center.y+ny*half}
  };
}

// Find where both edges of a curved slip road meet the outside edge of the
// main carriageway. The returned quadratic corners fill the concave seam.
export function mergeFillets(center, mainDirection, mainHalf, branchHalf, samples, radius=6) {
  const length=Math.hypot(mainDirection.x,mainDirection.y);
  if(length<.001||samples.length<3)return [];
  const t={x:mainDirection.x/length,y:mainDirection.y/length},n={x:-t.y,y:t.x};
  const dot=(a,b)=>a.x*b.x+a.y*b.y;
  const away=samples.slice(1).reduce((best,p)=>Math.abs(dot({x:p.x-center.x,y:p.y-center.y},n))>Math.abs(dot({x:best.x-center.x,y:best.y-center.y},n))?p:best,samples[1]);
  const side=Math.sign(dot({x:away.x-center.x,y:away.y-center.y},n));
  if(!side)return [];
  const corners=[];
  for(const edgeSide of [-1,1]){
    const border=samples.map((p,i)=>{
      const a=samples[Math.max(0,i-1)],b=samples[Math.min(samples.length-1,i+1)],dx=b.x-a.x,dy=b.y-a.y,l=Math.hypot(dx,dy)||1;
      return {x:p.x-dy/l*branchHalf*edgeSide,y:p.y+dx/l*branchHalf*edgeSide};
    });
    for(let i=1;i<border.length;i++){
      const a=border[i-1],b=border[i],da=dot({x:a.x-center.x,y:a.y-center.y},n)-side*mainHalf,db=dot({x:b.x-center.x,y:b.y-center.y},n)-side*mainHalf;
      if(da*side>0||db*side<0||Math.abs(db-da)<1e-6)continue;
      const u=-da/(db-da),point={x:a.x+(b.x-a.x)*u,y:a.y+(b.y-a.y)*u};
      const dx=b.x-a.x,dy=b.y-a.y,l=Math.hypot(dx,dy);if(l<.001)break;
      corners.push({point,out:{x:dx/l,y:dy/l},position:dot({x:point.x-center.x,y:point.y-center.y},t)});break;
    }
  }
  if(corners.length!==2)return [];
  corners.sort((a,b)=>a.position-b.position);
  return corners.map((c,i)=>({
    start:{x:c.point.x+t.x*radius*(i?1:-1),y:c.point.y+t.y*radius*(i?1:-1)},
    control:c.point,
    end:{x:c.point.x+c.out.x*radius,y:c.point.y+c.out.y*radius}
  }));
}
