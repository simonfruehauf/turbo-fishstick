// Procedural parking geometry shared by placement, rendering and capacity.
export function pointInPolygon(p,poly){let inside=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){const a=poly[i],b=poly[j];if(((a.y>p.y)!==(b.y>p.y))&&p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)inside=!inside}return inside}
const cross=(a,b,c)=>(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
function intersects(a,b,c,d){const ab=cross(a,b,c),ac=cross(a,b,d),cd=cross(c,d,a),ce=cross(c,d,b);if(Math.abs(ab)<1e-8&&Math.abs(ac)<1e-8){return Math.max(Math.min(a.x,b.x),Math.min(c.x,d.x))<=Math.min(Math.max(a.x,b.x),Math.max(c.x,d.x))+1e-8&&Math.max(Math.min(a.y,b.y),Math.min(c.y,d.y))<=Math.min(Math.max(a.y,b.y),Math.max(c.y,d.y))+1e-8}return ab*ac<=0&&cd*ce<=0}
export function polygonsOverlap(a,b){for(let i=0;i<a.length;i++)for(let j=0;j<b.length;j++)if(intersects(a[i],a[(i+1)%a.length],b[j],b[(j+1)%b.length]))return true;return pointInPolygon(a[0],b)||pointInPolygon(b[0],a)}
export function validParkingPolygon(poly){if(poly.length<3||poly.some(p=>!Number.isFinite(p.x)||!Number.isFinite(p.y)))return false;let area=0;for(let i=0;i<poly.length;i++){let a=poly[i],b=poly[(i+1)%poly.length];if(Math.hypot(a.x-b.x,a.y-b.y)<3)return false;area+=a.x*b.y-a.y*b.x;for(let j=i+1;j<poly.length;j++){if(j===i+1||(i===0&&j===poly.length-1))continue;if(intersects(a,b,poly[j],poly[(j+1)%poly.length]))return false}}return Math.abs(area)>200}
const slotCache=new WeakMap();
export function parkingSlots(building,angle=0){
 const cached=slotCache.get(building);if(cached&&cached.polygon===building.polygon&&cached.angle===angle&&cached.x===building.x&&cached.y===building.y)return cached.slots;const requestedAngle=angle;
 if(building.polygon){
  const poly=building.polygon,origin=building.accessPoint||{x:(poly[0].x+poly[1].x)/2,y:(poly[0].y+poly[1].y)/2};
  let dx=building.accessDirection?.x??building.x-origin.x,dy=building.accessDirection?.y??building.y-origin.y,len=Math.hypot(dx,dy);if(!len)return [];dx/=len;dy/=len;
  const at=(x,y)=>({x:origin.x+dx*x-dy*y,y:origin.y+dy*x+dx*y}),inside=(x,y)=>pointInPolygon(at(x,y),poly),maxDistance=Math.max(...poly.map(p=>(p.x-origin.x)*dx+(p.y-origin.y)*dy)),slots=[];
  // Parallel aisles are reached through a cross aisle near the entrance.
  const localY=poly.map(p=>-(p.x-origin.x)*dy+(p.y-origin.y)*dx),maxSide=Math.max(...localY.map(Math.abs)),branch=14;
  const connected=offset=>{if(offset===0)return true;let sign=Math.sign(offset);for(let y=0;Math.abs(y)<=Math.abs(offset);y+=sign*3)for(const x of [branch-6,branch,branch+6])if(!inside(x,y))return false;return true};
  const makeAisle=(offset,startX)=>{let found=[];if(!connected(offset))return found;for(let x=startX;x+4<maxDistance;x+=9){let clearAisle=true;for(let z=Math.max(1,x-9);z<=x+4;z+=3)if(!inside(z,offset-7)||!inside(z,offset)||!inside(z,offset+7)){clearAisle=false;break}if(!clearAisle)break;for(const side of [-1,1]){let clear=true;for(const xx of [x-4,x,x+4])for(const yy of [offset+side*8,offset+side*12,offset+side*17,offset+side*21,offset+side*26])if(!inside(xx,yy))clear=false;if(clear){const p=at(x,offset+side*17);found.push({...p,angle:Math.atan2(dy,dx)+Math.PI/2,width:8,length:18,aisleOffset:offset,aisleDistance:x,branchDistance:branch,side})}}}return found};
  let other=[];for(let offset=52;offset<maxSide+26;offset+=52)for(const side of [-1,1])other.push(...makeAisle(offset*side,23));
  slots.push(...makeAisle(0,other.length?23:5),...other);
  slotCache.set(building,{polygon:building.polygon,angle:requestedAngle,x:building.x,y:building.y,slots});return slots;
 }
 let poly=building.polygon;if(!poly){const c=Math.cos(angle),s=Math.sin(angle);poly=[[-34,-20],[34,-20],[34,20],[-34,20]].map(([x,y])=>({x:building.x+x*c-y*s,y:building.y+x*s+y*c}))}
 angle=building.polygon?Math.atan2(poly[1].y-poly[0].y,poly[1].x-poly[0].x):angle;const c=Math.cos(angle),s=Math.sin(angle),origin=poly[0],local=p=>({x:(p.x-origin.x)*c+(p.y-origin.y)*s,y:-(p.x-origin.x)*s+(p.y-origin.y)*c}),world=(x,y)=>({x:origin.x+x*c-y*s,y:origin.y+x*s+y*c}),localPoly=poly.map(local),xs=localPoly.map(p=>p.x),ys=localPoly.map(p=>p.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),slots=[];
 for(let y=minY+10,row=0;y+9<=maxY-1;y+=row++%2===1?32:20)for(let x=minX+5;x+4<=maxX-1;x+=9){let corners=[{x:x-4,y:y-9},{x:x+4,y:y-9},{x:x+4,y:y+9},{x:x-4,y:y+9}];if(corners.every(p=>pointInPolygon(p,localPoly))&&corners.every((p,i)=>{const q=corners[(i+1)%4];for(let n=1;n<4;n++)if(!pointInPolygon({x:p.x+(q.x-p.x)*n/4,y:p.y+(q.y-p.y)*n/4},localPoly))return false;return true})){ let p=world(x,y);slots.push({...p,angle:angle+Math.PI/2,width:8,length:18})}}
 slotCache.set(building,{polygon:building.polygon,angle:requestedAngle,x:building.x,y:building.y,slots});return slots;
}
