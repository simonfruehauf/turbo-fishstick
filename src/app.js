import {junctionWidthFlares,flareHalfWidth} from './junction-width.js';
import {intersectionMarkers,spacedNodeHandles} from './intersection-markers.js';
import {vehicleRoutePoints,departureLanePoint,parkingAnimation} from './vehicle-route.js';
import {approachRule,turnType,pathMetrics} from './traffic-rules.js';
import {parkingSlots,pointInPolygon,validParkingPolygon} from './parking-layout.js';
import { buildingOverlapsWorld, roadOverlapsBuildings } from './building-clearance.js';
import { laneCount, laneDivider, laneBoundary, laneCenter, travelLaneCenter } from './lane-geometry.js';
import { nearbyRoadBoundaries } from './road-boundaries.js';
import { moveRoadNode, movableRoadNodes } from './road-node-move.js';
import { buildingAccess, buildingVariant, roadsideAllowed, affectedBuildings } from './building-layout.js';
import { connectionDistance } from './connection-hit.js';
import { sampleCurve, projectCurve } from './curve-design.js';
import { buildRoadDepthIndex, roadsAboveVehicle, roadFootprint, projectedOverpasses } from './road-depth.js';
import { remapConnectionsAfterBuild, connectRampToLanes } from './ramp-links.js';
import { junctionVehiclePose } from './vehicle-pose.js';
import { circleFromClosedPath, pointOnCircularEdge, circularOffsetRadius, approachStopLine, mergeFillets, closedSplineSegments } from './geometry.js';
import { laneSection, laneRange, applyLaneSection, validateLaneSection, pickLane, remapLaneConnections, snapLaneRangeToJunction } from './lane-sections.js';
import {SIZE,distance,projection,starterWorld,validWorld,migrate,Simulation,buildGraph,validateRoadPath,terrainAt,movementKey,defaultMovement,demoWorld,signalState} from './model.js';
const $=s=>document.querySelector(s),canvas=$('#map'),view=$('#view');let ctx=canvas.getContext('2d'),world;try{const last=localStorage.getItem('tiny-detour-last-map'),raw=last?localStorage.getItem(`tiny-detour-map-v2-${last}`):null;let w=JSON.parse(raw||localStorage.getItem('tiny-detour-v1'));world=validWorld(w)?migrate(w):demoWorld('starter')}catch{world=starterWorld()}world.demoId ||= 'starter';repairParkingEntrances(world);let sim=new Simulation(world),shapeType='straight',tool='select',selected=null,connectionSource=null,start=null,control=null,mouse=null,snapTarget=null,deleteHover=null,drag=null,cam={x:512,y:512,zoom:1},speed=1,paused=false,overlay=false,last=0,acc=0,history=[],buildingPreviewId=null,laneSelection=null,laneEndTarget=null,roadsideSelection=new Set(),roadsideHover=null,parkingCorners=[];
const colors={home:'#83bd6a',shop:'#639dd3',industry:'#d1a16c'};function save(){try{const data=JSON.stringify(world),id=world.demoId||'starter';localStorage.setItem(`tiny-detour-map-v2-${id}`,data);if(id==='starter')localStorage.setItem('tiny-detour-v1',data);localStorage.setItem('tiny-detour-last-map',id);const status=$('.saved');if(status)status.textContent='● Saved locally';return true}catch(error){const status=$('.saved');if(status)status.textContent='● Local save unavailable';console.warn('Could not save the map locally',error);return false}}function title(){ $('#cityName').textContent=world.name;$('#mapName').textContent=world.name.toUpperCase();let welcome=$('#welcomeName');if(welcome)welcome.textContent=`Welcome to ${world.name}` }function change(fn,graph=null){history.push(JSON.stringify(world));if(history.length>30)history.shift();fn();sim.rebuild(graph);selected=null;start=null;control=null;inspector();save()}function fit(){let r=view.getBoundingClientRect();cam={x:512,y:512,zoom:Math.min(r.width,r.height)*.87/SIZE}}function resize(){let r=view.getBoundingClientRect(),d=devicePixelRatio||1;canvas.width=r.width*d;canvas.height=r.height*d;ctx.setTransform(d,0,0,d,0,0)}function point(e){let r=canvas.getBoundingClientRect();return {x:(e.clientX-r.left-r.width/2)/cam.zoom+cam.x,y:(e.clientY-r.top-r.height/2)/cam.zoom+cam.y}}function snap(p,level=null){let junction=null,endpoint=null,segment=null,laneMode=tool==='road'&&shapeType==='ramp'&&(!start||control);if($('#geometrySnap').checked){for(const n of sim.graph.nodes.values()){if(level!==null&&Math.abs(n.z-level)>.001)continue;if(!n.junction&&n.degree>1)continue;let d=distance(p,{x:n.x,y:n.y-n.z*28});if(d>22/cam.zoom)continue;let candidate={x:n.x,y:n.y,z:n.z,d,kind:n.junction?'Junction snap':'Road endpoint'};if(!n.junction){let edge=n.out[0]||n.incoming[0],road=edge&&sim.graph.roadById.get(edge.road);if(road){let length=distance(road.a,road.b);candidate.tangent={x:(road.b.x-road.a.x)/length,y:(road.b.y-road.a.y)/length};if(laneMode){candidate.laneAnchor=laneAnchorAt(road,p);if(candidate.laneAnchor)candidate.kind=(candidate.laneAnchor.direction===0?'Forward':'Reverse')+' lane '+(candidate.laneAnchor.lane+1)+' endpoint'}}}if(n.junction){if(!junction||d<junction.d)junction=candidate}else if(!endpoint||d<endpoint.d)endpoint=candidate}for(const r of world.roads){let z=r.z??0;if(z!==(r.zEnd??z)||level!==null&&Math.abs(z-level)>.001)continue;let q=projection({x:p.x,y:p.y+z*28},r.a,r.b),d=distance(p,{x:q.x,y:q.y-z*28}),radius=laneMode?Math.max(14/cam.zoom,(14+8*totalLanes(r))/2):14/cam.zoom;if(d<radius){let laneAnchor=laneMode?laneAnchorAt(r,p):null,length=distance(r.a,r.b),tangent={x:(r.b.x-r.a.x)/length,y:(r.b.y-r.a.y)/length},laneDistance=laneAnchor?Math.abs((p.x-q.x)*laneAnchor.normal.x+(p.y-q.y)*laneAnchor.normal.y-laneAnchor.offset):d;if(!segment||d<segment.d-.05||(Math.abs(d-segment.d)<.05&&laneDistance<(segment.laneDistance??Infinity))){segment={x:q.x,y:q.y,z,d,laneDistance,kind:laneAnchor?(laneAnchor.direction===0?'Forward':'Reverse')+' lane '+(laneAnchor.lane+1)+' snap':'Road snap',laneAnchor,tangent}}}}}let best=junction||endpoint||segment,z=level??+$('#elevation').value;if(best)return {x:best.x,y:best.y,z:best.z,snapKind:best.kind,laneAnchor:best.laneAnchor,tangent:best.tangent};if(!$('#guides').checked)return {x:Math.max(0,Math.min(SIZE,p.x)),y:Math.max(0,Math.min(SIZE,p.y+z*28)),z,snapKind:'free'};let x=p.x,y=p.y+z*28,guides=[],gx=null,gy=null;for(const n of sim.graph.nodes.values()){if(Math.abs(n.z-z)>.001||(!n.junction&&n.degree>1))continue;let dx=Math.abs(n.x-x),dy=Math.abs(n.y-y);if(dx<8/cam.zoom&&(!gx||dx<gx.d))gx={v:n.x,d:dx};if(dy<8/cam.zoom&&(!gy||dy<gy.d))gy={v:n.y,d:dy}}if(start&&Math.abs(start.z-z)<.001){let dx=Math.abs(start.x-x),dy=Math.abs(start.y-y);if(dx<8/cam.zoom&&(!gx||dx<gx.d))gx={v:start.x,d:dx};if(dy<8/cam.zoom&&(!gy||dy<gy.d))gy={v:start.y,d:dy}}if(gx){x=gx.v;guides.push({axis:'x',v:x})}if(gy){y=gy.v;guides.push({axis:'y',v:y})}let angleGuide=false;if(start&&!gx&&!gy&&Math.abs(start.z-z)<.001){let dx=x-start.x,dy=y-start.y,length=Math.hypot(dx,dy);if(length>35/cam.zoom){let angle=Math.atan2(dy,dx),candidates=Array.from({length:8},(_,i)=>i*Math.PI/4);for(const road of world.roads){if((road.z??0)!==z||(road.zEnd??road.z??0)!==z)continue;let hit=projection(start,road.a,road.b);if(distance(start,hit)>.1)continue;let a=Math.atan2(road.b.y-road.a.y,road.b.x-road.a.x);for(let i=0;i<4;i++)candidates.push(a+i*Math.PI/2)}let nearest=candidates.map(a=>({a,d:Math.abs(Math.atan2(Math.sin(angle-a),Math.cos(angle-a)))})).sort((a,b)=>a.d-b.d)[0];if(nearest&&nearest.d<Math.PI/36){x=start.x+Math.cos(nearest.a)*length;y=start.y+Math.sin(nearest.a)*length;guides.push({axis:'angle',x:start.x,y:start.y,dx:Math.cos(nearest.a),dy:Math.sin(nearest.a),angle:nearest.a});angleGuide=true}}}if(!gx&&!angleGuide&&Math.abs(x-Math.round(x/50)*50)<4/cam.zoom){x=Math.round(x/50)*50;guides.push({axis:'x',v:x,grid:true})}if(!gy&&!angleGuide&&Math.abs(y-Math.round(y/50)*50)<4/cam.zoom){y=Math.round(y/50)*50;guides.push({axis:'y',v:y,grid:true})}return {x:Math.max(0,Math.min(SIZE,x)),y:Math.max(0,Math.min(SIZE,y)),z,snapKind:guides.length?'Alignment guide':'free',guides}}
function laneAnchorAt(road,p){
 let picked=pickLane(road,p,zPoint(road,road.a,0),zPoint(road,road.b,1));
 if(!picked){let direction=laneCount(road,0)?0:1,len=distance(road.a,road.b);picked={direction,index:0,divider:laneDivider(road),normal:{x:-(road.b.y-road.a.y)/len,y:(road.b.x-road.a.x)/len}}}
 return {roadId:road.id,groupId:road.groupId??road.id,direction:picked.direction,lane:picked.index,normal:picked.normal,offset:laneCenter(road,picked.direction,picked.index)};
}
function near(p){let best=null;for(const n of sim.graph.nodes.values()){let d=distance(p,{x:n.x,y:n.y-n.z*28});if(n.junction&&d<20/cam.zoom&&(!best||d<best.d))best={type:'junction',id:n.id,d}}if(best)return best;for(const b of world.buildings){let d=b.polygon&&pointInPolygon(p,b.polygon)?0:distance(p,b);if(d<25/cam.zoom&&(!best||d<best.d))best={type:'building',id:b.id,d}}for(const r of world.roads){let a=zPoint(r,r.a,0),b=zPoint(r,r.b,1),q=projection(p,a,b),d=distance(p,q);if(d<(14+8*totalLanes(r))/2+3/cam.zoom&&(!best||d<best.d))best={type:'road',id:r.id,d,point:p}}return best}
const instructions={select:'Click a vehicle to see its route, or inspect a road or junction.',road:'Choose Line, Curve, Ramp, or Circle, then click to build. Esc cancels.',roadside:'Drag across roads to set building sides. Release over a road to apply; Esc cancels.', 'move-node':'Drag a road node or junction to move it. Esc cancels.',home:'Click to place homes near a road.',shop:'Click to place a shop near a road.',industry:'Click to place industry near a road.',parking:'Click corners; they snap to the grid and other corners. Click the first corner or press Enter to finish; Esc cancels.',delete:'Drag over roads or buildings to remove them. Press Ctrl+Z to undo.','lane-replace':'Click the start and end of the section to replace, or drag along it. Esc cancels.','lane-add':'Click the start and end of the new lane, or drag along the road. Choose the side at the start.','lane-remove':'Click the start and end of the lane to remove, or drag along it.'};function roadInstruction(){return shapeType==='straight'?'Line: click a start point, then an end point.':shapeType==='roundabout'?'Circle: click the center, then its edge.':(shapeType==='ramp'?'Ramp':'Curve')+': click a start, a control point, then an end.'}function setTool(t){roadsideSelection.clear();roadsideHover=null;$('#roadsideOptions').hidden=t!=='roadside';tool=t;parkingCorners=[];buildingPreviewId=null;laneSelection=null;laneEndTarget=null;deleteHover=null;const options=$('#roadOptions');options.hidden=!['road','lane-replace'].includes(t);options.classList.toggle('editing',!options.hidden);options.classList.toggle('lane-mode',t==='lane-replace');$('#optionsTitle').textContent=t==='lane-replace'?'REPLACEMENT LANES':'ROAD DESIGN';$('#snapBadge').classList.toggle('active',t==='road');start=null;control=null;selected=null;inspector();document.querySelectorAll('.tool').forEach(b=>{b.classList.toggle('active',b.dataset.tool===t);b.setAttribute('aria-pressed',String(b.dataset.tool===t))});$('#hint').textContent=t==='road'?roadInstruction():instructions[t]}
let markerCache=null;
function connectorMarkers(nodeId){if(markerCache?.graph!==sim.graph||markerCache?.id!==nodeId||markerCache?.zoom!==cam.zoom)markerCache={graph:sim.graph,id:nodeId,zoom:cam.zoom,markers:intersectionMarkers(sim.graph,nodeId,cam.zoom)};return markerCache.markers}
function visibleConnections(){
 if(tool!=='select'||selected?.type!=='junction')return [];
 const node=sim.graph.nodes.get(selected.id);if(!node)return [];
 const markers=connectorMarkers(selected.id),incoming=markers.filter(m=>m.incoming),outgoing=markers.filter(m=>!m.incoming),cfg=world.junctions[selected.id]||{},custom=Array.isArray(cfg.connectors),result=[];
 for(const a of incoming){if(connectionSource&&(connectionSource.edge.id!==a.edge.id||connectionSource.lane!==a.lane))continue;for(const b of outgoing){if(b.edge.to===a.edge.from)continue;const key=movementKey(a.edge,a.lane,b.edge,b.lane),allowed=custom?cfg.connectors.includes(key):defaultMovement(sim.graph,a.edge,a.lane,b.edge,b.lane);if(allowed)result.push({a,b,key})}}
 return result;
}
function connectionAt(p){
 if(!p||markerAt(p))return null;
 const node=selected?.type==='junction'&&sim.graph.nodes.get(selected.id);if(!node)return null;
 const control={x:node.x,y:node.y-node.z*28};let best=null;
 for(const connection of visibleConnections()){let d=connectionDistance(p,connection.a,control,connection.b);if(d<6/cam.zoom&&(!best||d<best.d))best={...connection,d}}
 return best;
}
function toggleConnection(a,b){
 if(b.edge.to===a.edge.from)return false;
 const cfg=world.junctions[selected.id]||{mode:'priority',phase:12},moves=connectorOptions(selected.id),set=new Set(Array.isArray(cfg.connectors)?cfg.connectors:moves.filter(m=>m.defaultAllowed).map(m=>m.key)),key=movementKey(a.edge,a.lane,b.edge,b.lane),existed=set.has(key);
 rememberEdit();if(existed)set.delete(key);else set.add(key);world.junctions[selected.id]={...cfg,connectors:[...set]};sim.rulesChanged();save();inspector();$('#hint').textContent=existed?'Lane connection removed.':'Lane connection added.';return true;
}
function drawLaneConnections(){if(tool!=='select'||selected?.type!=='junction')return;let node=sim.graph.nodes.get(selected.id);if(!node)return;let markers=connectorMarkers(selected.id),incoming=markers.filter(m=>m.incoming),outgoing=markers.filter(m=>!m.incoming),cfg=world.junctions[selected.id]||{},custom=Array.isArray(cfg.connectors),palette=['#e39136','#6ba6bd','#90bb62','#bb7cac','#ce7864','#d1b552'],hoveredKey=connectionAt(mouse)?.key;ctx.save();for(const m of markers){ctx.beginPath();ctx.moveTo(m.anchorX,m.anchorY);ctx.lineTo(m.x,m.y);ctx.strokeStyle='#647f6c55';ctx.lineWidth=1/cam.zoom;ctx.stroke()}for(let i=0;i<incoming.length;i++){let a=incoming[i],color=palette[i%palette.length];for(const b of outgoing){if(b.edge.to===a.edge.from||(custom?!cfg.connectors.includes(movementKey(a.edge,a.lane,b.edge,b.lane)):!defaultMovement(sim.graph,a.edge,a.lane,b.edge,b.lane)))continue;if(connectionSource&&(connectionSource.edge.id!==a.edge.id||connectionSource.lane!==a.lane))continue;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.quadraticCurveTo(node.x,node.y-node.z*28,b.x,b.y);let hovered=hoveredKey===movementKey(a.edge,a.lane,b.edge,b.lane);ctx.lineWidth=(hovered?4:2)/cam.zoom;ctx.strokeStyle=hovered?'#df6860':color+'99';ctx.stroke()}}for(const m of incoming){let rule=cfg.mode==='signal'?null:cfg.approaches?.[m.edge.id];if(!rule||rule==='auto'||m.lane!==0)continue;ctx.save();ctx.font=`bold ${10/cam.zoom}px sans-serif`;ctx.textAlign='center';ctx.fillStyle=rule==='stop'?'#bf4943':rule==='yield'?'#a26621':'#39775c';ctx.fillText(rule.toUpperCase(),m.x,m.y-10/cam.zoom);ctx.restore()}for(let i=0;i<markers.length;i++){let m=markers[i],selectedMarker=connectionSource&&m.incoming&&connectionSource.edge.id===m.edge.id&&connectionSource.lane===m.lane;ctx.beginPath();ctx.arc(m.x,m.y,(selectedMarker?6:5)/cam.zoom,0,Math.PI*2);let permitted=!m.incoming&&connectionSource&&m.edge.to!==connectionSource.edge.from&&(custom?cfg.connectors.includes(movementKey(connectionSource.edge,connectionSource.lane,m.edge,m.lane)):defaultMovement(sim.graph,connectionSource.edge,connectionSource.lane,m.edge,m.lane));ctx.fillStyle=m.incoming?palette[incoming.indexOf(m)%palette.length]:permitted?'#79ba91':'#f7f7f2';ctx.fill();ctx.strokeStyle=selectedMarker?'#263b37':'#667b72';ctx.lineWidth=1;ctx.stroke()}if(drag?.laneConnection&&drag.moved&&connectionSource){ctx.beginPath();ctx.moveTo(connectionSource.x,connectionSource.y);ctx.quadraticCurveTo(node.x,node.y-node.z*28,drag.pointer.x,drag.pointer.y);ctx.strokeStyle='#5ba8a8';ctx.lineWidth=2/cam.zoom;ctx.setLineDash([5/cam.zoom,4/cam.zoom]);ctx.stroke()}ctx.restore()}
function markerAt(p){if(tool!=='select'||selected?.type!=='junction')return null;return connectorMarkers(selected.id).reduce((best,x)=>{let d=distance(p,x);return d<9/cam.zoom&&(!best||d<best.d)?{...x,d}:best},null)}
function rememberEdit(){history.push(JSON.stringify(world));if(history.length>30)history.shift()}
function clickLaneMarker(p){let m=markerAt(p);if(!m)return false;if(m.incoming){connectionSource=m;selected.approach=m.edge.id;inspector();$('#hint').textContent='Choose a right-of-way rule, or click an outgoing lane to edit its connection.';return true}if(!connectionSource)return true;toggleConnection(connectionSource,m);return true}
function connectorOptions(nodeId){let node=sim.graph.nodes.get(nodeId);if(!node)return [];let incoming=node.incoming,outgoing=node.out;const bearing=e=>{let a=sim.graph.nodes.get(e.from),b=sim.graph.nodes.get(e.to),ang=Math.atan2(b.y-a.y,b.x-a.x);return ['E','S','W','N'][(Math.round(ang/(Math.PI/2))+4)%4]};let out=[];for(const a of incoming)for(const b of outgoing){if(b.to===a.from)continue;for(let la=0;la<a.lanes;la++)for(let lb=0;lb<b.lanes;lb++)out.push({key:movementKey(a,la,b,lb),label:`${bearing(a)} lane ${la+1} → ${bearing(b)} lane ${lb+1}`,defaultAllowed:defaultMovement(sim.graph,a,la,b,lb)})}return out}
function inspector(){const p=$('#inspector');if(!selected){p.classList.add('hidden');return}p.classList.remove('hidden');if(selected.type==='vehicle'){vehicleInspector(p);return}if(selected.type==='junction'){let cfg=world.junctions[selected.id]||{mode:'priority',phase:12},moves=connectorOptions(selected.id),custom=Array.isArray(cfg.connectors),allowed=custom?new Set(cfg.connectors):new Set(moves.map(m=>m.key));p.innerHTML=`<h3>Intersection rules</h3><p>Click an incoming lane, then an outgoing lane, or drag between them. Click a connection to remove it.</p><button data-mode="priority" class="${cfg.mode==='priority'?'active':''}">Give way</button><button data-mode="signal" class="${cfg.mode==='signal'?'active':''}">Traffic light</button>${cfg.mode==='signal'?`<p>Green interval: <b>${cfg.phase||12}s</b></p><input id="phase" type="range" min="5" max="30" value="${cfg.phase||12}"><p>Then 2s amber and 1s all red.</p><label class="intersection-option"><input id="rightOnRed" type="checkbox" ${cfg.rightOnRed?'checked':''}> Allow right on red</label><p>Stop first, then give way to crossing traffic. Outside lane only.</p>`:''}${cfg.mode!=='signal'?`<p>Give way to the right. Left turns yield to oncoming traffic. Roundabouts and main-road merges have priority.</p><p>Click an incoming lane to override its right of way.</p>${selected.approach?`<div class="approach-rules"><b>Selected approach</b><div>${[['priority','Priority'],['yield','Yield'],['stop','Stop'],['auto','Automatic']].map(([value,label])=>`<button data-rule="${value}" class="${(cfg.approaches?.[selected.approach]||'auto')===value?'active':''}">${label}</button>`).join('')}</div></div>`:''}`:''}<button id="autoMoves">Reset lane links</button>`;p.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{if(cfg.mode===b.dataset.mode)return;rememberEdit();world.junctions[selected.id]={...cfg,mode:b.dataset.mode,phase:cfg.phase||12};sim.rulesChanged();cacheGraph=null;save();inspector()});p.querySelectorAll('[data-rule]').forEach(button=>button.onclick=()=>{let value=button.dataset.rule,approaches={...cfg.approaches};if((approaches[selected.approach]||'auto')===value)return;rememberEdit();if(value==='auto')delete approaches[selected.approach];else approaches[selected.approach]=value;world.junctions[selected.id]={...cfg,approaches};sim.rulesChanged();cacheGraph=null;save();inspector()});const rightOnRed=p.querySelector('#rightOnRed');if(rightOnRed)rightOnRed.onchange=()=>{rememberEdit();world.junctions[selected.id]={...cfg,rightOnRed:rightOnRed.checked};sim.rulesChanged();save();inspector()};let range=p.querySelector('#phase');if(range)range.onchange=()=>{if((cfg.phase||12)===+range.value)return;rememberEdit();world.junctions[selected.id]={...cfg,mode:'signal',phase:+range.value};sim.rulesChanged();save();inspector()};p.querySelector('#autoMoves').onclick=()=>{if(!Array.isArray(world.junctions[selected.id]?.connectors))return;rememberEdit();delete world.junctions[selected.id]?.connectors;sim.rulesChanged();save();inspector()}}else if(selected.type==='building'){let b=world.buildings.find(b=>b.id===selected.id),ok=sim.graph.attachments.some(a=>a.id===b.id);p.innerHTML=`<h3>${b.type}</h3><p>${ok?'Connected to the road network.':'No ground road nearby.'}</p>${b.type==='parking'?`<p>${sim.parkingCapacity(b)} parking spaces</p>${b.parkingDisconnected?'<p>No clear road entrance. Draw a new parking area with room for a driveway.</p>':''}`:b.type==='home'?`<p>${sim.graph.homeParking?.get(b.id)?'Parking at home.':sim.graph.attachments.find(a=>a.id===b.id)?.parkingId?'Uses a nearby parking lot.':'No parking space available.'}</p>`:''}`}else{let r=world.roads.find(r=>r.id===selected.id);p.innerHTML=`<h3>${r.oneway?'One-way':'Two-way'} road</h3><p>Elevation ${r.z??0} → ${r.zEnd??r.z??0}</p><label class="road-lane-setting">First click → last click<select id="laneF">${[0,1,2,3].map(n=>`<option value="${n}" ${n===(r.laneAtoB??r.lanes??1)?'selected':''}>${n} lane${n===1?'':'s'}</option>`).join('')}</select></label><label class="road-lane-setting">Reverse direction<select id="laneB">${[0,1,2,3].map(n=>`<option value="${n}" ${n===(r.laneBtoA??(r.oneway?0:r.lanes??1))?'selected':''}>${n} lane${n===1?'':'s'}</option>`).join('')}</select></label>`;const update=()=>{let f=+p.querySelector('#laneF').value,b=+p.querySelector('#laneB').value;if(!f&&!b){$('#hint').textContent='Choose at least one lane.';inspector();return}if(!setLaneLayout(r,selected.point||r.a,f,b))inspector()};p.querySelector('#laneF').onchange=update;p.querySelector('#laneB').onchange=update}}
const uuid=()=>crypto.randomUUID();
function draftPoints(shape,a,b,handle){if(shape==='roundabout'){let radius=distance(a,b);return Array.from({length:49},(_,i)=>({x:a.x+radius*Math.cos(-i*Math.PI/24),y:a.y+radius*Math.sin(-i*Math.PI/24)}))}if((shape==='curve'||shape==='ramp')&&handle)return sampleCurve(a,handle,b,$('#guides').checked);return [a,b]}
let draftCache=null;
function draftError(shape,a,b,handle){let key=JSON.stringify([shape,a,b,handle,$('#lanesForward').value,$('#lanesBackward').value,$('#guides').checked]);if(draftCache?.graph===sim.graph&&draftCache.key===key)return draftCache.error;let error=calculateDraftError(shape,a,b,handle);draftCache={graph:sim.graph,key,error};return error}
function calculateDraftError(shape,a,b,handle){if(!a||!b)return null;if(shape!=='ramp'&&shape!=='roundabout'&&+$('#lanesForward').value+ +$('#lanesBackward').value===0)return 'Choose at least one lane.';if((shape==='curve'||shape==='ramp')&&!handle)return null;if(shape==='roundabout'&&(distance(a,b)<35||distance(a,b)>180))return 'Choose a radius between 35 and 180 units.';let f=shape==='ramp'?1:shape==='roundabout'?Math.max(1,+$('#lanesForward').value):+$('#lanesForward').value,back=shape==='ramp'||shape==='roundabout'?0:+$('#lanesBackward').value;return validateRoadPath(world,draftPoints(shape,a,b,handle),a.z,shape==='roundabout'?a.z:b.z,(14+8*(f+back))/2).error}
function roadBatch(points,z,zEnd,shapeRoundabout=false,singleLaneRamp=false,anchors=null){let lanesForward=singleLaneRamp?1:shapeRoundabout?Math.max(1,+$('#lanesForward').value):+$('#lanesForward').value,lanesBackward=shapeRoundabout||singleLaneRamp?0:+$('#lanesBackward').value,roads=[],groupId=uuid();if(lanesForward+lanesBackward===0)return 'A road needs at least one open direction.';let checked=validateRoadPath(world,points,z,zEnd,(14+8*(lanesForward+lanesBackward))/2);if(checked.error)return checked.error;for(let i=1;i<points.length;i++){let a=points[i-1],b=points[i];roads.push({id:uuid(),groupId,a:{x:a.x,y:a.y},b:{x:b.x,y:b.y},z:checked.levels[i-1],zEnd:checked.levels[i],lanes:Math.max(lanesForward,lanesBackward),laneAtoB:lanesForward,laneBtoA:lanesBackward,oneway:lanesForward===0||lanesBackward===0,roundabout:shapeRoundabout,ramp:singleLaneRamp,roadside:+$('#newRoadside').value})}let proposed={...world,roads:[...world.roads,...roads],junctions:structuredClone(world.junctions)},graph=buildGraph(proposed);remapConnectionsAfterBuild(sim.graph,graph,proposed.junctions);let linkError=singleLaneRamp&&anchors?connectRampToLanes(graph,proposed.junctions,groupId,anchors.start,anchors.end):null;if(linkError)return linkError;change(()=>{world.roads=proposed.roads;world.junctions=proposed.junctions},graph);return null}

function buildClick(p){let shape=shapeType,target=+$('#elevation').value;if(shape==='roundabout'){if(!start){start=snap(p,target);$('#hint').textContent='Click again to set the roundabout radius.';return}let rad=distance(start,p);if(rad<35||rad>180){$('#hint').textContent='Choose a radius between 35 and 180 units.';return}let points=Array.from({length:49},(_,i)=>({x:start.x+rad*Math.cos(-i*Math.PI/24),y:start.y+rad*Math.sin(-i*Math.PI/24)})),err=roadBatch(points,target,target,true);$('#hint').textContent=err||'Roundabout built. Snap roads to its ring to connect.';return}
if(!start){start=snap(p,null);$('#hint').textContent=(shape==='curve'||shape==='ramp')?'Click the curve control point.':'Choose an endpoint; change elevation for a ramp.';return}if((shape==='curve'||shape==='ramp')&&!control){control=snap(p,start.z);$('#hint').textContent='Click the curve endpoint.';return}let end=snap(p,target),points=draftPoints(shape,start,end,control);let err=roadBatch(points,start.z,target,false,shape==='ramp',{start,end});$('#hint').textContent=err||instructions.road}
function roadAt(p){let best=null;for(const r of world.roads){let a=zPoint(r,r.a,0),b=zPoint(r,r.b,1),q=projection(p,a,b),d=distance(p,q),radius=(14+totalLanes(r)*8)/2+8;if(d<radius&&(!best||d<best.d))best={road:r,d,point:{x:r.a.x+(r.b.x-r.a.x)*q.t,y:r.a.y+(r.b.y-r.a.y)*q.t}}}return best}
function deleteRanges(r,p){const radius=35,group=r.groupId||r.id,result=[];for(const road of world.roads){if((road.groupId||road.id)!==group)continue;let q=projection(p,road.a,road.b),dist=distance(p,q),len=distance(road.a,road.b);if(dist>=radius||len<.01)continue;let reach=Math.sqrt(radius*radius-dist*dist)/len,lo=Math.max(0,q.t-reach),hi=Math.min(1,q.t+reach);if(lo*len<14)lo=0;if((1-hi)*len<14)hi=1;let at=t=>({x:road.a.x+(road.b.x-road.a.x)*t,y:road.a.y+(road.b.y-road.a.y)*t});result.push({road,lo,hi,a:at(lo),b:at(hi)})}return result}
function deleteSection(r,p,batch=false){let ranges=deleteRanges(r,p);if(!ranges.length)return;let apply=()=>{const ids=new Set(ranges.map(x=>x.road.id));let replacements=[];for(const cut of ranges){let road=cut.road,z0=road.z??0,z1=road.zEnd??z0,z=t=>z0+(z1-z0)*t;if(cut.lo>0)replacements.push({...road,id:uuid(),groupId:road.groupId||road.id,b:cut.a,zEnd:z(cut.lo)});if(cut.hi<1)replacements.push({...road,id:uuid(),groupId:road.groupId||road.id,a:cut.b,z:z(cut.hi)})}world.roads=world.roads.filter(x=>!ids.has(x.id));world.roads.push(...replacements)};if(batch){apply();selected=null;cacheGraph=null}else{let old=sim.graph;change(apply);remapConnectionsAfterBuild(old,sim.graph,world.junctions);save()}}

function rebuildAfterBulldoze(){let old=sim.graph;sim.rebuild();remapConnectionsAfterBuild(old,sim.graph,world.junctions)}
function bulldoze(p){let hit=roadAt(p);if(hit){deleteSection(hit.road,hit.point,true);return true}let b=world.buildings.find(b=>distance(p,b)<18);if(b){world.buildings=world.buildings.filter(x=>x!==b);cacheGraph=null;return true}return false}

function selectedLaneSection(road,p){return laneSection(world,sim.graph,road,p)}
function setLaneLayout(road,p,forward,reverse,edit=null,range=null){let oldGraph=sim.graph,section=range||selectedLaneSection(road,p);if(!section.length)return false;let error=validateLaneSection(world,section,forward,reverse);if(error){$('#hint').textContent=error;return false}change(()=>applyLaneSection(world,section,forward,reverse,uuid));remapLaneConnections(oldGraph,sim.graph,world.junctions,section,edit);sim.routeCache.clear();deleteHover=null;save();return true}
function editLane(p,delta,range=null,initialHit=null){let hit=initialHit||roadAt(p);if(!hit){$('#hint').textContent='Click a side of a road.';return}let r=hit.road;if(delta===0){let f=+$('#lanesForward').value,b=+$('#lanesBackward').value;if(!f&&!b){$('#hint').textContent='Choose at least one lane.';return}if(setLaneLayout(r,hit.point,f,b,null,range))$('#hint').textContent='Lane layout replaced.';return}let picked=pickLane(r,p,zPoint(r,r.a,0),zPoint(r,r.b,1),delta>0);if(!picked){$('#hint').textContent='There is no lane on that side.';return}let {direction,index,count:old}=picked;if(delta>0)index=old;let count=Math.max(0,Math.min(3,old+delta));if(count===old||count+laneCount(r,1-direction)===0){$('#hint').textContent='That lane count is not available.';return}let forward=direction===0?count:laneCount(r,0),reverse=direction===1?count:laneCount(r,1);if(setLaneLayout(r,hit.point,forward,reverse,{direction,index,action:delta>0?'add':'remove'},range))$('#hint').textContent=(direction===0?'Forward':'Reverse')+' lane '+(index+1)+(delta>0?' added.':' removed.')}

function laneDragTarget(section,p){
 let best=null;
 for(const span of section){
  const a=zPoint(span.road,span.a,span.lo),b=zPoint(span.road,span.b,span.hi),q=projection(p,a,b),d=distance(p,q);
  if(!best||d<best.d)best={road:span.road,point:{x:span.a.x+(span.b.x-span.a.x)*q.t,y:span.a.y+(span.b.y-span.a.y)*q.t},d};
 }
 return best&&best.d<32/cam.zoom?snapLaneRangeToJunction(section,sim.graph,best,18/cam.zoom):null;
}
function selectRoadsideAt(p){
 const hit=roadAt(p);if(!hit)return;const group=hit.road.groupId??hit.road.id;let added=false;
 for(const road of world.roads)if((road.groupId??road.id)===group&&!roadsideSelection.has(road.id)){roadsideSelection.add(road.id);added=true}
 if(added)roadsideTone('select');
}
function roadsideTone(kind){
 if(!$('#roadsideSound').checked)return;try{const Audio=window.AudioContext||window.webkitAudioContext;if(!Audio)return;const audio=roadsideTone.audio||(roadsideTone.audio=new Audio());const oscillator=audio.createOscillator(),gain=audio.createGain();oscillator.frequency.value=kind==='apply'?660:kind==='cancel'?250:440;gain.gain.setValueAtTime(.035,audio.currentTime);gain.gain.exponentialRampToValueAtTime(.001,audio.currentTime+.07);oscillator.connect(gain);gain.connect(audio.destination);oscillator.start();oscillator.stop(audio.currentTime+.07)}catch{}
}
function applyRoadside(){
 const mode=+$('#roadsideMode').value,ids=new Set(roadsideSelection);roadsideSelection.clear();if(!ids.size)return;
 const affected=affectedBuildings(world.buildings,world.roads,ids,mode);
 if(affected.length&&$('#protectRoadside').checked){roadsideTone('cancel');$('#hint').textContent=`Protected: ${affected.length} existing building(s) on sides being disabled. Turn off protection to apply.`;return}
 if(!world.roads.some(r=>ids.has(r.id)&&(r.roadside??3)!==mode)){$('#hint').textContent='Those roads already have these building sides.';return}
 change(()=>{for(const r of world.roads)if(ids.has(r.id))r.roadside=mode});roadsideTone('apply');$('#hint').textContent=affected.length?`Building sides updated. ${affected.length} existing building(s) kept their access.`:'Building sides updated. Press Ctrl+Z to undo.';
}
function drawRoadsidePreview(){
 if(tool!=='roadside')return;const ids=new Set(roadsideSelection);if(!ids.size&&roadsideHover){const group=roadsideHover.road.groupId??roadsideHover.road.id;for(const r of world.roads)if((r.groupId??r.id)===group)ids.add(r.id)}
 const mode=+$('#roadsideMode').value;ctx.save();ctx.lineCap='round';for(const r of world.roads){if(!ids.has(r.id))continue;let a=zPoint(r,r.a,0),b=zPoint(r,r.b,1),dx=b.x-a.x,dy=b.y-a.y,len=Math.hypot(dx,dy)||1,n={x:-dy/len,y:dx/len},offset=(14+8*totalLanes(r))/2+6;line(a,b,4/cam.zoom,'#188c9bc0');if(len>30/cam.zoom){ctx.save();ctx.translate((a.x+b.x)/2,(a.y+b.y)/2);ctx.rotate(Math.atan2(dy,dx));ctx.fillStyle='#176c78';ctx.beginPath();ctx.moveTo(6/cam.zoom,0);ctx.lineTo(-4/cam.zoom,-4/cam.zoom);ctx.lineTo(-4/cam.zoom,4/cam.zoom);ctx.closePath();ctx.fill();ctx.restore()}for(const [sign,bit] of [[-1,2],[1,1]])line(offsetPoint(a,n,sign*offset),offsetPoint(b,n,sign*offset),5/cam.zoom,mode&bit?'#54bf80dd':'#e6796bdd')}ctx.restore();
}
function clickMap(e){let p=point(e);if(tool==='select'&&!markerAt(p)){const car=vehicleAt(p);if(car){selected={type:'vehicle',id:car.id};connectionSource=null;inspector();return}}if(tool==='lane-add'||tool==='lane-remove'||tool==='lane-replace'){editLane(p,tool==='lane-replace'?0:tool==='lane-add'?1:-1);return}if(clickLaneMarker(p))return;let link=connectionAt(p);if(link){toggleConnection(link.a,link.b);return}if(tool==='road'){buildClick(p);return}if(tool==='parking'){if(parkingCorners.length>=3&&distance(p,parkingCorners[0])<12/cam.zoom){finishParking();return}p=parkingSnap(p);if(p.x<0||p.y<0||p.x>SIZE||p.y>SIZE){$('#hint').textContent='Choose corners inside the map.';return}parkingCorners.push({x:p.x,y:p.y});$('#hint').textContent=parkingCorners.length<3?'Add at least three corners.':'Add another corner, click the first corner, or press Enter to finish.';return}if(['home','shop','industry'].includes(tool)){if(!canPlaceBuilding(p)){$('#hint').textContent=roadsideAllowed(p,world.roads)?'Buildings need clear land.':'Building placement is disabled on this side of the nearest road.';return}let id=buildingPreviewId||uuid();change(()=>world.buildings.push({id,type:tool,x:p.x,y:p.y}));buildingPreviewId=null;return}let hit=near(p);if(tool==='delete'){if(hit?.type==='road')deleteSection(world.roads.find(r=>r.id===hit.id),p);else if(hit?.type==='junction'){$('#hint').textContent='Click a road away from the junction to remove a section.'}else if(hit?.type==='building')change(()=>world.buildings=world.buildings.filter(b=>b.id!==hit.id));return}selected=hit;connectionSource=null;if(hit?.type==='junction'){let n=sim.graph.nodes.get(hit.id);cam.zoom=Math.max(cam.zoom,1.8);cam.x=n.x+Math.min(280,view.clientWidth*.52)/(2*cam.zoom);cam.y=n.y-n.z*28}inspector()}
let nodeHandleCache=null;
function nodeHandles(){if(nodeHandleCache?.graph!==sim.graph||nodeHandleCache?.zoom!==cam.zoom)nodeHandleCache={graph:sim.graph,zoom:cam.zoom,nodes:spacedNodeHandles(movableRoadNodes(world,sim.graph),cam.zoom)};return nodeHandleCache.nodes}
function nodeAt(p){if(!p||tool!=='move-node')return null;let best=null;for(const n of nodeHandles()){let d=distance(p,{x:n.x,y:n.y-n.z*28});if(d<12/cam.zoom&&(!best||d<best.d))best={node:n,d}}return best?.node||null}
function drawNodeHandles(){if(tool!=='move-node')return;let hover=nodeAt(mouse);ctx.save();for(const n of nodeHandles()){let active=drag?.nodeMove?.id===n.id||hover?.id===n.id,p={x:n.x,y:n.y-n.z*28};ctx.beginPath();ctx.arc(p.x,p.y,(active?8:5)/cam.zoom,0,Math.PI*2);ctx.fillStyle=active?'#54b4a7':'#fff';ctx.fill();ctx.lineWidth=2/cam.zoom;ctx.strokeStyle='#278e89';ctx.stroke()}if(drag?.nodeMove&&drag.moved){let n=drag.nodeMove,p=drag.pointer;ctx.setLineDash([6/cam.zoom,4/cam.zoom]);ctx.lineWidth=3/cam.zoom;ctx.strokeStyle='#248f8e';for(const e of [...n.out,...n.incoming]){let other=sim.graph.nodes.get(e.from===n.id?e.to:e.from);if(!other)continue;ctx.beginPath();ctx.moveTo(other.x,other.y-other.z*28);ctx.lineTo(p.x,p.y);ctx.stroke()}ctx.setLineDash([]);ctx.beginPath();ctx.arc(p.x,p.y,8/cam.zoom,0,Math.PI*2);ctx.fillStyle='#54b4a7';ctx.fill();ctx.stroke()}ctx.restore()}
canvas.onpointerdown=e=>{canvas.setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,originX:e.clientX,originY:e.clientY,moved:false,button:e.button};if(tool==='roadside'&&e.button===0){drag.roadside=true;drag.previous=point(e);roadsideSelection.clear();selectRoadsideAt(drag.previous);return}if(tool==='move-node'&&e.button===0){let n=nodeAt(point(e));if(n){drag.nodeMove=n;drag.origin=point(e);drag.pointer=drag.origin;return}}if(tool==='select'&&selected?.type==='junction'&&e.button===0){let p=point(e),marker=markerAt(p);if(marker?.incoming){connectionSource=marker;$('#hint').textContent='Choose an outgoing lane to add or remove its connection.';drag.laneConnection=true;drag.origin=p;drag.pointer=p;return}}if(['lane-add','lane-remove','lane-replace'].includes(tool)&&e.button===0){let p=point(e);if(laneSelection){drag.laneFinish=true;return}let hit=roadAt(p);if(hit){let section=selectedLaneSection(hit.road,hit.point),snapped=snapLaneRangeToJunction(section,sim.graph,hit,18/cam.zoom);drag.laneEdit={hit:snapped,origin:p,section,range:[]};return}$('#hint').textContent='Choose a point on the road first.';drag.laneFinish=true;return}if(tool==='delete'&&e.button===0){drag.bulldoze=true;drag.previous=point(e);history.push(JSON.stringify(world));if(history.length>30)history.shift();drag.changed=bulldoze(drag.previous);if(drag.changed)rebuildAfterBulldoze();deleteHover=null}};canvas.onpointermove=e=>{if(drag?.button===2){let dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(dx||dy){drag.moved=true;cam.x-=dx/cam.zoom;cam.y-=dy/cam.zoom;drag.x=e.clientX;drag.y=e.clientY}mouse=null;snapTarget=null;return}mouse=point(e);if(tool==='roadside'&&(!drag||drag.roadside)){roadsideHover=roadAt(mouse);if(drag?.roadside){let d=distance(drag.previous,mouse),steps=Math.max(1,Math.ceil(d/8));for(let i=1;i<=steps;i++)selectRoadsideAt({x:drag.previous.x+(mouse.x-drag.previous.x)*i/steps,y:drag.previous.y+(mouse.y-drag.previous.y)*i/steps});drag.previous=mouse}return}canvas.style.cursor=tool==='parking'?'crosshair':nodeAt(mouse)?'grab':markerAt(mouse)||connectionAt(mouse)?'pointer':'';if(drag?.nodeMove){drag.pointer=mouse;drag.moved=drag.moved||distance(drag.origin,mouse)>4/cam.zoom;canvas.style.cursor='grabbing';return}if(drag?.laneConnection){drag.pointer=mouse;drag.moved=drag.moved||distance(drag.origin,mouse)>4/cam.zoom;return}if(drag?.laneEdit){let edit=drag.laneEdit;drag.moved=drag.moved||distance(edit.origin,mouse)>4/cam.zoom;let end=laneDragTarget(edit.section,mouse);laneEndTarget=end;edit.range=drag.moved&&end?laneRange(edit.section,edit.hit,end):[];deleteHover=edit.range.length?{...edit.hit,section:edit.range,pickPoint:edit.origin}:null;return}if(laneSelection&&(!drag||drag.laneFinish)){let end=laneDragTarget(laneSelection.section,mouse);laneEndTarget=end;let range=end?laneRange(laneSelection.section,laneSelection.hit,end):[];deleteHover=range.length?{...laneSelection.hit,section:range,pickPoint:laneSelection.origin}:null;return}if((tool==='lane-add'||tool==='lane-remove'||tool==='lane-replace')&&(!drag||drag.button===0)){deleteHover=roadAt(mouse);if(deleteHover)deleteHover.section=selectedLaneSection(deleteHover.road,deleteHover.point);return}if(tool==='delete'&&(!drag||drag.button===0)){deleteHover=roadAt(mouse);if(drag?.bulldoze){let prev=drag.previous,dist=distance(prev,mouse),steps=Math.max(1,Math.ceil(dist/18));let changed=false;for(let i=1;i<=steps;i++){let p={x:prev.x+(mouse.x-prev.x)*i/steps,y:prev.y+(mouse.y-prev.y)*i/steps};changed=bulldoze(p)||changed}if(changed)rebuildAfterBulldoze();drag.changed=changed||drag.changed;drag.previous=mouse;deleteHover=roadAt(mouse)}return}if(tool==='road'&&mouse){let snapLevel=start?((shapeType==='curve'||shapeType==='ramp')&&!control?start.z:+$('#elevation').value):null,target=snap(mouse,snapLevel),badge=$('#snapBadge');snapTarget=target;let error=start?draftError(shapeType,start,target,control):null;badge.classList.toggle('connected',target.snapKind!=='free');badge.classList.toggle('blocked',!!error);badge.querySelector('span').textContent=error||(target.snapKind==='free'?'No snap target':target.snapKind+(start?' · endpoint ready':' · start'))}if(!drag)return;let dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(drag.moved||Math.hypot(e.clientX-drag.originX,e.clientY-drag.originY)>4){drag.moved=true;cam.x-=dx/cam.zoom;cam.y-=dy/cam.zoom;drag.x=e.clientX;drag.y=e.clientY}};canvas.onpointerup=e=>{laneEndTarget=null;if(drag?.roadside){if(roadAt(point(e)))applyRoadside();else{roadsideSelection.clear();roadsideTone('cancel');$('#hint').textContent='Selection cancelled.'}}else if(drag?.nodeMove){if(drag.moved){let n=drag.nodeMove,p=point(e),target={x:p.x,y:p.y+n.z*28},result=moveRoadNode(world,sim.graph,n.id,target,uuid);if(result.error)$('#hint').textContent=result.error;else{change(()=>{world.roads=result.world.roads;world.junctions=result.world.junctions},result.graph);$('#hint').textContent='Road node moved. Press Ctrl+Z to undo.'}}}else if(drag?.laneFinish){if(laneSelection){let end=laneDragTarget(laneSelection.section,point(e)),range=end?laneRange(laneSelection.section,laneSelection.hit,end):[],length=range.reduce((sum,part)=>sum+distance(part.a,part.b),0);if(length<10)$('#hint').textContent='Choose an end point on the same road, at least 10 units away.';else{let edit=laneSelection;laneSelection=null;editLane(edit.origin,tool==='lane-replace'?0:tool==='lane-add'?1:-1,range,edit.hit);deleteHover=null}}}else if(drag?.laneEdit){let edit=drag.laneEdit,delta=tool==='lane-replace'?0:tool==='lane-add'?1:-1;if(drag.moved){let length=edit.range.reduce((sum,part)=>sum+distance(part.a,part.b),0);if(length<10)$('#hint').textContent='Drag along the same road for at least 10 units.';else editLane(edit.origin,delta,edit.range,edit.hit);deleteHover=null}else{laneSelection=edit;$('#hint').textContent='Click the end of the section. Move near a junction to snap to it. Esc cancels.'}}else if(drag?.laneConnection){if(drag.moved)clickLaneMarker(point(e));else clickLaneMarker(drag.origin)}else if(drag?.bulldoze){if(drag.changed){save();inspector()}else history.pop();deleteHover=roadAt(point(e))}else if(drag&&!drag.moved&&drag.button===0)clickMap(e);drag=null};canvas.onpointercancel=()=>{drag=null;roadsideSelection.clear()};canvas.onpointerleave=()=>{if(!drag){deleteHover=null;mouse=null;snapTarget=null;roadsideHover=null}canvas.style.cursor=''};canvas.oncontextmenu=e=>e.preventDefault();function zoom(f,x,y){let r=view.getBoundingClientRect(),sx=x??r.width/2,sy=y??r.height/2,wx=(sx-r.width/2)/cam.zoom+cam.x,wy=(sy-r.height/2)/cam.zoom+cam.y;cam.zoom=Math.max(.25,Math.min(3,cam.zoom*f));cam.x=wx-(sx-r.width/2)/cam.zoom;cam.y=wy-(sy-r.height/2)/cam.zoom}canvas.onwheel=e=>{e.preventDefault();let r=canvas.getBoundingClientRect();zoom(e.deltaY<0?1.12:1/1.12,e.clientX-r.left,e.clientY-r.top)};
document.querySelectorAll('[data-shape]').forEach(b=>b.onclick=()=>{shapeType=b.dataset.shape;setTool('road');start=null;control=null;document.querySelectorAll('[data-shape]').forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b))});$('#hint').textContent=(shapeType==='curve'||shapeType==='ramp')?(shapeType==='ramp'?'Ramp: start, control point, then end.':'Curve: start, control point, then end.'):shapeType==='roundabout'?'Roundabout: click center, then radius.':'Straight: click start, then end.'});for(const [id,delta] of [['#elevationDown',-1],['#elevationUp',1]])$(id).onclick=()=>{let el=$('#elevation');el.selectedIndex=Math.max(0,Math.min(el.options.length-1,el.selectedIndex+delta));el.dispatchEvent(new Event('change'))};$('#tools').onclick=e=>{let b=e.target.closest('[data-tool]');if(b)setTool(b.dataset.tool)};$('#pause').onclick=()=>{paused=!paused;$('#pause').textContent=paused?'▶':'Ⅱ';$('#pause').setAttribute('aria-label',paused?'Resume simulation':'Pause simulation')};document.querySelectorAll('.speed').forEach(b=>b.onclick=()=>{speed=+b.dataset.speed;document.querySelectorAll('.speed').forEach(x=>x.classList.toggle('active',x===b))});$('#demand').value=Math.round(world.demand*100);$('#demandValue').textContent=$('#demand').value+'%';$('#demand').oninput=e=>{world.demand=+e.target.value/100;$('#demandValue').textContent=e.target.value+'%';save()};$('#overlay').onchange=e=>overlay=e.target.checked;$('#plus').onclick=()=>zoom(1.25);$('#minus').onclick=()=>zoom(.8);$('#fit').onclick=fit;$('#dismiss').onclick=()=>$('#tip')?.remove();setTimeout(()=>$('#tip')?.remove(),8000);function replace(w){mouse=null;buildingPreviewId=null;laneSelection=null;deleteHover=null;world=w;world.demoId||='starter';repairParkingEntrances(world);sim=new Simulation(w);selected=null;start=null;control=null;history=[];$('#demand').value=Math.round((world.demand??1)*100);$('#demandValue').textContent=$('#demand').value+'%';$('#mapSelect').value=world.demoId;inspector();title();save();fit()}$('#mapSelect').value=world.demoId;$('#mapSelect').onchange=e=>{save();const id=e.target.value;let cached;try{cached=JSON.parse(localStorage.getItem(`tiny-detour-map-v2-${id}`))}catch{}replace(validWorld(cached)?migrate(cached):demoWorld(id))};$('#reset').onclick=()=>{if(confirm('Reset this map and remove your local changes?'))replace(demoWorld(world.demoId))};$('#blank').onclick=()=>{let exists=false;try{exists=!!localStorage.getItem('tiny-detour-map-v2-blank')}catch{}if(exists&&!confirm('Replace your saved blank map? This cannot be undone.'))return;replace(demoWorld('blank'))};function undo(){if(!history.length){$('#hint').textContent='Nothing to undo yet.';return}world=JSON.parse(history.pop());repairParkingEntrances(world);sim.world=world;sim.rebuild();sim.rulesChanged();$('#demand').value=Math.round((world.demand??1)*100);$('#demandValue').textContent=$('#demand').value+'%';selected=null;start=null;control=null;save();inspector();$('#hint').textContent='Last edit undone.'}$('#undo').onclick=undo;window.onkeydown=e=>{if(e.key==='Enter'&&tool==='parking'&&!['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)){e.preventDefault();finishParking();return}if(e.key==='Escape'){parkingCorners=[];if(drag?.roadside){drag=null;roadsideTone('cancel')}roadsideSelection.clear();if(drag?.nodeMove)drag=null;start=null;control=null;connectionSource=null;laneSelection=null;laneEndTarget=null;deleteHover=null;$('#hint').textContent=instructions[tool]}if(tool==='roadside'&&e.key.toLowerCase()==='i'&&!['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)){let el=$('#roadsideMode');el.value=String((+el.value)^3);roadsideTone('select');e.preventDefault()}if(e.key>='1'&&e.key<='6'&&!['INPUT','SELECT'].includes(e.target.tagName))setTool(['select','road','home','shop','industry','delete'][+e.key-1]);if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'&&!['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)){e.preventDefault();undo()}};
function line(a,b,width,color,dash=[]){ctx.lineWidth=width;ctx.strokeStyle=color;ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.setLineDash([])}function shape(points,scale=1,offset=0,pathOnly=false){let segments=closedSplineSegments(points,scale,offset);if(!segments.length)return;ctx.beginPath();ctx.moveTo(segments[0].start.x,segments[0].start.y);for(const {c1,c2,end} of segments)ctx.bezierCurveTo(c1.x,c1.y,c2.x,c2.y,end.x,end.y);ctx.closePath();if(!pathOnly)ctx.fill()}function totalLanes(rd){return laneCount(rd,0)+laneCount(rd,1)}function zPoint(rd,p,t){return {x:p.x,y:p.y-((rd.z??0)+((rd.zEnd??rd.z??0)-(rd.z??0))*t)*28}}function offsetPoint(p,n,d){return {x:p.x+n.x*d,y:p.y+n.y*d}}
function drawLaneTransitions(level,pass,surfaceOnly=false){
 for(const node of sim.graph.nodes.values()){
  if(node.junction||node.degree!==2||Math.min(2,Math.round(node.z))!==level)continue;
  const arms=new Map();for(const edge of [...node.incoming,...node.out]){let id=edge.from===node.id?edge.to:edge.from,other=sim.graph.nodes.get(id),road=sim.graph.roadById.get(edge.road);if(!other||!road)continue;let dx=other.x-node.x,dy=(other.y-other.z*28)-(node.y-node.z*28),len=Math.hypot(dx,dy);if(len>.01&&!arms.has(id))arms.set(id,{dx:dx/len,dy:dy/len,len,road})}
  let pair=[...arms.values()];if(pair.length!==2)continue;let [a,b]=pair,wa=14+8*totalLanes(a.road),wb=14+8*totalLanes(b.road);if(wa===wb||a.dx*b.dx+a.dy*b.dy>-.94)continue;
  let reach=Math.min(32,a.len*.7,b.len*.7);if(reach<2)continue;let center={x:node.x,y:node.y-node.z*28},normal={x:-a.dy,y:a.dx},pa={x:center.x+a.dx*reach,y:center.y+a.dy*reach},pb={x:center.x+b.dx*reach,y:center.y+b.dy*reach},extra=pass===0?1.5:0,ha=wa/2+extra,hb=wb/2+extra,al=offsetPoint(pa,normal,ha),ar=offsetPoint(pa,normal,-ha),bl=offsetPoint(pb,normal,hb),br=offsetPoint(pb,normal,-hb);
  // Match the adjoining road tangents so the added width tapers smoothly.
  const vx=(pb.x-pa.x)*.42,vy=(pb.y-pa.y)*.42;
  const taper=(from,to,reverse=false)=>{let sign=reverse?-1:1;ctx.bezierCurveTo(from.x+vx*sign,from.y+vy*sign,to.x-vx*sign,to.y-vy*sign,to.x,to.y)};
  if(pass<2){
   ctx.fillStyle=pass===0?'#979996':'#fff';ctx.beginPath();ctx.moveTo(al.x,al.y);taper(al,bl);ctx.lineTo(br.x,br.y);taper(br,ar,true);ctx.closePath();ctx.fill();
   if(pass===1&&!surfaceOnly){ctx.beginPath();ctx.moveTo(al.x,al.y);taper(al,bl);ctx.moveTo(ar.x,ar.y);taper(ar,br);ctx.strokeStyle='#979996';ctx.lineWidth=1.5;ctx.stroke()}
  }
  if(pass===2){
   const point=(p,r,offset)=>{let a=zPoint(r,r.a,0),b=zPoint(r,r.b,1),dx=b.x-a.x,dy=b.y-a.y,l=Math.hypot(dx,dy)||1;return {x:p.x-dy/l*offset,y:p.y+dx/l*offset}};
   const da=laneDivider(a.road),db=laneDivider(b.road);
   const mark=(start,end,width,color,dash=[])=>{ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(start.x,start.y);ctx.bezierCurveTo(start.x+vx,start.y+vy,end.x-vx,end.y-vy,end.x,end.y);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();ctx.setLineDash([])};
   if(laneCount(a.road,0)&&laneCount(a.road,1)&&laneCount(b.road,0)&&laneCount(b.road,1))mark(point(pa,a.road,da),point(pb,b.road,db),1.3,'#e3c840');
   for(const direction of [0,1])for(let i=1;i<Math.max(laneCount(a.road,direction),laneCount(b.road,direction));i++){
    const oa=laneBoundary(a.road,direction,Math.min(i,laneCount(a.road,direction))),ob=laneBoundary(b.road,direction,Math.min(i,laneCount(b.road,direction)));
    mark(point(pa,a.road,oa),point(pb,b.road,ob),1,'#aaaead',[5,8]);
   }
  }
 }
}
function drawDeadEnds(roads,level,pass){
 const available=new Set(roads.map(r=>r.id));
 for(const node of sim.graph.nodes.values()){
  if(node.degree!==1||Math.min(2,Math.round(node.z))!==level||node.x<1||node.y<1||node.x>SIZE-1||node.y>SIZE-1)continue;
  const edge=node.out[0]||node.incoming[0],road=edge&&sim.graph.roadById.get(edge.road);if(!road||!available.has(road.id))continue;
  const radius=(14+8*totalLanes(road)+(pass===0?3:0))/2;
  ctx.fillStyle=pass===0?(road.oneway&&!road.roundabout?'#303431':'#979996'):'#fff';ctx.beginPath();ctx.arc(node.x,node.y-node.z*28,radius,0,Math.PI*2);ctx.fill();
 }
}
// Join center markings where two separately drawn roads form one bend.
function drawBendMarkings(level){
 for(const n of sim.graph.nodes.values()){
  if(n.junction||n.degree!==2||Math.min(2,Math.round(n.z))!==level)continue;
  const arms=new Map();for(const e of [...n.incoming,...n.out]){
   const other=sim.graph.nodes.get(e.from===n.id?e.to:e.from),road=sim.graph.roadById.get(e.road);if(!other||!road||!laneCount(road,0)||!laneCount(road,1)||road.roundabout)continue;
   let dx=other.x-n.x,dy=(other.y-other.z*28)-(n.y-n.z*28),len=Math.hypot(dx,dy);if(len<2)continue;
   const ra=zPoint(road,road.a,0),rb=zPoint(road,road.b,1),rl=Math.hypot(rb.x-ra.x,rb.y-ra.y)||1,divider=laneDivider(road);
   arms.set(other.id,{dx:dx/len,dy:dy/len,len,road,offset:{x:-(rb.y-ra.y)/rl*divider,y:(rb.x-ra.x)/rl*divider}});
  }
  const pair=[...arms.values()];if(pair.length!==2)continue;const [a,b]=pair;
  if((a.road.groupId??a.road.id)===(b.road.groupId??b.road.id)||totalLanes(a.road)!==totalLanes(b.road))continue;
  const center={x:n.x,y:n.y-n.z*28},reach=Math.min(9,a.len*.45,b.len*.45),p=arm=>({x:center.x+arm.offset.x+arm.dx*reach,y:center.y+arm.offset.y+arm.dy*reach});
  const p1=p(a),p2=p(b),o1={x:center.x+a.offset.x,y:center.y+a.offset.y},o2={x:center.x+b.offset.x,y:center.y+b.offset.y};
  line({x:o1.x-a.dx*2,y:o1.y-a.dy*2},p1,4,'#fff');line({x:o2.x-b.dx*2,y:o2.y-b.dy*2},p2,4,'#fff');
  const den=a.dx*b.dy-a.dy*b.dx;let control={x:(o1.x+o2.x)/2,y:(o1.y+o2.y)/2};if(Math.abs(den)>.05){let t=((o2.x-o1.x)*b.dy-(o2.y-o1.y)*b.dx)/den;control={x:o1.x+t*a.dx,y:o1.y+t*a.dy}}
  ctx.beginPath();ctx.moveTo(p1.x,p1.y);ctx.quadraticCurveTo(control.x,control.y,p2.x,p2.y);ctx.strokeStyle='#e3c840';ctx.lineWidth=1.3;ctx.stroke();
 }
}
// Cut the old segment outlines out of a taper before drawing its surface.
function clipLaneTapers(nodes,level,group){
 for(const n of nodes){
  if(n.junction||n.degree!==2||Math.min(2,Math.round(n.z))!==level)continue;
  const arms=new Map();for(const e of [...n.incoming,...n.out]){const other=sim.graph.nodes.get(e.from===n.id?e.to:e.from),r=sim.graph.roadById.get(e.road);if(!other||!r)continue;const dx=other.x-n.x,dy=(other.y-other.z*28)-(n.y-n.z*28),len=Math.hypot(dx,dy);if(len>.01&&!arms.has(other.id))arms.set(other.id,{dx:dx/len,dy:dy/len,len,r})}
  const pair=[...arms.values()];if(pair.length!==2)continue;const [a,b]=pair,wa=14+8*totalLanes(a.r),wb=14+8*totalLanes(b.r);
  if(wa===wb||a.dx*b.dx+a.dy*b.dy>-.94||![a.r,b.r].some(r=>(r.groupId??r.id)===group))continue;
  const reach=Math.min(32,a.len*.7,b.len*.7);if(reach<2)continue;
  const center={x:n.x,y:n.y-n.z*28},normal={x:-a.dy,y:a.dx},half=Math.max(wa,wb)/2+4,pa={x:center.x+a.dx*reach,y:center.y+a.dy*reach},pb={x:center.x+b.dx*reach,y:center.y+b.dy*reach};
  ctx.beginPath();ctx.rect(-4096,-4096,8192,8192);ctx.moveTo(pa.x+normal.x*half,pa.y+normal.y*half);ctx.lineTo(pb.x+normal.x*half,pb.y+normal.y*half);ctx.lineTo(pb.x-normal.x*half,pb.y-normal.y*half);ctx.lineTo(pa.x-normal.x*half,pa.y-normal.y*half);ctx.closePath();ctx.clip('evenodd');
 }
}
// Keep center and lane markings out of the junction without painting over road edges.
function clipJunctionMarkings(level){
 for(const node of sim.graph.nodes.values()){
  if(!node.junction||node.mergeGroups.size||Math.min(2,Math.round(node.z))!==level)continue;
  const roads=[...node.incoming,...node.out].map(e=>sim.graph.roadById.get(e.road)).filter(Boolean);
  if(!roads.length||roads.some(r=>r.roundabout))continue;
  const radius=Math.max(...roads.map(r=>(14+8*totalLanes(r))/2))-1;
  ctx.beginPath();ctx.rect(-4096,-4096,8192,8192);ctx.moveTo(node.x+radius,node.y-node.z*28);ctx.arc(node.x,node.y-node.z*28,radius,0,Math.PI*2);ctx.clip('evenodd');
 }
}
function drawRoadGroups(roads, shadow=false, level=null, isolated=false, renderPass=null, surfaceOnly=false){
 if(!roads.length)return;
 const taperNodes=shadow||isolated?[]:[...sim.graph.nodes.values()].filter(n=>!n.junction&&n.degree===2&&new Set([...n.incoming,...n.out].map(e=>sim.graph.roadById.get(e.road)).filter(Boolean).map(totalLanes)).size>1);
 const groups=new Map();for(const r of roads){let key=r.groupId||r.id;if(!groups.has(key))groups.set(key,[]);groups.get(key).push(r)}
 const sets=[];for(const list of groups.values()){let pending=list.slice();const compatible=(a,b)=>distance(a.b,b.a)<.1&&laneCount(a,0)===laneCount(b,0)&&laneCount(a,1)===laneCount(b,1);while(pending.length){let start=pending.findIndex(seg=>!pending.some(other=>other!==seg&&compatible(other,seg)));if(start<0)start=0;let part=[pending.splice(start,1)[0]];for(;;){let i=pending.findIndex(seg=>compatible(part.at(-1),seg));if(i<0)break;part.push(pending.splice(i,1)[0])}sets.push(part)}}const circles=new Map();for(const list of sets){if(!list[0].roundabout)continue;let points=[zPoint(list[0],list[0].a,0),...list.map(seg=>zPoint(seg,seg.b,1))],circle=circleFromClosedPath(points);if(circle){circles.set(list,circle);ringCircles.set(list[0].groupId||list[0].id,{...circle,y:circle.y+(list[0].z??0)*28})}}ctx.lineJoin='round';ctx.lineCap='butt';
 function path(list,offset=0){ctx.beginPath();let circle=circles.get(list);if(circle){let a=zPoint(list[0],list[0].a,0),b=zPoint(list[0],list[0].b,1),radius=circularOffsetRadius(circle,a,b,offset);if(radius>0){ctx.arc(circle.x+(shadow?7:0),circle.y+(shadow?8:0),radius,0,Math.PI*2);return}}let pts=[zPoint(list[0],list[0].a,0),...list.map(seg=>zPoint(seg,seg.b,1))];for(let i=0;i<pts.length;i++){let prev=pts[Math.max(0,i-1)],next=pts[Math.min(pts.length-1,i+1)],dx=next.x-prev.x,dy=next.y-prev.y,len=Math.hypot(dx,dy)||1,p=offsetPoint(pts[i],{x:-dy/len,y:dx/len},offset);let extension=i===0?-1:i===pts.length-1?1:0;p.x+=dx/len*extension;p.y+=dy/len*extension;if(!i)ctx.moveTo(p.x+(shadow?7:0),p.y+(shadow?8:0));else ctx.lineTo(p.x+(shadow?7:0),p.y+(shadow?8:0))}}

 for(const pass of renderPass===null?(shadow?[0]:[0,1,2]):[renderPass]){if(!shadow&&!isolated&&pass<2)drawDeadEnds(roads,level??Math.min(2,Math.round(sets[0]?.[0]?.z??0)),pass);if(!shadow&&!isolated&&pass<2){for(const n of sim.graph.nodes.values()){if(n.junction||n.degree!==2||n.roads.size<2||Math.min(2,Math.round(n.z))!==(level??Math.min(2,Math.round((sets[0]?.[0]?.z??0)))))continue;let roads=[...n.roads].map(id=>world.roads.find(r=>(r.groupId||r.id)===id)).filter(Boolean);if(roads.length<2||roads.some(r=>(r.z??0)!==(r.zEnd??r.z??0))||new Set(roads.map(r=>totalLanes(r))).size>1)continue;let width=Math.max(...roads.map(r=>14+8*totalLanes(r)));ctx.fillStyle=pass===0?'#979996':'#fff';ctx.beginPath();ctx.arc(n.x,n.y-n.z*28,width/2+(pass===0?1.5:0),0,Math.PI*2);ctx.fill()}}for(const list of sets){let r=list[0],f=laneCount(r,0),b=laneCount(r,1),width=14+Math.max(1,f+b)*8;ctx.save();if(!shadow&&!isolated)clipLaneTapers(taperNodes,level??Math.min(2,Math.round(r.z??0)),r.groupId??r.id);ctx.setLineDash([]);if(pass===2&&!shadow&&!isolated)clipJunctionMarkings(level??Math.min(2,Math.round(r.z??0)));if(pass<2){path(list);ctx.strokeStyle=shadow?'#53595730':pass===0?(r.oneway&&!r.roundabout?'#303431':'#979996'):'#fff';ctx.lineWidth=width+(shadow?8:pass===0?3:0);ctx.stroke();ctx.restore();continue}let divider=laneDivider(r);if(f&&b){path(list,divider);ctx.strokeStyle='#e3c840';ctx.lineWidth=1.3;ctx.stroke()}ctx.strokeStyle='#aaaead';ctx.lineWidth=1;ctx.setLineDash([5,8]);for(let i=1;i<f;i++){path(list,laneBoundary(r,0,i));ctx.stroke()}for(let i=1;i<b;i++){path(list,laneBoundary(r,1,i));ctx.stroke()}ctx.setLineDash([]);if((!f||!b)&&!r.roundabout){let accumulated=0,nextArrow=Math.min(75,list.reduce((sum,seg)=>sum+distance(zPoint(seg,seg.a,0),zPoint(seg,seg.b,1)),0)/2);for(const seg of list){let a=zPoint(seg,seg.a,0),z=zPoint(seg,seg.b,1),dx=z.x-a.x,dy=z.y-a.y,len=Math.hypot(dx,dy);if(len<.001)continue;while(accumulated+len>=nextArrow){let t=(nextArrow-accumulated)/len;ctx.save();ctx.translate(a.x+dx*t,a.y+dy*t);ctx.rotate(Math.atan2(dy,dx)+(f?0:Math.PI));ctx.fillStyle='#a2aaa6';ctx.beginPath();ctx.moveTo(4.5,0);ctx.lineTo(-2,-3);ctx.lineTo(-2,-1);ctx.lineTo(-5,-1);ctx.lineTo(-5,1);ctx.lineTo(-2,1);ctx.lineTo(-2,3);ctx.closePath();ctx.fill();ctx.restore();nextArrow+=220}accumulated+=len}}ctx.restore()}
 if(!shadow&&!isolated){let layer=level??Math.min(2,Math.round(sets[0]?.[0]?.z??0));drawLaneTransitions(layer,pass,surfaceOnly);if(pass<2)drawJunctionWidthFlares(layer,pass);if(pass===1)drawMergeFillets(layer,surfaceOnly);if(pass<2)for(const node of sim.graph.nodes.values())if(node.junction&&!node.mergeGroups.size&&Math.min(2,Math.round(node.z))===layer)drawJunctionFillets(node,pass);if(pass===2)drawBendMarkings(layer)}
 }
}
// Redraw only the higher road around fractional-height crossings within a layer.
function drawProjectedOverpasses(roads,patches,level){
 for(const patch of patches){
  // The patch must not repaint a highway over its own connected ramp mouth.
  const connected=[...sim.graph.nodes.values()].some(node=>{
   if(!node.junction||!node.roads.has(patch.group)||!node.roads.has(patch.other))return false;
   const incident=[...node.incoming,...node.out].map(e=>sim.graph.roadById.get(e.road)).filter(Boolean);
   const reach=Math.max(20,...incident.map(road=>(14+8*totalLanes(road))/2))+16;
   return Math.hypot(node.x-patch.x,node.y-node.z*28-patch.y)<reach+patch.radius;
  });
  if(connected)continue;
  const minHeight=(patch.high+patch.low)/2;
  const upper=roads.filter(r=>(r.groupId||r.id)===patch.group&&Math.max(r.z??0,r.zEnd??r.z??0)>=minHeight);
  if(!upper.length)continue;
  ctx.save();ctx.beginPath();ctx.arc(patch.x,patch.y,patch.radius,0,Math.PI*2);ctx.clip();
  drawRoadGroups(upper,false,level,true);ctx.restore();
 }
}
// Round the two outside corners where a same-level slip road meets a wide road.
// Sampling the actual curved branch avoids extrapolating its tangent at the node.
function drawMergeFillets(level,surfaceOnly=false){
 const graph=sim.graph;
 for(const node of graph.nodes.values()){
  if(!node.mergeGroups.size||Math.min(2,Math.round(node.z))!==level)continue;
  let incident=[...node.incoming,...node.out];
  for(const mainGroup of node.mergeGroups){
   let mainEdge=incident.find(e=>{let r=graph.roadById.get(e.road);return (r?.groupId??r?.id)===mainGroup}),main=mainEdge&&graph.roadById.get(mainEdge.road);
   let other=mainEdge&&graph.nodes.get(mainEdge.from===node.id?mainEdge.to:mainEdge.from);
   if(!main||!other)continue;
   const center={x:node.x,y:node.y-node.z*28},direction={x:other.x-node.x,y:(other.y-other.z*28)-center.y};
   const seen=new Set();
   for(const e of incident){
    let branch=graph.roadById.get(e.road),group=branch?.groupId??branch?.id;
    if(!branch||group===mainGroup||seen.has(group)||!branch.oneway||totalLanes(branch)!==1||branch.roundabout)continue;
    seen.add(group);let prev=node,current=graph.nodes.get(e.from===node.id?e.to:e.from),samples=[center],walked=0;
    while(current&&walked<220&&samples.length<100&&Math.abs(current.z-node.z)<.01){
     let p={x:current.x,y:current.y-current.z*28};walked+=distance(samples.at(-1),p);samples.push(p);
     if(current.junction)break;
     let candidates=[...current.incoming,...current.out].map(edge=>{let r=graph.roadById.get(edge.road),next=graph.nodes.get(edge.from===current.id?edge.to:edge.from);return {r,next}}).filter(x=>x.next&&x.next.id!==prev.id&&(x.r?.groupId??x.r?.id)===group);
     let next=candidates[0]?.next;if(!next)break;prev=current;current=next;
    }
    const corners=mergeFillets(center,direction,(14+8*totalLanes(main))/2,(14+8*totalLanes(branch))/2,samples);
    for(const {start,control,end} of corners){
     if(!surfaceOnly){ctx.lineCap='butt';ctx.lineJoin='round';ctx.strokeStyle='#fff';ctx.lineWidth=4;ctx.beginPath();ctx.moveTo(start.x,start.y);ctx.lineTo(control.x,control.y);ctx.lineTo(end.x,end.y);ctx.stroke();}
     ctx.fillStyle='#fff';ctx.beginPath();ctx.moveTo(start.x,start.y);ctx.lineTo(control.x,control.y);ctx.lineTo(end.x,end.y);ctx.quadraticCurveTo(control.x,control.y,start.x,start.y);ctx.closePath();ctx.fill();
     if(!surfaceOnly){ctx.strokeStyle=main.oneway?'#303431':'#979996';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(start.x,start.y);ctx.quadraticCurveTo(control.x,control.y,end.x,end.y);ctx.stroke();}
    }
   }
  }
 }
}
// Blend unequal opposing road widths into the junction surface before its
// rounded corners are filled. The final outline follows the union of both.
function drawJunctionWidthFlares(level,pass){
 ctx.fillStyle=pass===0?'#979996':'#fff';
 for(const node of sim.graph.junctionNodes){
  if(Math.min(2,Math.round(node.z))!==level)continue;
  for(const flare of junctionWidthFlares(sim.graph,node)){
   ctx.beginPath();
   for(const side of [1,-1]){
    const sections=side===1?flare.sections:[...flare.sections].reverse();
    for(let i=0;i<sections.length;i++){const p=sections[i],x=p.x-p.dy*p.width*side,y=p.y+p.dx*p.width*side;if(side===1&&i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)}
   }
   ctx.closePath();ctx.fill();
  }
 }
}
// Fill the concave corners between incident road surfaces with small curved
// fillets. Draw their outer edge separately so road borders do not cross the join.
function drawJunctionFillets(node,pass){
 const arms=[],flares=new Map(junctionWidthFlares(sim.graph,node).map(flare=>[flare.otherId,flare]));
 for(const e of [...node.incoming,...node.out]){const other=sim.graph.nodes.get(e.from===node.id?e.to:e.from),road=sim.graph.roadById.get(e.road);if(!other||!road)continue;let dx=other.x-node.x,dy=other.y-node.y,len=Math.hypot(dx,dy);if(len<.01)continue;dx/=len;dy/=len;let angle=Math.atan2(dy,dx);if(angle<0)angle+=Math.PI*2;if(arms.some(a=>Math.abs(Math.atan2(Math.sin(a.angle-angle),Math.cos(a.angle-angle)))<.06))continue;arms.push({dx,dy,angle,width:flares.get(other.id)?.wide??(14+8*totalLanes(road))/2,flare:flares.get(other.id),roundabout:!!road.roundabout,road,other})}
 if(arms.length<3)return;arms.sort((a,b)=>a.angle-b.angle);const center={x:node.x,y:node.y-node.z*28};
 // Follow the actual approach, including short segments that make up a curve.
 // The junction corner must meet that outline at its real position, not the
 // extrapolated tangent at the intersection.
 function trace(arm,side,reach){
  const group=arm.road.groupId??arm.road.id,points=[];let prev=node,current=arm.other,walked=0;
  for(let steps=0;current&&steps<100;steps++){
   const a={x:prev.x,y:prev.y-prev.z*28},b={x:current.x,y:current.y-current.z*28},dx=b.x-a.x,dy=b.y-a.y,len=Math.hypot(dx,dy);if(len<.01)break;
   const nx=-dy/len*side,ny=dx/len*side,remaining=Math.max(0,reach-walked),take=Math.min(len,remaining);
   if(!points.length)points.push({x:a.x+nx*arm.width,y:a.y+ny*arm.width});
   const width=arm.flare?flareHalfWidth(arm.flare,walked+take):arm.width;
   points.push({x:a.x+dx/len*take+nx*width,y:a.y+dy/len*take+ny*width});
   walked+=take;if(take<len-.001||walked>=reach-.001||current.junction)break;
   const next=[...current.incoming,...current.out].map(e=>({road:sim.graph.roadById.get(e.road),node:sim.graph.nodes.get(e.from===current.id?e.to:e.from)})).find(v=>v.node&&v.node.id!==prev.id&&(arm.flare||(v.road?.groupId??v.road?.id)===group));
   prev=current;current=next?.node;
  }
  return points;
 }
 for(let i=0;i<arms.length;i++){let a=arms[i],b=arms[(i+1)%arms.length],gap=(b.angle-a.angle+Math.PI*2)%(Math.PI*2);if(gap<.45||gap>Math.PI-.12||a.roundabout&&b.roundabout)continue;let na={x:-a.dy,y:a.dx},nb={x:b.dy,y:-b.dx},ea={x:na.x*a.width,y:na.y*a.width},eb={x:nb.x*b.width,y:nb.y*b.width},den=a.dx*b.dy-a.dy*b.dx;if(Math.abs(den)<.1)continue;let tx=((eb.x-ea.x)*b.dy-(eb.y-ea.y)*b.dx)/den,uy=((eb.x-ea.x)*a.dy-(eb.y-ea.y)*a.dx)/den;if(tx<0||uy<0||tx>90||uy>90)continue;let curve=Math.min(9,a.width*.6,b.width*.6),p={x:center.x+ea.x+a.dx*(tx+curve),y:center.y+ea.y+a.dy*(tx+curve)},q={x:center.x+eb.x+b.dx*(uy+curve),y:center.y+eb.y+b.dy*(uy+curve)},c={x:center.x+ea.x+a.dx*tx,y:center.y+ea.y+a.dy*tx};let pa=trace(a,1,tx+curve),pb=trace(b,-1,uy+curve);if(pa.length>1)p=pa.at(-1);if(pb.length>1)q=pb.at(-1);
 ctx.fillStyle=pass===0?'#979996':'#fff';ctx.beginPath();ctx.moveTo(center.x,center.y);ctx.lineTo(p.x,p.y);ctx.quadraticCurveTo(c.x,c.y,q.x,q.y);ctx.closePath();ctx.fill();if(pass===0){ctx.strokeStyle='#979996';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.quadraticCurveTo(c.x,c.y,q.x,q.y);ctx.stroke()}}
}
// Signal lamps use the same phase calculation as the simulation.
function drawTrafficSignals(level){
 for(const node of sim.graph.junctionNodes){
  const cfg=world.junctions[node.id];if(!node.junction||cfg?.mode!=='signal'||Math.min(2,Math.round(node.z))!==level)continue;
  const width=node.width;
  for(const edge of node.incoming){
   const from=sim.graph.nodes.get(edge.from);if(!from)continue;
   const dx=node.x-from.x,dy=node.y-from.y,len=Math.hypot(dx,dy);if(!len)continue;
   const state=signalState(sim.time,cfg.phase,Math.abs(dx)>=Math.abs(dy),cfg.offset),distance=width/2+9;
   const x=node.x-dx/len*distance-dy/len*(width/2+4),y=node.y-node.z*28-dy/len*distance+dx/len*(width/2+4);
   if(!inView(x,y))continue;
   ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fillStyle='#34433e';ctx.fill();
   ctx.beginPath();ctx.arc(x,y,2.5,0,Math.PI*2);ctx.fillStyle=state==='red'?'#e96255':state==='amber'?'#f5bc51':'#65c77d';ctx.fill();
  }
 }
}
function drawJunctionMarkings(level=0){for(const n of sim.graph.nodes.values()){if(!n.junction||Math.min(2,Math.round(n.z))!==level)continue;let connected=[...n.roads].map(id=>world.roads.find(r=>(r.groupId||r.id)===id)).filter(Boolean);if(n.mergeGroups.size&&world.junctions[n.id]?.mode!=='signal')continue;let y=n.y-n.z*28,w=Math.max(...[...n.roads].map(id=>{let r=world.roads.find(x=>(x.groupId||x.id)===id);return r?14+8*totalLanes(r):26}));let ring=connected.find(r=>r.roundabout);if(ring){let segments=world.roads.filter(r=>r.groupId===ring.groupId&&r.roundabout);if(segments.length>=8&&distance(segments[0].a,segments.at(-1).b)<.1){let center=segments.reduce((c,r)=>({x:c.x+r.a.x/segments.length,y:c.y+r.a.y/segments.length}),{x:0,y:0}),radius=segments.reduce((sum,r)=>sum+distance(center,r.a)/segments.length,0),inner=radius-(14+totalLanes(ring)*8)/2;ctx.save();ctx.beginPath();ctx.rect(0,-100,SIZE,SIZE+200);ctx.moveTo(center.x+inner,center.y-n.z*28);ctx.arc(center.x,center.y-n.z*28,Math.max(0,inner),0,Math.PI*2);ctx.clip('evenodd');ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(n.x,y,w/2+3,0,Math.PI*2);ctx.fill();ctx.restore()}}let seen=new Set();for(const e of n.incoming){let other=sim.graph.nodes.get(e.from),rd=sim.graph.roadById.get(e.road);if(!other||!rd||rd.roundabout)continue;let dx=n.x-other.x,dy=(n.y-n.z*28)-(other.y-other.z*28),angle=Math.round(Math.atan2(dy,dx)*100),key=(rd.groupId||rd.id)+':'+angle;if(seen.has(key))continue;seen.add(key);let mark=approachStopLine({x:n.x,y},{x:other.x,y:other.y-other.z*28},e.lanes,laneCount(rd,1-e.direction),w);if(!mark)continue;ctx.beginPath();ctx.moveTo(mark.a.x,mark.a.y);ctx.lineTo(mark.b.x,mark.b.y);ctx.strokeStyle='#aaa';ctx.lineWidth=1.5;ctx.setLineDash([3,3]);ctx.stroke();ctx.setLineDash([]);}}}

const ringCircles=new Map();
let roadDepthIndex=[new Map(),new Map(),new Map()];
const staticCanvas=document.createElement('canvas'),upperCanvases=[document.createElement('canvas'),document.createElement('canvas')];let cacheGraph=null;
// Keep the static map in world coordinates so pan and zoom only composite it.
const mapCacheScale=2, mapCachePixels=SIZE*mapCacheScale;
let cachedBitmaps=null,bitmapGeneration=0,activeRoadLayers=[true,true,true],buildingAccessCache=new Map(),visibleBounds=null;
function snapshotMapLayers(){
 const generation=++bitmapGeneration;
 if(cachedBitmaps){for(const bitmap of cachedBitmaps)bitmap?.close();cachedBitmaps=null}
 if(typeof createImageBitmap!=='function')return;
 Promise.all([staticCanvas,...upperCanvases].map((layer,i)=>activeRoadLayers[i]?createImageBitmap(layer):Promise.resolve(null))).then(images=>{if(generation!==bitmapGeneration){for(const bitmap of images)bitmap?.close();return}cachedBitmaps=images}).catch(()=>{});
}
function drawCachedLayer(layer){
 if(!activeRoadLayers[layer])return;const {left,top,right,bottom}=visibleBounds;
 const x=Math.max(0,left),y=Math.max(0,top),endX=Math.min(SIZE,right),endY=Math.min(SIZE,bottom),w=endX-x,h=endY-y;if(w<=0||h<=0)return;
 const source=cachedBitmaps?.[layer]??(layer===0?staticCanvas:upperCanvases[layer-1]);
 ctx.drawImage(source,x*mapCacheScale,y*mapCacheScale,w*mapCacheScale,h*mapCacheScale,x,y,w,h);
}
function inView(x,y,margin=30){return !visibleBounds||(x>=visibleBounds.left-margin&&x<=visibleBounds.right+margin&&y>=visibleBounds.top-margin&&y<=visibleBounds.bottom+margin)}
function prepareMapLayer(layer,r){
 if(layer.width!==mapCachePixels||layer.height!==mapCachePixels){layer.width=mapCachePixels;layer.height=mapCachePixels}
 const layerContext=layer.getContext('2d'),sx=layer.width/r.width,sy=layer.height/r.height;
 layerContext.setTransform(1,0,0,1,0,0);layerContext.clearRect(0,0,layer.width,layer.height);
 layerContext.setTransform(sx*cam.zoom,0,0,sy*cam.zoom,sx*(r.width/2-cam.x*cam.zoom),sy*(r.height/2-cam.y*cam.zoom));
 return layerContext;
}
// Composite a single outline around the union of road surfaces. Drawing each
// road border separately leaves seams wherever widened roads and fillets meet.
const roadSurfaceCanvas=document.createElement('canvas'),roadOutlineCanvas=document.createElement('canvas');
function drawRoadSurface(roads,level,r){
 if(!roads.length)return;
 const target=ctx;
 ctx=prepareMapLayer(roadSurfaceCanvas,r);
 drawRoadGroups(roads,false,level,false,1,true);
 if(roadOutlineCanvas.width!==mapCachePixels||roadOutlineCanvas.height!==mapCachePixels){roadOutlineCanvas.width=mapCachePixels;roadOutlineCanvas.height=mapCachePixels}
 const outline=roadOutlineCanvas.getContext('2d');outline.setTransform(1,0,0,1,0,0);outline.clearRect(0,0,mapCachePixels,mapCachePixels);
 const rx=1.5*cam.zoom*mapCachePixels/r.width,ry=1.5*cam.zoom*mapCachePixels/r.height;
 for(let i=0;i<32;i++){const a=i*Math.PI*2/32;outline.drawImage(roadSurfaceCanvas,Math.cos(a)*rx,Math.sin(a)*ry)}
 outline.globalCompositeOperation='source-in';outline.fillStyle='#979996';outline.fillRect(0,0,mapCachePixels,mapCachePixels);
 // Layer cuts are internal points on a sloping road, not road ends. Remove
 // the transverse outline there while leaving the side outlines intact.
 const sx=mapCachePixels/r.width,sy=mapCachePixels/r.height;
 outline.setTransform(sx*cam.zoom,0,0,sy*cam.zoom,sx*(r.width/2-cam.x*cam.zoom),sy*(r.height/2-cam.y*cam.zoom));
 outline.globalCompositeOperation='destination-out';outline.strokeStyle='#000';outline.lineCap='butt';
 for(const road of roads){
  if(Math.abs((road.zEnd??road.z??0)-(road.z??0))<1e-8)continue;
  const a=zPoint(road,road.a,0),b=zPoint(road,road.b,1),length=Math.hypot(b.x-a.x,b.y-a.y);if(length<.001)continue;
  const dx=(b.x-a.x)/length,dy=(b.y-a.y)/length;
  for(const [point,z,original,cutInside,otherZ] of [[a,road.z??0,road.a,road.layerCutStart,road.zEnd??road.z??0],[b,road.zEnd??road.z??0,road.b,road.layerCutEnd,road.z??0]]){
   if(![.5,1.5].some(cut=>Math.abs(z-cut)<1e-7))continue;
   // Also handle a layer boundary coinciding with an existing segment joint.
   const continues=cutInside||world.roads.some(other=>{
    if(other.id===road.id||(other.groupId??other.id)!==(road.groupId??road.id))return false;
    return [[other.a,other.z??0,other.zEnd??other.z??0],[other.b,other.zEnd??other.z??0,other.z??0]].some(([p,level,next])=>distance(p,original)<.01&&Math.abs(level-z)<1e-7&&(otherZ-z)*(next-z)<0);
   });
   if(!continues)continue;
   outline.lineWidth=Math.max(1,14+8*totalLanes(road)-.5);outline.beginPath();outline.moveTo(point.x-dx*4,point.y-dy*4);outline.lineTo(point.x+dx*4,point.y+dy*4);outline.stroke();
  }
 }
 outline.setTransform(1,0,0,1,0,0);outline.globalCompositeOperation='source-over';
 target.save();target.setTransform(1,0,0,1,0,0);target.drawImage(roadOutlineCanvas,0,0);target.drawImage(roadSurfaceCanvas,0,0);target.restore();
 ctx=target;
}
// Roads only share a surface near graph connections. Separate surfaces keep
// parallel roads distinct even when their painted footprints overlap.
function drawUnifiedRoads(roads,level,r){
 if(!roads.length)return;
 drawRoadSurface(roads,level,r);
 drawRoadGroups(roads,false,level,false,2);
}
// Restore boundaries only where two unrelated, near-parallel road surfaces
// overlap in projection. The masks reuse each road's actual canvas outline.
const roadBoundaryCanvas=document.createElement('canvas');
function drawNearbyRoadBoundaries(r){
 const boundaries=nearbyRoadBoundaries(world.roads,sim.graph),byGroup=new Map(),levels=new Set();
 for(const boundary of boundaries){if(!byGroup.has(boundary.group))byGroup.set(boundary.group,[]);byGroup.get(boundary.group).push(boundary)}
 const original=ctx;
 for(const [group,entries] of byGroup){
  const roads=world.roads.filter(road=>(road.groupId??road.id)===group);
  ctx=prepareMapLayer(roadBoundaryCanvas,r);
  drawRoadGroups(roads,false,null,true,0,true);
  ctx.globalCompositeOperation='destination-out';drawRoadGroups(roads,false,null,true,1,true);
  ctx.globalCompositeOperation='source-in';ctx.fillStyle='#979996';ctx.fillRect(-100,-100,SIZE+200,SIZE+200);ctx.globalCompositeOperation='source-over';
  for(const entry of entries){
   const target=entry.level===0?staticCanvas:upperCanvases[entry.level-1];
   const out=target.getContext('2d');out.save();out.beginPath();
   for(const clip of entry.clips){
    const dx=clip.b.x-clip.a.x,dy=clip.b.y-clip.a.y,len=Math.hypot(dx,dy);if(len<.01)continue;
    const ux=dx/len,uy=dy/len,nx=-uy*clip.half,ny=ux*clip.half;
    const ax=clip.a.x-ux*2,ay=clip.a.y-uy*2,bx=clip.b.x+ux*2,by=clip.b.y+uy*2;
    out.moveTo(ax+nx,ay+ny);out.lineTo(bx+nx,by+ny);out.lineTo(bx-nx,by-ny);out.lineTo(ax-nx,ay-ny);out.closePath();
   }
   out.clip();
   for(const node of entry.joins){out.beginPath();out.rect(-4096,-4096,8192,8192);out.moveTo(node.x+65,node.y-(node.z??0)*28);out.arc(node.x,node.y-(node.z??0)*28,65,0,Math.PI*2);out.clip('evenodd')}
   out.setTransform(1,0,0,1,0,0);out.drawImage(roadBoundaryCanvas,0,0);out.restore();levels.add(entry.level);
  }
 }
 ctx=original;return levels;
}
function carHeading(c){const vector=e=>{if(!e)return null;let a=sim.graph.nodes.get(e.from),b=sim.graph.nodes.get(e.to);if(!a||!b)return null;let dx=b.x-a.x,dy=b.y-a.y,l=Math.hypot(dx,dy)||1;return {x:dx/l,y:dy/l}};let current=vector(c.edge),prev=vector(c.path[c.index-1]),next=vector(c.path[c.index+1]);if(!current)return {x:1,y:0};let x=current.x,y=current.y,span=16;if(prev&&c.pos<span){let w=(1-c.pos/span)*.5;x=x*(1-w)+prev.x*w;y=y*(1-w)+prev.y*w}let remaining=c.edge.length-c.pos;if(next&&remaining<span){let w=(1-remaining/span)*.5;x=x*(1-w)+next.x*w;y=y*(1-w)+next.y*w}let l=Math.hypot(x,y)||1;return {x:x/l,y:y/l}}
// Bend the rendered vehicle path through a junction while the simulation
// continues to measure progress along the underlying road edges.
function carCenter(c,t){
 let edge=c.edge,a=sim.graph.nodes.get(edge.from),b=sim.graph.nodes.get(edge.to);if(!a||!b)return {x:0,y:0};
 let position=t*edge.length,previous=c.path[c.index-1],next=c.path[c.index+1];
 const unit=e=>{if(!e)return null;let from=sim.graph.nodes.get(e.from),to=sim.graph.nodes.get(e.to);if(!from||!to)return null;let dx=to.x-from.x,dy=to.y-from.y,len=Math.hypot(dx,dy)||1;return {x:dx/len,y:dy/len}};
 const current=unit(edge),quadratic=(node,incoming,outgoing,r,u)=>{let p0={x:node.x-incoming.x*r,y:node.y-incoming.y*r},p2={x:node.x+outgoing.x*r,y:node.y+outgoing.y*r},v=1-u;return {x:v*v*p0.x+2*v*u*node.x+u*u*p2.x,y:v*v*p0.y+2*v*u*node.y+u*u*p2.y}};
 if(previous&&a.junction){let incoming=unit(previous),r=Math.min(18,previous.length/2,edge.length/2);if(incoming&&r>1&&position<r&&incoming.x*current.x+incoming.y*current.y<.98)return quadratic(a,incoming,current,r,(r+position)/(2*r))}
 if(next&&b.junction){let outgoing=unit(next),r=Math.min(18,edge.length/2,next.length/2),remaining=edge.length-position;if(outgoing&&r>1&&remaining<r&&current.x*outgoing.x+current.y*outgoing.y<.98)return quadratic(b,current,outgoing,r,(r-remaining)/(2*r))}
 let road=sim.graph.roadById.get(edge.road),circle=road?.roundabout&&ringCircles.get(road.groupId||road.id);if(circle)return pointOnCircularEdge(circle,a,b,t);
 return {x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t};
}
function vehicleAt(point){
 let best=null;
 for(const car of sim.cars){const pose=vehiclePoses.get(car.id);if(!pose)continue;const d=distance(point,pose),radius=Math.max(9,10/cam.zoom);if(d<radius&&(!best||d<best.d))best={car,d}}
 return best?.car;
}
function carTurn(car){
 const metrics=pathMetrics(sim.graph,car.path),i=metrics.junctions[car.index];
 if(i<0||metrics.starts[i+1]-metrics.starts[car.index]-car.pos>60)return null;
 const turn=turnType(car.path[i],car.path[i+1]);return turn==='straight'?null:turn;
}
function vehicleInspector(panel){
 if(selected?.type!=='vehicle')return;
 const car=sim.cars.find(c=>c.id===selected.id),parked=sim.parked.find(c=>c.id===selected.id);
 const close='<button id="closeVehicle" class="inspector-close" aria-label="Close vehicle inspection">×</button>';
 if(!car){panel.innerHTML=`${close}<h3>Vehicle ${selected.id}</h3><p>${parked?'Arrived and parked.':'This vehicle has left the road network.'}</p>`}
 else{
  const building=world.buildings.find(b=>b.id===car.destinationId),destination=building?`${building.type[0].toUpperCase()+building.type.slice(1)} ${world.buildings.indexOf(building)+1}`:world.ports.some(p=>p.id===car.destinationId)?'Map exit':'Road destination';
  const status=car.routeBlocked?'No available route':car.speed<3?(car.waitReason||'Waiting for space'):car.laneChange?'Changing lanes':car.acceleration<-6?'Slowing down':'Driving';
  const turn=carTurn(car),maneuver=car.laneChange?`Lane ${car.laneChange.from+1} → ${car.laneChange.to+1}`:turn?`Turn ${turn} ahead`:`Lane ${car.lane+1}`;
  const html=`${close}<span class="inspector-eyebrow">LIVE VEHICLE</span><h3>${(car.length??10)>12?'Truck':'Car'} ${car.id}</h3><div class="vehicle-state">${status}</div><dl class="vehicle-details"><dt>Destination</dt><dd>${destination}</dd><dt>Next action</dt><dd>${maneuver}</dd><dt>Parking</dt><dd>${building?(car.parkingSlot==null?'Waiting for a space':'Space reserved'):'Leaving the map'}</dd></dl><p class="route-key"><i></i> Planned route · arrows show direction</p><p>The route updates as this vehicle moves or its lane connections change.</p>`;
  if(panel.innerHTML!==html)panel.innerHTML=html;
 }
 panel.querySelector('#closeVehicle').onclick=()=>{selected=null;inspector()};
}
let selectedRouteCache=null;
function selectedRoute(){
 const car=selected?.type==='vehicle'&&sim.cars.find(c=>c.id===selected.id);
 if(!car||car.routeBlocked)return null;
 const key=`${car.id}|${car.index}|${car.lane}|${Math.floor(sim.time*4)}`;
 if(selectedRouteCache?.path!==car.path||selectedRouteCache?.graph!==sim.graph||selectedRouteCache?.key!==key)selectedRouteCache={key,path:car.path,graph:sim.graph,points:vehicleRoutePoints(sim.graph,car)};
 return {car,points:selectedRouteCache.points.filter(p=>p.edgeIndex>car.index||p.position>=car.pos)};
}
function drawVehicleRoute(layer){
 const route=selectedRoute();if(!route?.points.length)return;
 const {points}=route,layerOf=p=>Math.min(2,Math.max(0,Math.round(p.z)));
 ctx.save();ctx.lineCap='round';ctx.lineJoin='round';ctx.beginPath();
 let started=false;
 for(const p of points){if(layerOf(p)!==layer){started=false;continue}if(!started){ctx.moveTo(p.x,p.y);started=true}else ctx.lineTo(p.x,p.y)}
 ctx.lineWidth=7/cam.zoom;ctx.strokeStyle='#ffffffce';ctx.stroke();ctx.lineWidth=4/cam.zoom;ctx.strokeStyle='#168d88';ctx.stroke();
 let spacing=0;
 for(let i=1;i<points.length;i++){
  const a=points[i-1],b=points[i],d=distance(a,b);spacing+=d;
  if(layerOf(b)!==layer||spacing<60/cam.zoom||d<.1||!inView(b.x,b.y))continue;
  spacing=0;const dx=(b.x-a.x)/d,dy=(b.y-a.y)/d,r=4/cam.zoom;
  ctx.beginPath();ctx.moveTo(b.x-dx*r-dy*r,b.y-dy*r+dx*r);ctx.lineTo(b.x+dx*r,b.y+dy*r);ctx.lineTo(b.x-dx*r+dy*r,b.y-dy*r-dx*r);ctx.strokeStyle='#fff';ctx.lineWidth=1.5/cam.zoom;ctx.stroke();
 }
 const end=points.at(-1);
 if(layerOf(end)===layer){ctx.beginPath();ctx.arc(end.x,end.y,7/cam.zoom,0,Math.PI*2);ctx.fillStyle='#168d88';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2/cam.zoom;ctx.stroke();ctx.beginPath();ctx.arc(end.x,end.y,2/cam.zoom,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill()}
 ctx.restore();
}
function drawSelectedVehicle(){
 if(selected?.type!=='vehicle')return;
 const pose=vehiclePoses.get(selected.id);if(!pose)return;
 ctx.save();ctx.beginPath();ctx.arc(pose.x,pose.y,11/cam.zoom,0,Math.PI*2);ctx.strokeStyle='#168d88';ctx.lineWidth=2/cam.zoom;ctx.stroke();ctx.restore();
}
function drawPrioritySigns(level){
 for(const node of sim.graph.junctionNodes){
  if(Math.min(2,Math.round(node.z))!==level||!inView(node.x,node.y-node.z*28,70))continue;
  const cfg=world.junctions[node.id]||{};if(cfg.mode==='signal')continue;
  for(const edge of node.incoming){
   const rule=approachRule(sim.graph,node,edge,cfg);if(rule==='auto'||edge.roundabout)continue;
   const r=sim.graph.roadById.get(edge.road),off=travelLaneCenter(r,edge.direction,edge.lanes-1)+9;
   const x=node.x-edge.hx*node.stopDistance-edge.hy*off,y=node.y-node.z*28-edge.hy*node.stopDistance+edge.hx*off;
   ctx.save();ctx.translate(x,y);ctx.rotate(Math.atan2(edge.hy,edge.hx));ctx.beginPath();
   if(rule==='yield'){ctx.moveTo(-4,-4);ctx.lineTo(4,0);ctx.lineTo(-4,4);ctx.closePath();ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#c86a55';ctx.lineWidth=1.6;ctx.stroke()}
   else if(rule==='stop'){for(let i=0;i<8;i++){let a=Math.PI/8+i*Math.PI/4;ctx.lineTo(Math.cos(a)*4.5,Math.sin(a)*4.5)}ctx.closePath();ctx.fillStyle='#c86a55';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=1;ctx.stroke()}
   else{ctx.rect(-3,-3,6,6);ctx.fillStyle='#e0b654';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=1;ctx.stroke()}
   ctx.restore();
  }
 }
}
function drawWaitReason(){
 if(!mouse)return;
 let closest=null;
 for(const c of sim.cars){if(c.speed>=3||!c.waitReason)continue;const pose=vehiclePoses.get(c.id);if(!pose)continue;const d=distance(mouse,pose);if(d<14/cam.zoom&&(!closest||d<closest.d))closest={...pose,d,reason:c.waitReason}}
 if(!closest)return;ctx.save();ctx.font=`${12/cam.zoom}px sans-serif`;let pad=6/cam.zoom,w=ctx.measureText(closest.reason).width+pad*2,h=22/cam.zoom,x=closest.x-w/2,y=closest.y-31/cam.zoom;ctx.fillStyle='#263c36ee';ctx.fillRect(x,y,w,h);ctx.fillStyle='#fff';ctx.textAlign='left';ctx.textBaseline='middle';ctx.fillText(closest.reason,x+pad,y+h/2);ctx.restore();
}
const vehiclePoses=new Map();
function drawCars(layer){for(const c of sim.cars){let t=Math.min(1,(c.pos+(paused?0:c.speed*acc))/c.edge.length),level=c.edge.zFrom+(c.edge.zTo-c.edge.zFrom)*t;if(Math.min(2,Math.max(0,Math.round(level)))!==layer)continue;let a=sim.graph.nodes.get(c.edge.from),b=sim.graph.nodes.get(c.edge.to);if(!a||!b)continue;let heading=carHeading(c),dx=heading.x,dy=heading.y,rd=sim.graph.roadById.get(c.edge.road),f=rd?laneCount(rd,c.edge.direction):1,back=rd?laneCount(rd,1-c.edge.direction):1,progress=c.laneChange?Math.min(1,(c.laneChange.distance+(paused?0:c.speed*acc))/c.laneChange.length):0,eased=progress*progress*(3-2*progress),visualLane=c.laneChange?c.laneChange.from+(c.laneChange.to-c.laneChange.from)*eased:c.lane,off=rd?travelLaneCenter(rd,c.edge.direction,visualLane):(back-f)*4+(visualLane+.5)*8,side=1,z=(c.edge.zFrom+(c.edge.zTo-c.edge.zFrom)*t)*28,center=carCenter(c,t),x=center.x-dy*off*side,y=center.y+dx*off*side-z;let turn=!c.laneChange&&junctionVehiclePose(sim.graph,c.path,c.index,t*c.edge.length,c.lane);if(turn){x=turn.x;y=turn.y;dx=turn.dx;dy=turn.dy}vehiclePoses.set(c.id,{x,y,dx,dy,level});if(!inView(x,y))continue;ctx.save();for(const upper of roadsAboveVehicle(roadDepthIndex,layer,{x,y},level,c.edge.road,rd?.groupId??rd?.id,rd)){let points=roadFootprint(upper);if(points.length<3)continue;ctx.beginPath();ctx.rect(-4096,-4096,8192,8192);ctx.moveTo(points[0].x,points[0].y);for(let i=1;i<points.length;i++)ctx.lineTo(points[i].x,points[i].y);ctx.closePath();ctx.clip('evenodd')}ctx.translate(x,y);ctx.rotate(Math.atan2(dy,dx));let carLength=c.length??10;ctx.fillStyle='#39474255';ctx.fillRect(-carLength/2+1,-2,carLength,6);ctx.fillStyle=c.color;ctx.fillRect(-carLength/2,-3,carLength,6);ctx.strokeStyle='#39474299';ctx.lineWidth=.6;ctx.strokeRect(-carLength/2,-3,carLength,6);ctx.fillStyle='#263d49';ctx.fillRect(carLength/2-3,-2,1.5,4);ctx.fillStyle='#ffffff88';ctx.fillRect(-carLength/2+2,-2,Math.max(2,carLength-7),1);if(c.speed<3||c.acceleration<-6){ctx.fillStyle='#ed554c';ctx.fillRect(-carLength/2,-2.8,1.5,1.4);ctx.fillRect(-carLength/2,1.4,1.5,1.4)}if((c.laneChange||carTurn(c))&&Math.floor(sim.time*2)%2===0){ctx.fillStyle='#ffd468';let side=c.laneChange?Math.sign(c.laneChange.to-c.laneChange.from):carTurn(c)==='right'?1:-1;ctx.fillRect(carLength/2-2,side>0?2:-3.4,1.5,1.5)}ctx.restore()}}
function drawLaneSnap(target){
 const lane=target?.laneAnchor;if(!lane)return;
 const x=target.x+lane.normal.x*lane.offset,y=target.y-(target.z||0)*28+lane.normal.y*lane.offset,tx=lane.normal.y,ty=-lane.normal.x,length=14/cam.zoom;
 ctx.save();ctx.lineCap='round';ctx.strokeStyle='#48b6c9';ctx.lineWidth=7;ctx.beginPath();ctx.moveTo(x-tx*length,y-ty*length);ctx.lineTo(x+tx*length,y+ty*length);ctx.stroke();ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(x,y,3/cam.zoom,0,Math.PI*2);ctx.fill();ctx.restore();
}
function draftPath(shape,a,b,handle){
 ctx.beginPath();
 if(shape==='roundabout'){ctx.arc(a.x,a.y-a.z*28,distance(a,b),0,Math.PI*2);return}
 let points=(shape==='curve'||shape==='ramp')&&handle?projectCurve(draftPoints(shape,a,b,handle),a.z,b.z):[{x:a.x,y:a.y-a.z*28},{x:b.x,y:b.y-b.z*28}];
 ctx.moveTo(points[0].x,points[0].y);for(const p of points.slice(1))ctx.lineTo(p.x,p.y);
}
function parkingSnap(p){
 const threshold=8/cam.zoom,result={x:p.x,y:p.y,guides:[]};
 for(const axis of ['x','y']){let best=null;for(const corner of parkingCorners){let delta=Math.abs(p[axis]-corner[axis]);if(delta<threshold&&(!best||delta<best.delta))best={value:corner[axis],delta}}let grid=Math.round(p[axis]/50)*50,delta=Math.abs(p[axis]-grid);if(delta<threshold&&(!best||delta<best.delta))best={value:grid,delta};if(best){result[axis]=best.value;result.guides.push({axis,value:best.value})}}
 return result;
}
function parkingAnchor(poly,base,w=world){
 const candidates=[],clearanceWorld={...w,buildings:w.buildings.filter(b=>b.id!==base.id)};
 for(const road of w.roads){if((road.z??0)!==0||(road.zEnd??0)!==0)continue;for(let i=0;i<poly.length;i++){let a=poly[i],b=poly[(i+1)%poly.length];for(let k=1;k<16;k++){let t=k/16,p={x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t},q=projection(p,road.a,road.b),d=distance(p,q),edge=(14+8*totalLanes(road))/2;if(d<=edge+.5||d>=100)continue;let direction={x:(p.x-q.x)/d,y:(p.y-q.y)/d},inside={x:p.x+direction.x*4,y:p.y+direction.y*4};if(!pointInPolygon(inside,poly))continue;candidates.push({p,q,d,edge,direction,road})}}}
 candidates.sort((a,b)=>a.d-b.d);let best=null,bestScore=-Infinity;
 for(const candidate of candidates){const {p,q,d,edge,direction,road}=candidate,start={x:q.x+direction.x*edge,y:q.y+direction.y*edge},trial={...base,accessPoint:p,accessDirection:direction,accessRoadId:road.id,parkingDisconnected:false};
  if(!roadsideAllowed(trial,[road])||roadOverlapsBuildings(clearanceWorld,start,p,0,0,7))continue;
  let count=parkingSlots(trial).length,score=count-d*.04;if(count&&score>bestScore){best=candidate;bestScore=score}
 }
 return best;
}
function repairParkingEntrances(w){
 for(const b of w.buildings){if(b.type!=='parking'||!b.polygon)continue;const anchor=parkingAnchor(b.polygon,b,w);if(anchor){b.accessPoint=anchor.p;b.accessDirection=anchor.direction;b.accessRoadId=anchor.road.id;delete b.parkingDisconnected}else b.parkingDisconnected=true}
}
function finishParking(){
 let poly=parkingCorners.map(p=>({...p}));if(!validParkingPolygon(poly)){ $('#hint').textContent='Use at least three corners to make a non-crossing area large enough for parking.';return}
 let x=poly.reduce((s,p)=>s+p.x,0)/poly.length,y=poly.reduce((s,p)=>s+p.y,0)/poly.length,building={id:uuid(),type:'parking',x,y,polygon:poly};
 if(poly.some(p=>terrainAt(world,p)!=='land')||buildingOverlapsWorld(world,building)){$('#hint').textContent='Parking area must be on clear land, away from roads and buildings.';return}
 let anchor=parkingAnchor(poly,building);if(!anchor){$('#hint').textContent='No clear road entrance and parking aisle fit here. Try a wider area near a road.';return}
 building.accessPoint=anchor.p;building.accessDirection=anchor.direction;building.accessRoadId=anchor.road.id;
 parkingCorners=[];change(()=>world.buildings.push(building));$('#hint').textContent='Parking area created. Click corners to draw another.';
}
function drawParkingPreview(){
 if(tool!=='parking'||!parkingCorners.length)return;let target=mouse?parkingSnap(mouse):null;ctx.save();if(target?.guides.length){ctx.strokeStyle='#58a9a988';ctx.lineWidth=1/cam.zoom;ctx.setLineDash([6/cam.zoom,5/cam.zoom]);for(const guide of target.guides){ctx.beginPath();if(guide.axis==='x'){ctx.moveTo(guide.value,0);ctx.lineTo(guide.value,SIZE)}else{ctx.moveTo(0,guide.value);ctx.lineTo(SIZE,guide.value)}ctx.stroke()}}ctx.strokeStyle='#299b9c';ctx.fillStyle='#49b3a833';ctx.lineWidth=2/cam.zoom;ctx.setLineDash([5/cam.zoom,3/cam.zoom]);ctx.beginPath();ctx.moveTo(parkingCorners[0].x,parkingCorners[0].y);for(const p of parkingCorners.slice(1))ctx.lineTo(p.x,p.y);if(target)ctx.lineTo(target.x,target.y);if(parkingCorners.length>=3){ctx.closePath();ctx.fill()}ctx.stroke();ctx.setLineDash([]);for(const p of parkingCorners){ctx.beginPath();ctx.arc(p.x,p.y,4/cam.zoom,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.stroke()}ctx.restore();
}
function canPlaceBuilding(point){
 return point.x>=0&&point.y>=0&&point.x<=SIZE&&point.y<=SIZE&&terrainAt(world,point)==='land'&&!!roadsideAllowed(point,world.roads)&&!buildingOverlapsWorld(world,{type:tool,x:point.x,y:point.y});
}
function drawBuildingPreview(){
 if(!mouse||!['home','shop','industry'].includes(tool))return;
 const building={id:buildingPreviewId||(buildingPreviewId=uuid()),type:tool,x:mouse.x,y:mouse.y},valid=canPlaceBuilding(mouse),access=buildingAccess(building,world.roads),color=valid?(access?'#35a6a3':'#bd9343'):'#d85650';
 ctx.save();ctx.globalAlpha=.66;drawBuildingAccess(building,access);drawBuilding(building,access);ctx.restore();
 ctx.save();ctx.translate(building.x,building.y);ctx.rotate(access?.angle||0);const w=tool==='home'?30:tool==='shop'?42:tool==='parking'?68:48,h=tool==='home'?17:tool==='shop'?27:tool==='parking'?40:29;ctx.strokeStyle=color;ctx.fillStyle=valid?'#ffffff22':'#e9555540';ctx.lineWidth=2/cam.zoom;ctx.setLineDash([5/cam.zoom,3/cam.zoom]);ctx.fillRect(-w/2,-h/2,w,h);ctx.strokeRect(-w/2,-h/2,w,h);ctx.restore();
 const message=!valid?'Cannot build here':access?'Connected to road':'No road nearby',x=mouse.x+16/cam.zoom,y=mouse.y+23/cam.zoom;ctx.save();ctx.font=`${11/cam.zoom}px sans-serif`;let width=ctx.measureText(message).width+14/cam.zoom;ctx.fillStyle='#fffffff2';ctx.fillRect(x,y-13/cam.zoom,width,19/cam.zoom);ctx.strokeStyle=color;ctx.lineWidth=1/cam.zoom;ctx.strokeRect(x,y-13/cam.zoom,width,19/cam.zoom);ctx.fillStyle='#344743';ctx.fillText(message,x+7/cam.zoom,y);ctx.restore();
}
function drawBuildingAccess(building,access){
 if(building.type==='home'&&world.buildings.some(b=>b.id===building.id)){let spot=sim.graph.homeParking?.get(building.id);if(spot?.kind==='side'){let dx=spot.x-spot.start.x,dy=spot.y-spot.start.y,len=Math.hypot(dx,dy)||1;dx/=len;dy/=len;let end={x:spot.x+dx*6,y:spot.y+dy*6};line(spot.start,end,12,'#969b94');line(spot.start,{x:end.x-dx*1.5,y:end.y-dy*1.5},9,'#fff');ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(spot.start.x,spot.start.y,5.5,0,Math.PI*2);ctx.fill()}}
 if(!access||access.drivewayLength<1)return;
 const {start,end,dx,dy,drivewayLength}=access,scale=building.scale||1,variant=buildingVariant(building.id);
 const width=building.type==='home'?12:11;
 line(start,end,width+3,'#969b94');line(start,end,width,'#fff');
 // Open the outer road border at the mouth of the driveway.
 ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(start.x-dx*1.2,start.y-dy*1.2,(width+2)/2,0,Math.PI*2);ctx.fill();
 if(building.type==='shop'&&drivewayLength>15*scale){
  ctx.save();ctx.translate(end.x,end.y);ctx.rotate(Math.atan2(dy,dx));ctx.scale(scale,scale);
  const depth=13,half=18,mouth=6;
  ctx.fillStyle='#fff';ctx.fillRect(-depth,-half,depth+1,half*2);
  ctx.strokeStyle='#969b94';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(-depth,-mouth);ctx.lineTo(-depth,-half);ctx.lineTo(1,-half);ctx.lineTo(1,half);ctx.lineTo(-depth,half);ctx.lineTo(-depth,mouth);ctx.stroke();
  ctx.lineWidth=.8;for(const y of [-12,-7,7,12]){ctx.beginPath();ctx.moveTo(-depth+1,y);ctx.lineTo(-3,y);ctx.stroke()}
  for(const [i,y] of [-14.5,-9.5,9.5,14.5].entries())if((variant>>(i*2)&3)===0){ctx.fillStyle=['#5494ce','#e3a849','#80b773','#bd77b7'][i];ctx.fillRect(-9.5,y-1.4,5.5,2.8)}
  ctx.restore();
 }
}
function drawParkingSpaces(building,access){ctx.save();ctx.strokeStyle='#a5b0aa';ctx.lineWidth=.8;ctx.lineCap='round';for(const slot of parkingSlots(building,access?.angle||0)){ctx.save();ctx.translate(slot.x,slot.y);ctx.rotate(slot.angle);let l=slot.length/2,w=slot.width/2;ctx.beginPath();ctx.moveTo(-l,-w);ctx.lineTo(l,-w);ctx.moveTo(-l,w);ctx.lineTo(l,w);let outer=slot.side===-1?-l:l;ctx.moveTo(outer,-w);ctx.lineTo(outer,w);ctx.stroke();ctx.restore()}ctx.restore()}
function drawBuilding(building,access){
 if(building.type==='parking'&&building.polygon){ctx.save();ctx.fillStyle='#f8f9f6';ctx.strokeStyle='#9ba8a0';ctx.lineJoin='round';ctx.lineWidth=1.7;ctx.beginPath();ctx.moveTo(building.polygon[0].x,building.polygon[0].y);for(const p of building.polygon.slice(1))ctx.lineTo(p.x,p.y);ctx.closePath();ctx.fill();ctx.stroke();drawParkingSpaces(building,access);ctx.restore();return}
 ctx.save();ctx.translate(building.x,building.y);ctx.rotate(access?.angle||0);ctx.scale(building.scale||1,building.scale||1);
 if(building.type==='parking'){ctx.fillStyle='#fff';ctx.strokeStyle='#969b94';ctx.lineWidth=1.5;ctx.beginPath();ctx.roundRect(-34,-20,68,40,3);ctx.fill();ctx.stroke();ctx.restore();drawParkingSpaces(building,access);return}
 let home=building.type==='home',shop=building.type==='shop',w=home?30:shop?42:48,h=home?17:shop?27:29,side=access?.side||1;
 ctx.fillStyle='#5e625750';ctx.beginPath();ctx.moveTo(-w/2+5,-h/2+8);ctx.lineTo(w/2+6,-h/2+8);ctx.lineTo(w/2+12,h/2+12);ctx.lineTo(-w/2+10,h/2+12);ctx.closePath();ctx.fill();
 ctx.fillStyle=home?'#6ca845':shop?(building.variant==='purple'?'#633c9a':'#4d80c6'):'#696c6c';ctx.strokeStyle=home?'#629b40':shop?(building.variant==='purple'?'#4f2c7d':'#376cac'):'#646766';ctx.lineWidth=1.5;ctx.fillRect(-w/2,-h/2,w,h);ctx.strokeRect(-w/2,-h/2,w,h);
 if(home){
  ctx.fillStyle='#9bd371';ctx.fillRect(-w/2+1,-h/2+1,w-2,h*.48);ctx.strokeStyle='#7db957';ctx.beginPath();ctx.moveTo(-w/2+1,0);ctx.lineTo(w/2-1,0);ctx.stroke();
  ctx.fillStyle='#86c05e';ctx.beginPath();ctx.moveTo(-5,side*(h/2-3));ctx.lineTo(0,side*(h/2-8));ctx.lineTo(5,side*(h/2-3));ctx.lineTo(5,side*(h/2+4));ctx.lineTo(-5,side*(h/2+4));ctx.closePath();ctx.fill();ctx.stroke();
  ctx.beginPath();ctx.moveTo(0,side*(h/2-8));ctx.lineTo(0,side*(h/2+3));ctx.stroke();
 }else if(shop){
  ctx.fillStyle=building.variant==='purple'?'#916ac1':'#6ea6e6';ctx.fillRect(-w/2+2,-h/2+2,w-4,h-4);ctx.fillStyle=building.variant==='purple'?'#6e46a4':'#4b7dc1';ctx.fillRect(-7,-h/2-2,14,h+4);ctx.strokeRect(-7,-h/2-2,14,h+4);
 }else{
  ctx.fillStyle='#919493';ctx.fillRect(-w/2+2,-h/2+2,w-4,h-4);ctx.strokeStyle='#747878';for(let x=-w/2+7;x<w/2;x+=8){ctx.beginPath();ctx.moveTo(x,-h/2+2);ctx.lineTo(x,h/2-2);ctx.stroke()}
  ctx.fillStyle=buildingVariant(building.id)%3===0?'#5189c6':'#d68e52';ctx.fillRect(-w/2+4,-h/2+4,10,h-8);
 }
 ctx.restore();
}
function drawParkedVehicles(){
 for(const car of sim.parked){
  const access=buildingAccessCache.get(car.destinationId),building=access?.building;if(!access||!building)continue;
  const {leaving,progress:t}=parkingAnimation(car,sim.time);
  const slot=building.type==='parking'?parkingSlots(building,access.angle)[car.parkingSlot]:null;
  if(building.type==='parking'&&!slot)continue;
  const along=building.type==='shop'?(car.parkingSlot-1.5)*7:building.type==='industry'?(car.parkingSlot-1)*7:0;
  const homeSpot=building.type==='home'?sim.graph.homeParking?.get(building.id):null;
  const target=slot||homeSpot||{x:access.end.x-access.dx*6+Math.cos(access.angle)*along,y:access.end.y-access.dy*6+Math.sin(access.angle)*along};
  const roadAnchor=leaving?(departureLanePoint(sim.graph,car.nextTrip)||homeSpot?.start||access.start):(car.start||homeSpot?.start||access.start);
  let points=[roadAnchor,homeSpot?.start||access.start,target];
  if(building.polygon&&slot){let d=building.accessDirection||{x:building.x-access.end.x,y:building.y-access.end.y},length=Math.hypot(d.x,d.y)||1,dx=d.x/length,dy=d.y/length,at=(x,y)=>({x:access.end.x+dx*x-dy*y,y:access.end.y+dy*x+dx*y}),along=slot.aisleDistance??((slot.x-access.end.x)*dx+(slot.y-access.end.y)*dy),offset=slot.aisleOffset??0;points=[roadAnchor,access.start,access.end];if(offset)points.push(at(slot.branchDistance,0),at(slot.branchDistance,offset));points.push(at(along,offset),target)}
  let lengths=points.slice(1).map((p,i)=>distance(p,points[i])),remaining=t*lengths.reduce((a,b)=>a+b,0),i=0;while(i<lengths.length-1&&remaining>lengths[i])remaining-=lengths[i++];let from=points[i],to=points[i+1],part=lengths[i]?Math.min(1,remaining/lengths[i]):1,x=from.x+(to.x-from.x)*part,y=from.y+(to.y-from.y)*part;
  const direction=t>=.99?(building.type==='parking'?slot.angle:homeSpot?.angle??Math.atan2(access.dy,access.dx)):Math.atan2(to.y-from.y,to.x-from.x)+(leaving?Math.PI:0);
  ctx.save();ctx.translate(x,y);ctx.rotate(direction);ctx.fillStyle='#39474255';ctx.fillRect(-(car.length??10)/2+1,-2,car.length??10,6);ctx.fillStyle=car.color;ctx.fillRect(-(car.length??10)/2,-3,car.length??10,6);ctx.fillStyle='#263d49';ctx.fillRect(2,-2,1.5,4);ctx.restore();
 }
}
function render(){let r=view.getBoundingClientRect();ctx.clearRect(0,0,r.width,r.height);ctx.fillStyle='#b2cde9';ctx.fillRect(0,0,r.width,r.height);ctx.save();ctx.translate(r.width/2-cam.x*cam.zoom,r.height/2-cam.y*cam.zoom);ctx.scale(cam.zoom,cam.zoom);ctx.save();ctx.beginPath();ctx.rect(0,0,SIZE,SIZE);ctx.clip();if(cacheGraph!==sim.graph){ringCircles.clear();const original=ctx,viewCam=cam,viewRect=r;try{cam={x:SIZE/2,y:SIZE/2,zoom:1};r={width:SIZE,height:SIZE};ctx=prepareMapLayer(staticCanvas,r);ctx.fillStyle=world.ground||'#eeeee8';ctx.fillRect(0,0,SIZE,SIZE);for(const t of world.terrain){if(t.kind==='field'){ctx.fillStyle=t.color||'#d6df91';shape(t.points)}else if(t.kind==='water'){ctx.fillStyle='#f1e5b5';shape(t.points,1.12);ctx.fillStyle=t.color||'#a6c5ee';shape(t.points)}else if(t.kind==='forest'){ctx.fillStyle='#d7e8aa';shape(t.points,1.08);ctx.fillStyle='#b8d986';shape(t.points,.85);ctx.fillStyle='#aad481';shape(t.points,.64);}else{ctx.fillStyle='#62645d38';shape(t.points,1.1,{x:13,y:16});ctx.fillStyle='#777972';shape(t.points);ctx.save();shape(t.points,1,0,true);ctx.clip();ctx.fillStyle='#696b65';shape(t.points,.91,{x:30,y:37});ctx.fillStyle='#858780';shape(t.points,.76,{x:-27,y:-20});ctx.fillStyle='#73756e';shape(t.points,.58,{x:28,y:40});ctx.fillStyle='#92948d';shape(t.points,.37,{x:-36,y:-16});ctx.restore()}}const layers=[[],[],[]];for(const road of world.roads){let z0=road.z??0,z1=road.zEnd??z0,cuts=[0,1];if(z0!==z1)for(const z of [.5,1.5]){let t=(z-z0)/(z1-z0);if(t>0&&t<1)cuts.push(t)}cuts.sort((a,b)=>a-b);for(let i=1;i<cuts.length;i++){let a=cuts[i-1],b=cuts[i],point=t=>({x:road.a.x+(road.b.x-road.a.x)*t,y:road.a.y+(road.b.y-road.a.y)*t}),part={...road,a:point(a),b:point(b),z:z0+(z1-z0)*a,zEnd:z0+(z1-z0)*b,layerCutStart:a>0,layerCutEnd:b<1},layer=Math.min(2,Math.max(0,Math.round((part.z+part.zEnd)/2)));layers[layer].push(part)}}roadDepthIndex=buildRoadDepthIndex(layers);let overpasses=projectedOverpasses(layers);drawRoadGroups([...layers[1],...layers[2]],true);drawUnifiedRoads(layers[0],0,r);drawJunctionMarkings(0);drawProjectedOverpasses(layers[0],overpasses[0],0);let accesses=new Map();for(const b of world.buildings){let access=buildingAccess(b,world.roads);if(access){accesses.set(b.id,{...access,building:b});drawBuildingAccess(b,access)}}
for(const b of world.buildings)drawBuilding(b,accesses.get(b.id));for(const b of world.buildings)if(b.type==='parking'&&b.polygon){let access=accesses.get(b.id);if(access){drawBuildingAccess(b,access);ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(access.end.x,access.end.y,5.5,0,Math.PI*2);ctx.fill()}}for(let level=1;level<=2;level++){ctx=prepareMapLayer(upperCanvases[level-1],r);drawUnifiedRoads(layers[level],level,r);drawJunctionMarkings(level);drawProjectedOverpasses(layers[level],overpasses[level],level)}const boundaryLevels=drawNearbyRoadBoundaries(r);buildingAccessCache=accesses;activeRoadLayers=[true,layers[1].length>0||boundaryLevels.has(1),layers[2].length>0||boundaryLevels.has(2)];cacheGraph=sim.graph;snapshotMapLayers()}finally{ctx=original;cam=viewCam;r=viewRect}}visibleBounds={left:cam.x-r.width/(2*cam.zoom),right:cam.x+r.width/(2*cam.zoom),top:cam.y-r.height/(2*cam.zoom),bottom:cam.y+r.height/(2*cam.zoom)};vehiclePoses.clear();drawCachedLayer(0);drawTrafficSignals(0);drawPrioritySigns(0);drawVehicleRoute(0);if(overlay){const slow=new Map();for(const c of sim.cars)if(c.speed<5)slow.set(c.edge.road,(slow.get(c.edge.road)||0)+1);for(const rd of world.roads){let n=slow.get(rd.id)||0;if(n)line(zPoint(rd,rd.a,0),zPoint(rd,rd.b,1),7,n>3?'#e17161aa':'#e4bb59aa')}}drawCars(0);drawParkedVehicles();for(let level=1;level<=2;level++){drawCachedLayer(level);drawTrafficSignals(level);drawPrioritySigns(level);drawVehicleRoute(level);drawCars(level)}drawSelectedVehicle();drawWaitReason();drawLaneConnections();drawNodeHandles();drawBuildingPreview();drawParkingPreview();drawRoadsidePreview();if(tool==='road'&&shapeType==='ramp'){drawLaneSnap(start);drawLaneSnap(snapTarget)}if(laneEndTarget?.snapped){let hit=laneEndTarget,r=hit.road,t=projection(hit.point,r.a,r.b).t,p=zPoint(r,hit.point,t);ctx.save();ctx.strokeStyle='#168d97';ctx.fillStyle='#ffffff';ctx.lineWidth=2/cam.zoom;ctx.beginPath();ctx.arc(p.x,p.y,8/cam.zoom,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.font=`${11/cam.zoom}px sans-serif`;ctx.fillStyle='#176771';ctx.fillText('Junction',p.x+11/cam.zoom,p.y-9/cam.zoom);ctx.restore()}if(laneSelection){let hit=laneSelection.hit,r=hit.road,t=projection(hit.point,r.a,r.b).t,p=zPoint(r,hit.point,t);ctx.save();ctx.fillStyle='#ffffff';ctx.strokeStyle='#238b9b';ctx.lineWidth=2/cam.zoom;ctx.beginPath();ctx.arc(p.x,p.y,6/cam.zoom,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.restore()}if((tool==='lane-replace'||tool==='lane-add'||tool==='lane-remove')&&deleteHover){let section=deleteHover.section||selectedLaneSection(deleteHover.road,deleteHover.point),r=deleteHover.road,hitA=zPoint(r,r.a,0),hitB=zPoint(r,r.b,1),picked=pickLane(r,deleteHover.pickPoint||mouse,hitA,hitB,tool==='lane-add');for(const span of section){let a=zPoint(span.road,span.a,span.lo),b=zPoint(span.road,span.b,span.hi);if(tool==='lane-replace'){line(a,b,14+totalLanes(span.road)*8,'#5ab4c688');continue}if(!picked)continue;let sx=b.x-a.x,sy=b.y-a.y,sl=Math.hypot(sx,sy)||1,normal={x:-sy/sl,y:sx/sl},index=tool==='lane-add'?picked.count:picked.index,offset=laneCenter(r,picked.direction,index);line(offsetPoint(a,normal,offset),offsetPoint(b,normal,offset),8,tool==='lane-add'?'#5ab4c699':'#e35e5999')}}if(tool==='delete'&&deleteHover){for(const section of deleteRanges(deleteHover.road,deleteHover.point)){let r=section.road,a=zPoint(r,section.a,section.lo),b=zPoint(r,section.b,section.hi);line(a,b,14+totalLanes(r)*8,'#eb5a5299')}}if(tool==='select'&&selected?.type==='junction')for(const n of sim.graph.nodes.values())if(n.id===selected.id){ctx.fillStyle=world.junctions[n.id]?.mode==='signal'?'#e5ac55':'#7e9e82';ctx.beginPath();ctx.arc(n.x,n.y-n.z*28,4/cam.zoom,0,7);ctx.fill()}if(tool==='road'&&$('#guides').checked){ctx.save();ctx.strokeStyle='#6a9c9c30';ctx.lineWidth=.6/cam.zoom;ctx.setLineDash([2/cam.zoom,7/cam.zoom]);for(let n=0;n<=SIZE;n+=50){ctx.beginPath();ctx.moveTo(n,0);ctx.lineTo(n,SIZE);ctx.moveTo(0,n);ctx.lineTo(SIZE,n);ctx.stroke()}ctx.restore()}if(tool==='road'&&mouse&&snapTarget?.guides?.length){ctx.save();ctx.setLineDash([8,6]);ctx.strokeStyle='#58a9a9aa';ctx.lineWidth=1/cam.zoom;for(const g of snapTarget.guides){ctx.beginPath();if(g.axis==='x'){ctx.moveTo(g.v,0);ctx.lineTo(g.v,SIZE)}else if(g.axis==='y'){ctx.moveTo(0,g.v-(snapTarget.z||0)*28);ctx.lineTo(SIZE,g.v-(snapTarget.z||0)*28)}else{let y=g.y-(snapTarget.z||0)*28;ctx.moveTo(g.x-g.dx*SIZE,y-g.dy*SIZE);ctx.lineTo(g.x+g.dx*SIZE,y+g.dy*SIZE)}ctx.stroke()}let angle=snapTarget.guides.find(g=>g.axis==='angle');if(angle){let label=Math.round(((angle.angle*180/Math.PI)%360+360)%360)+'°',x=snapTarget.x+14/cam.zoom,y=snapTarget.y-(snapTarget.z||0)*28+12/cam.zoom;ctx.setLineDash([]);ctx.font=`${11/cam.zoom}px sans-serif`;let width=ctx.measureText(label).width+12/cam.zoom;ctx.fillStyle='#ffffffed';ctx.fillRect(x,y-13/cam.zoom,width,19/cam.zoom);ctx.strokeStyle='#a7bdb4';ctx.lineWidth=1/cam.zoom;ctx.strokeRect(x,y-13/cam.zoom,width,19/cam.zoom);ctx.fillStyle='#4b6760';ctx.fillText(label,x+6/cam.zoom,y)}ctx.restore()}if(tool==='road'&&mouse&&!start){let q=snapTarget||snap(mouse,null);ctx.strokeStyle=q.snapKind==='free'?'#d69753':'#36a4a3';ctx.lineWidth=2;ctx.beginPath();ctx.arc(q.x,q.y-(q.z||0)*28,10,0,7);ctx.stroke();ctx.beginPath();ctx.moveTo(q.x-15,q.y-(q.z||0)*28);ctx.lineTo(q.x+15,q.y-(q.z||0)*28);ctx.moveTo(q.x,q.y-(q.z||0)*28-15);ctx.lineTo(q.x,q.y-(q.z||0)*28+15);ctx.stroke()}if(start&&mouse){let end=snapTarget||snap(mouse,start.z),ok=!draftError(shapeType,start,end,control);let forward=shapeType==='ramp'?1:shapeType==='roundabout'?Math.max(1,+$('#lanesForward').value):+$('#lanesForward').value,backward=shapeType==='ramp'||shapeType==='roundabout'?0:+$('#lanesBackward').value;ctx.save();ctx.lineCap='round';ctx.lineJoin='round';ctx.lineWidth=14+8*(forward+backward);ctx.strokeStyle=ok?'#46b0c080':'#df716780';draftPath(shapeType,start,end,control);ctx.stroke();ctx.restore();ctx.setLineDash([7,6]);ctx.strokeStyle=ok?'#4aa5a4':'#d36f58';ctx.lineWidth=3;draftPath(shapeType,start,end,control);ctx.stroke();ctx.setLineDash([]);for(const q of [start,end]){ctx.strokeStyle=q.snapKind==='free'?'#d69753':'#36a4a3';ctx.lineWidth=2;ctx.beginPath();ctx.arc(q.x,q.y-(q.z||0)*28,10,0,7);ctx.stroke();ctx.beginPath();ctx.moveTo(q.x-15,q.y-(q.z||0)*28);ctx.lineTo(q.x+15,q.y-(q.z||0)*28);ctx.moveTo(q.x,q.y-(q.z||0)*28-15);ctx.lineTo(q.x,q.y-(q.z||0)*28+15);ctx.stroke()}}ctx.restore();ctx.restore();if(selected?.type==='vehicle'&&(render.lastVehicleUpdate??-1)<sim.time-.2){vehicleInspector($('#inspector'));render.lastVehicleUpdate=sim.time}let carCount=sim.cars.length,waiting=sim.cars.filter(c=>c.speed<3).length;if(render.lastCars!==carCount){$('#cars').innerHTML=`${carCount} <em>vehicles</em>`;render.lastCars=carCount}if(render.lastTrips!==sim.completed){$('#trips').textContent=sim.completed;render.lastTrips=sim.completed}if(render.lastWaiting!==waiting){$('#waiting').innerHTML=`${waiting} <em>vehicles</em>`;render.lastWaiting=waiting}}
function frame(t){
 requestAnimationFrame(frame);
 if(!last)last=t;let dt=Math.min(.1,(t-last)/1000);last=t;
 try{if(!paused){acc+=dt*speed;while(acc>=.05){sim.tick(.05);acc-=.05}}}catch(error){paused=true;acc=0;$('#pause').textContent='▶';$('#pause').setAttribute('aria-label','Resume simulation');$('#hint').textContent='Simulation paused after an error. You can resume it.';console.error('Simulation error',error)}
 try{render();frame.renderFailed=false}catch(error){ctx=canvas.getContext('2d');resize();cacheGraph=null;if(!frame.renderFailed){console.error('Map rendering failed',error);$('#hint').textContent='Map rendering failed; retrying.';frame.renderFailed=true}}
}window.addEventListener('resize',resize);resize();fit();title();save();setTool('select');requestAnimationFrame(frame);
