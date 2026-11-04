import { defaultMovement, movementKey, nodeKey, projection, distance } from './model.js';

// When a new road cuts an existing edge, retain rules at its original junctions.
export function remapConnectionsAfterBuild(oldGraph,newGraph,junctions){
 for(const [nodeId,cfg] of Object.entries(junctions)){
  if(!Array.isArray(cfg.connectors))continue;
  const node=newGraph.nodes.get(nodeId);
  cfg.connectors=cfg.connectors.flatMap(key=>{
   const m=key.match(/^(.*)#(\d+)>(.*)#(\d+)$/);if(!m||!node)return [];
   const oldIn=oldGraph.edgeById.get(m[1]),oldOut=oldGraph.edgeById.get(m[3]);
   if(!oldIn||!oldOut)return [];
   const find=(edges,old)=>{
    const same=edges.find(e=>e.id===old.id)||edges.find(e=>e.road===old.road&&e.direction===old.direction);
    if(same)return same;
    // Bulldozing splits a road into new IDs. Match the surviving piece at
    // this junction to its old directed edge without borrowing another arm.
    const oldRoad=oldGraph.roadById.get(old.road),a=oldGraph.nodes.get(old.from),b=oldGraph.nodes.get(old.to);
    if(!oldRoad||!a||!b)return null;
    return edges.find(e=>{
     const road=newGraph.roadById.get(e.road),from=newGraph.nodes.get(e.from),to=newGraph.nodes.get(e.to);
     if(!road||!from||!to||e.direction!==old.direction||(road.groupId??road.id)!==(oldRoad.groupId??oldRoad.id))return false;
     const near=from.id===nodeId?from:to,far=from.id===nodeId?to:from;
     const t=Math.min(.5,distance(a,b)*.5/(e.length||1)),sample={x:near.x+(far.x-near.x)*t,y:near.y+(far.y-near.y)*t};
     return distance(sample,projection(sample,a,b))<.05;
    });
   };
   const incoming=find(node.incoming,oldIn),outgoing=find(node.out,oldOut),a=+m[2],b=+m[4];
   return incoming&&outgoing&&a<incoming.lanes&&b<outgoing.lanes?[movementKey(incoming,a,outgoing,b)]:[];
  });
 }
}

function defaultLinks(graph,node){
 const links=[];
 for(const incoming of node.incoming)for(const outgoing of node.out){
  if(outgoing.to===incoming.from)continue;
  for(let a=0;a<incoming.lanes;a++)for(let b=0;b<outgoing.lanes;b++)
   if(defaultMovement(graph,incoming,a,outgoing,b))links.push(movementKey(incoming,a,outgoing,b));
 }
 return links;
}

// The first click leaves the selected lane; the final click joins it.
export function connectRampToLanes(graph,junctions,rampGroup,start,end){
 for(const [anchor,leaving] of [[start,true],[end,false]]){
  if(!anchor?.laneAnchor)continue;
  const chosen=anchor.laneAnchor,node=graph.nodes.get(nodeKey(anchor,anchor.z??0));
  if(!node)return 'The selected lane is no longer connected.';
  const rampEdges=(leaving?node.out:node.incoming).filter(e=>{
   const road=graph.roadById.get(e.road);return (road?.groupId??road?.id)===rampGroup;
  });
  const candidates=(leaving?node.incoming:node.out).filter(e=>e.direction===chosen.direction);
  // At a lane-layout boundary the clicked piece may be on the other side of
  // the node. Use the adjacent piece of the same road when it has that lane.
  const main=candidates.find(e=>e.road===chosen.roadId&&chosen.lane<e.lanes) || candidates.find(e=>{const road=graph.roadById.get(e.road);return chosen.groupId && (road?.groupId??road?.id)===chosen.groupId && chosen.lane<e.lanes});
  if(!main||chosen.lane>=main.lanes||rampEdges.length!==1)
   return leaving?'Choose a lane that approaches the ramp.':'Choose a lane that continues away from the ramp.';
  const cfg=junctions[node.id]||{mode:'priority',phase:12};
  const links=new Set(Array.isArray(cfg.connectors)?cfg.connectors:defaultLinks(graph,node));
  const ramp=rampEdges[0];
  for(const incoming of node.incoming)for(const outgoing of node.out){
   if((leaving&&outgoing.id!==ramp.id)||(!leaving&&incoming.id!==ramp.id))continue;
   for(let a=0;a<incoming.lanes;a++)for(let b=0;b<outgoing.lanes;b++)links.delete(movementKey(incoming,a,outgoing,b));
  }
  links.add(leaving?movementKey(main,chosen.lane,ramp,0):movementKey(ramp,0,main,chosen.lane));
  junctions[node.id]={...cfg,connectors:[...links]};
 }
 return null;
}
