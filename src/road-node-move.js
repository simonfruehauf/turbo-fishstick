import { buildGraph, distance, movementKey, nodeKey, projection, validateRoad, SIZE } from './model.js';

// Materialize the arms of a derived graph node, then move their common endpoint.
export function moveRoadNode(world, oldGraph, nodeId, target, makeId) {
 const node=oldGraph.nodes.get(nodeId);
 if(!node)return {error:'That road node no longer exists.'};
 if(target.x<0||target.x>SIZE||target.y<0||target.y>SIZE)return {error:'Keep the node inside the map.'};
 if([...oldGraph.nodes.values()].some(n=>n.id!==nodeId&&Math.abs(n.z-node.z)<.001&&distance(n,target)<.1))return {error:'Choose a position away from another node.'};
 if(world.ports.some(p=>Math.abs((p.z??0)-node.z)<.001&&distance(p,node)<.1))return {error:'Map boundary connections cannot be moved.'};
 const roads=[],origin=new Map(),changed=[];
 for(const r of world.roads){
  const za=r.z??0,zb=r.zEnd??za, a=distance(r.a,node)<.02&&Math.abs(za-node.z)<.001,b=distance(r.b,node)<.02&&Math.abs(zb-node.z)<.001;
  const q=projection(node,r.a,r.b),inside=!a&&!b&&za===zb&&Math.abs(za-node.z)<.001&&q.t>1e-6&&q.t<1-1e-6&&distance(node,q)<.02;
  let pieces;
  if(inside){const groupId=r.groupId??r.id;pieces=[{...r,groupId,b:{...target}},{...r,id:makeId(),groupId,a:{...target}}]}
  else if(a||b)pieces=[{...r,...(a?{a:{...target}}:{}),...(b?{b:{...target}}:{})}];
  else pieces=[r];
  for(const piece of pieces){roads.push(piece);origin.set(piece.id,r.id);if(inside||a||b)changed.push(piece)}
 }
 if(!changed.length)return {error:'This node has no movable road geometry.'};
 for(const r of changed){let err=validateRoad(world,r.a,r.b,r.z??0,r.zEnd??r.z??0,(14+8*((r.laneAtoB??r.lanes??1)+(r.laneBtoA??(r.oneway?0:r.lanes??1))))/2);if(err)return {error:err}}
 const proposed={...world,roads,junctions:structuredClone(world.junctions)},graph=buildGraph(proposed),newId=nodeKey(target,node.z);
 if(node.junction&&!graph.nodes.get(newId)?.junction)return {error:'Moving here would disconnect the intersection.'};
 const configs=proposed.junctions;proposed.junctions={};
 for(const [id,cfg] of Object.entries(configs)){
  const mappedId=id===nodeId?newId:id,oldNode=oldGraph.nodes.get(id),newNode=graph.nodes.get(mappedId);
  if(!oldNode||!newNode||!newNode.junction)return {error:'This move would lose intersection settings.'};
  if(proposed.junctions[mappedId])return {error:'This move would merge intersection settings.'};
  if(Array.isArray(cfg.connectors)){
   const mapEdge=(oldEdge,edges)=>{
    const oldOther=oldGraph.nodes.get(oldEdge.from===id?oldEdge.to:oldEdge.from);if(!oldOther)return null;
    const candidates=edges.filter(e=>origin.get(e.road)===oldEdge.road&&e.direction===oldEdge.direction);
    candidates.sort((a,b)=>{const other=e=>graph.nodes.get(e.from===mappedId?e.to:e.from);return distance(other(a),oldOther)-distance(other(b),oldOther)});
    return candidates[0];
   };
   const links=[];
   for(const key of cfg.connectors){const m=key.match(/^(.*)#(\d+)>(.*)#(\d+)$/),a=m&&oldGraph.edgeById.get(m[1]),b=m&&oldGraph.edgeById.get(m[3]);if(!a||!b)return {error:'Could not preserve an existing lane connection.'};const incoming=mapEdge(a,newNode.incoming),outgoing=mapEdge(b,newNode.out);if(!incoming||!outgoing||+m[2]>=incoming.lanes||+m[4]>=outgoing.lanes)return {error:'Could not preserve an existing lane connection.'};links.push(movementKey(incoming,+m[2],outgoing,+m[4]))}
   cfg.connectors=links;
  }
  proposed.junctions[mappedId]=cfg;
 }
 return {world:proposed,graph,nodeId:newId};
}

export function movableRoadNodes(world, graph){
 const endpoints=new Set();for(const r of world.roads){endpoints.add(nodeKey(r.a,r.z??0));endpoints.add(nodeKey(r.b,r.zEnd??r.z??0))}return [...graph.nodes.values()].filter(n=>n.junction||endpoints.has(n.id));
}

