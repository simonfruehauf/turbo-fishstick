export const LANE_WIDTH=8;
export const laneCount=(road,direction)=>road[direction===0?'laneAtoB':'laneBtoA']??(road.oneway&&direction===1?0:road.lanes??1);
// Offsets are measured along the left normal of the stored a-to-b direction.
export const laneDivider=road=>(laneCount(road,1)-laneCount(road,0))*LANE_WIDTH/2;
export const laneBoundary=(road,direction,index)=>laneDivider(road)+(direction===0?1:-1)*index*LANE_WIDTH;
export const laneCenter=(road,direction,index)=>laneBoundary(road,direction,index+.5);
// In a vehicle's travel frame, the normal reverses for reverse-direction lanes.
export const travelLaneCenter=(road,direction,index)=>(direction===0?1:-1)*laneCenter(road,direction,index);
