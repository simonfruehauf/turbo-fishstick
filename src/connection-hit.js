// Approximate distance to the same quadratic curve used by the lane editor.
export function connectionDistance(point, start, control, end) {
  let best = Infinity;
  let previous = start;
  for (let i = 1; i <= 24; i++) {
    const t = i / 24, u = 1 - t;
    const next = { x: u*u*start.x + 2*u*t*control.x + t*t*end.x, y: u*u*start.y + 2*u*t*control.y + t*t*end.y };
    const dx = next.x - previous.x, dy = next.y - previous.y;
    const length2 = dx*dx + dy*dy;
    const fraction = length2 ? Math.max(0, Math.min(1, ((point.x-previous.x)*dx + (point.y-previous.y)*dy)/length2)) : 0;
    best = Math.min(best, Math.hypot(point.x-previous.x-fraction*dx, point.y-previous.y-fraction*dy));
    previous = next;
  }
  return best;
}
