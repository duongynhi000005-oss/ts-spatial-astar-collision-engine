/**
 * Exact 2D/3D Segment, Capsule & AABB Clearance and Collision Primitives.
 * Designed for PCB/CAD trace DRC, robotics obstacle avoidance, and game physics.
 */

import type { Box2D, Box3D } from "./spatial_index";

export interface Point2D {
  x: number;
  y: number;
}

export interface Point3D extends Point2D {
  z: number;
}

export interface Segment2D {
  start: Point2D;
  end: Point2D;
}

export interface Capsule2D extends Segment2D {
  radius: number;
}

function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}

export function pointToSegmentDistance2D(p: Point2D, seg: Segment2D): number {
  const dx = seg.end.x - seg.start.x;
  const dy = seg.end.y - seg.start.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq <= 1e-12) {
    return Math.hypot(p.x - seg.start.x, p.y - seg.start.y);
  }
  const t = clamp(
    ((p.x - seg.start.x) * dx + (p.y - seg.start.y) * dy) / lenSq,
    0,
    1,
  );
  const projX = seg.start.x + t * dx;
  const projY = seg.start.y + t * dy;
  return Math.hypot(p.x - projX, p.y - projY);
}

function cross2D(o: Point2D, a: Point2D, b: Point2D): number {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

function onSegment2D(p: Point2D, a: Point2D, b: Point2D): boolean {
  return (
    p.x >= Math.min(a.x, b.x) - 1e-9 &&
    p.x <= Math.max(a.x, b.x) + 1e-9 &&
    p.y >= Math.min(a.y, b.y) - 1e-9 &&
    p.y <= Math.max(a.y, b.y) + 1e-9
  );
}

export function segmentsIntersect2D(s1: Segment2D, s2: Segment2D): boolean {
  const d1 = cross2D(s2.start, s2.end, s1.start);
  const d2 = cross2D(s2.start, s2.end, s1.end);
  const d3 = cross2D(s1.start, s1.end, s2.start);
  const d4 = cross2D(s1.start, s1.end, s2.end);

  if (
    ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
    ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
  ) {
    return true;
  }

  if (Math.abs(d1) <= 1e-9 && onSegment2D(s1.start, s2.start, s2.end)) return true;
  if (Math.abs(d2) <= 1e-9 && onSegment2D(s1.end, s2.start, s2.end)) return true;
  if (Math.abs(d3) <= 1e-9 && onSegment2D(s2.start, s1.start, s1.end)) return true;
  if (Math.abs(d4) <= 1e-9 && onSegment2D(s2.end, s1.start, s1.end)) return true;

  return false;
}

export function segmentToSegmentDistance2D(s1: Segment2D, s2: Segment2D): number {
  if (segmentsIntersect2D(s1, s2)) return 0;
  return Math.min(
    pointToSegmentDistance2D(s1.start, s2),
    pointToSegmentDistance2D(s1.end, s2),
    pointToSegmentDistance2D(s2.start, s1),
    pointToSegmentDistance2D(s2.end, s1),
  );
}

export function capsuleToCapsuleClearance2D(c1: Capsule2D, c2: Capsule2D): number {
  return segmentToSegmentDistance2D(c1, c2) - (c1.radius + c2.radius);
}

export function segmentIntersectsBox2D(
  seg: Segment2D,
  box: Box2D,
  margin = 0,
): boolean {
  const b: Box2D = {
    minX: box.minX - margin,
    minY: box.minY - margin,
    maxX: box.maxX + margin,
    maxY: box.maxY + margin,
  };

  let t0 = 0;
  let t1 = 1;
  const dx = seg.end.x - seg.start.x;
  const dy = seg.end.y - seg.start.y;

  const p = [-dx, dx, -dy, dy];
  const q = [
    seg.start.x - b.minX,
    b.maxX - seg.start.x,
    seg.start.y - b.minY,
    b.maxY - seg.start.y,
  ];

  for (let i = 0; i < 4; i++) {
    if (Math.abs(p[i]) <= 1e-12) {
      if (q[i] < 0) return false;
    } else {
      const r = q[i] / p[i];
      if (p[i] < 0) {
        if (r > t1) return false;
        if (r > t0) t0 = r;
      } else {
        if (r < t0) return false;
        if (r < t1) t1 = r;
      }
    }
  }
  return t0 <= t1;
}

export function capsuleBounds3D(
  seg: Segment2D,
  layerZ: number,
  radius: number,
  layerThickness = 0.1,
): Box3D {
  return {
    minX: Math.min(seg.start.x, seg.end.x) - radius,
    minY: Math.min(seg.start.y, seg.end.y) - radius,
    maxX: Math.max(seg.start.x, seg.end.x) + radius,
    maxY: Math.max(seg.start.y, seg.end.y) + radius,
    minZ: layerZ - layerThickness * 0.5,
    maxZ: layerZ + layerThickness * 0.5,
  };
}
