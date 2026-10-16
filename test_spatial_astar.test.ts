import { describe, expect, test } from "bun:test";
import {
  capsuleToCapsuleClearance2D,
  MultilayerAStarRouter,
  SpatialHashGrid3D,
  segmentIntersectsBox2D,
  segmentToSegmentDistance2D,
} from "./index";

describe("TS 2D/3D Spatial Index, Capsule Collision & Multilayer A* Engine", () => {
  test("1. SpatialHashGrid3D inserts, queries with margin, and removes items in O(1) cells", () => {
    const grid = new SpatialHashGrid3D<{ label: string }>(5);
    grid.insert(
      "obs-1",
      { minX: 0, minY: 0, minZ: 0, maxX: 4, maxY: 4, maxZ: 0 },
      { label: "Pad A" },
    );
    grid.insert(
      "obs-2",
      { minX: 20, minY: 20, minZ: 1, maxX: 25, maxY: 25, maxZ: 1 },
      { label: "Keepout B" },
    );

    const hitNearA = grid.query(
      { minX: 4.5, minY: 2, minZ: 0, maxX: 5, maxY: 3, maxZ: 0 },
      1.0,
    );
    expect(hitNearA.map((x) => x.id)).toEqual(["obs-1"]);

    expect(grid.remove("obs-1")).toBe(true);
    expect(grid.size()).toBe(1);
    const afterRemove = grid.query(
      { minX: 0, minY: 0, minZ: 0, maxX: 5, maxY: 5, maxZ: 0 },
      1.0,
    );
    expect(afterRemove.length).toBe(0);
  });

  test("2. Exact segment/capsule clearance and Liang-Barsky segment-box intersection", () => {
    const dCross = segmentToSegmentDistance2D(
      { start: { x: 0, y: 0 }, end: { x: 10, y: 10 } },
      { start: { x: 0, y: 10 }, end: { x: 10, y: 0 } },
    );
    expect(dCross).toBe(0);

    const clearance = capsuleToCapsuleClearance2D(
      { start: { x: 0, y: 0 }, end: { x: 10, y: 0 }, radius: 0.5 },
      { start: { x: 0, y: 3 }, end: { x: 10, y: 3 }, radius: 0.5 },
    );
    expect(clearance).toBeCloseTo(2.0, 6);

    expect(
      segmentIntersectsBox2D(
        { start: { x: -5, y: 5 }, end: { x: 15, y: 5 } },
        { minX: 2, minY: 2, maxX: 8, maxY: 8 },
        0.2,
      ),
    ).toBe(true);
  });

  test("3. MultilayerAStarRouter routes around planar obstacles and simplifies collinear points", () => {
    const router = new MultilayerAStarRouter(4);
    router.addObstacle({
      id: "wall-1",
      bounds: { minX: 8, minY: -4, minZ: 0, maxX: 12, maxY: 8, maxZ: 0 },
    });

    const res = router.findPath(
      { x: 0, y: 0, layer: 0 },
      { x: 20, y: 0, layer: 0 },
      {
        gridStep: 2,
        bounds: { minX: -10, minY: -10, maxX: 30, maxY: 20 },
        layers: [0],
        clearance: 0.5,
        viaCost: 10,
        allowDiagonal: false,
      },
    );

    expect(res.success).toBe(true);
    expect(res.vias.length).toBe(0);
    expect(res.path[0]).toEqual({ x: 0, y: 0, layer: 0 });
    expect(res.path[res.path.length - 1]).toEqual({ x: 20, y: 0, layer: 0 });
    for (const pt of res.path) {
      const insideObstacle = pt.x >= 8 && pt.x <= 12 && pt.y >= -4 && pt.y <= 8;
      expect(insideObstacle).toBe(false);
    }
  });

  test("4. MultilayerAStarRouter automatically drops a Via to Layer 1 when Layer 0 is completely walled off", () => {
    const router = new MultilayerAStarRouter(4);
    // Complete vertical wall on Layer 0 blocking x=10 across the entire board height
    router.addObstacle({
      id: "full-layer0-wall",
      bounds: { minX: 9, minY: -20, minZ: 0, maxX: 11, maxY: 20, maxZ: 0 },
    });

    const res = router.findPath(
      { x: 0, y: 0, layer: 0 },
      { x: 20, y: 0, layer: 0 },
      {
        gridStep: 2,
        bounds: { minX: -4, minY: -10, maxX: 24, maxY: 10 },
        layers: [0, 1],
        clearance: 0.5,
        viaCost: 6,
        allowDiagonal: false,
      },
    );

    expect(res.success).toBe(true);
    expect(res.vias.length).toBeGreaterThanOrEqual(2);
    expect(res.vias[0].fromLayer).toBe(0);
    expect(res.vias[0].toLayer).toBe(1);
    expect(res.path[res.path.length - 1]).toEqual({ x: 20, y: 0, layer: 0 });
  });
});
