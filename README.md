# ⚡ TypeScript 2D/3D Spatial Indexing, Capsule Collision & Multilayer A* Pathfinding Engine (`v1.0.0`)

A zero-dependency, deterministic TypeScript spatial geometry and pathfinding engine engineered for **browser-based CAD/EDA autorouters, 2D/3D robotics obstacle navigation, and real-time game/simulation engines**.

[![Ko-fi Shop ($5 Instant Download)](https://img.shields.io/badge/Ko--fi_Shop-Download_Full_Kit_($5)-FF5E5B?style=for-the-badge&logo=ko-fi&logoColor=white)](https://ko-fi.com/nhi741323)

---

## 📦 What's Included in the Kit

| File | Description |
| :--- | :--- |
| `spatial_index.ts` | **`SpatialHashGrid3D<T>`** — $O(1)$ 3D voxel/cell spatial hash index with configurable cell size, clearance-inflated AABB queries, and fast insertion/removal without full-tree rebalancing. |
| `collision.ts` | **Exact Geometric Primitives** — 2D/3D segment-to-segment distance, capsule-to-capsule DRC clearance (`capsuleToCapsuleClearance2D`), and Liang-Barsky parameterized segment-vs-AABB intersection (`segmentIntersectsBox2D`). |
| `multilayer_astar.ts` | **`MultilayerAStarRouter`** — Binary-heap accelerated 2D/3D A* pathfinder supporting planar Manhattan/Octilinear routing, bend penalties, automatic multi-layer Via transitions (`viaCost`), net-aware obstacle filtering (`netId`), and collinear waypoint simplification (`simplifyCollinearPath`). |
| `demo_interactive.html` | **Interactive SVG Visualizer** — Zero-build dark-mode HTML/SVG playground where you can toggle obstacles live and watch the router automatically drop vias to Layer 1 when Layer 0 is blocked. |
| `test_spatial_astar.test.ts` | **100% Passing Unit Test Suite** (`bun test`) verifying spatial hash accuracy, capsule DRC clearance, planar obstacle avoidance, and automatic multi-layer via bridging. |

---

## 🚀 Quickstart

```bash
bun test
```

### Example Usage

```typescript
import { MultilayerAStarRouter } from "./index";

const router = new MultilayerAStarRouter(4);

// Add a keepout wall on Layer 0
router.addObstacle({
  id: "keepout-1",
  bounds: { minX: 9, minY: -20, minZ: 0, maxX: 11, maxY: 20, maxZ: 0 },
});

const route = router.findPath(
  { x: 0, y: 0, layer: 0 },
  { x: 20, y: 0, layer: 0 },
  {
    gridStep: 2,
    bounds: { minX: -4, minY: -10, maxX: 24, maxY: 10 },
    layers: [0, 1],
    clearance: 0.5,
    viaCost: 6,
  },
);

console.log("Routed:", route.success, "Vias placed:", route.vias);
```
