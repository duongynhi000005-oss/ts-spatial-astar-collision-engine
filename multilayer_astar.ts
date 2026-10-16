/**
 * Multilayer Obstacle-Aware A* Pathfinder with Spatial Hash Grid Acceleration,
 * Via Transition Costing, and Collinear Waypoint Simplification.
 */

import { segmentIntersectsBox2D } from "./collision";
import { type Box3D, SpatialHashGrid3D } from "./spatial_index";

export interface GridNode3D {
  x: number;
  y: number;
  layer: number;
}

export interface Obstacle3D {
  id: string;
  bounds: Box3D;
  netId?: string;
}

export interface RouteOptions {
  gridStep: number;
  bounds: { minX: number; minY: number; maxX: number; maxY: number };
  layers: number[];
  clearance: number;
  viaCost: number;
  bendPenalty?: number;
  allowDiagonal?: boolean;
  netId?: string;
  maxIterations?: number;
}

export interface RouteResult {
  success: boolean;
  path: GridNode3D[];
  vias: { x: number; y: number; fromLayer: number; toLayer: number }[];
  totalCost: number;
  visitedNodes: number;
}

interface HeapNode {
  key: string;
  node: GridNode3D;
  g: number;
  f: number;
  dirX: number;
  dirY: number;
}

class MinBinaryHeap {
  private readonly data: HeapNode[] = [];

  get size(): number {
    return this.data.length;
  }

  push(item: HeapNode): void {
    this.data.push(item);
    this.bubbleUp(this.data.length - 1);
  }

  pop(): HeapNode | undefined {
    if (this.data.length === 0) return undefined;
    const top = this.data[0];
    const bottom = this.data.pop()!;
    if (this.data.length > 0) {
      this.data[0] = bottom;
      this.sinkDown(0);
    }
    return top;
  }

  private bubbleUp(idx: number): void {
    while (idx > 0) {
      const parent = (idx - 1) >> 1;
      if (this.data[idx].f >= this.data[parent].f) break;
      [this.data[idx], this.data[parent]] = [this.data[parent], this.data[idx]];
      idx = parent;
    }
  }

  private sinkDown(idx: number): void {
    const len = this.data.length;
    while (true) {
      const left = (idx << 1) + 1;
      const right = left + 1;
      let smallest = idx;
      if (left < len && this.data[left].f < this.data[smallest].f) {
        smallest = left;
      }
      if (right < len && this.data[right].f < this.data[smallest].f) {
        smallest = right;
      }
      if (smallest === idx) break;
      [this.data[idx], this.data[smallest]] = [this.data[smallest], this.data[idx]];
      idx = smallest;
    }
  }
}

export class MultilayerAStarRouter {
  private readonly spatialIndex: SpatialHashGrid3D<Obstacle3D>;

  constructor(cellSize = 10) {
    this.spatialIndex = new SpatialHashGrid3D<Obstacle3D>(cellSize);
  }

  addObstacle(obstacle: Obstacle3D): void {
    this.spatialIndex.insert(obstacle.id, obstacle.bounds, obstacle);
  }

  removeObstacle(id: string): boolean {
    return this.spatialIndex.remove(id);
  }

  private isSegmentBlocked(
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    layer: number,
    clearance: number,
    netId?: string,
  ): boolean {
    const queryBox: Box3D = {
      minX: Math.min(x1, x2),
      minY: Math.min(y1, y2),
      maxX: Math.max(x1, x2),
      maxY: Math.max(y1, y2),
      minZ: layer - 0.25,
      maxZ: layer + 0.25,
    };
    const candidates = this.spatialIndex.query(queryBox, clearance);
    for (const item of candidates) {
      if (netId && item.data.netId === netId) continue;
      if (
        segmentIntersectsBox2D(
          { start: { x: x1, y: y1 }, end: { x: x2, y: y2 } },
          item.bounds,
          clearance,
        )
      ) {
        return true;
      }
    }
    return false;
  }

  private isViaBlocked(
    x: number,
    y: number,
    fromLayer: number,
    toLayer: number,
    clearance: number,
    netId?: string,
  ): boolean {
    const minL = Math.min(fromLayer, toLayer);
    const maxL = Math.max(fromLayer, toLayer);
    const queryBox: Box3D = {
      minX: x,
      minY: y,
      maxX: x,
      maxY: y,
      minZ: minL - 0.25,
      maxZ: maxL + 0.25,
    };
    const candidates = this.spatialIndex.query(queryBox, clearance);
    return candidates.some((c) => !(netId && c.data.netId === netId));
  }

  findPath(start: GridNode3D, goal: GridNode3D, opts: RouteOptions): RouteResult {
    const step = opts.gridStep;
    const bendPenalty = opts.bendPenalty ?? 0.25 * step;
    const maxIterations = opts.maxIterations ?? 25000;
    const snap = (v: number) => Math.round(v / step) * step;

    const sNode: GridNode3D = {
      x: snap(start.x),
      y: snap(start.y),
      layer: start.layer,
    };
    const gNode: GridNode3D = {
      x: snap(goal.x),
      y: snap(goal.y),
      layer: goal.layer,
    };

    const nodeKey = (n: GridNode3D) =>
      `${Math.round(n.x / step)},${Math.round(n.y / step)},${n.layer}`;

    const heuristic = (n: GridNode3D) => {
      const dx = Math.abs(n.x - gNode.x);
      const dy = Math.abs(n.y - gNode.y);
      const dist2D = opts.allowDiagonal
        ? Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy)
        : dx + dy;
      const layerPenalty = n.layer === gNode.layer ? 0 : opts.viaCost;
      return dist2D + layerPenalty;
    };

    const open = new MinBinaryHeap();
    const gScore = new Map<string, number>();
    const cameFrom = new Map<string, string>();
    const nodeMap = new Map<string, GridNode3D>();

    const startKey = nodeKey(sNode);
    gScore.set(startKey, 0);
    nodeMap.set(startKey, sNode);
    open.push({
      key: startKey,
      node: sNode,
      g: 0,
      f: heuristic(sNode),
      dirX: 0,
      dirY: 0,
    });

    const goalKey = nodeKey(gNode);
    const dirs2D = opts.allowDiagonal
      ? [
          [1, 0],
          [-1, 0],
          [0, 1],
          [0, -1],
          [1, 1],
          [1, -1],
          [-1, 1],
          [-1, -1],
        ]
      : [
          [1, 0],
          [-1, 0],
          [0, 1],
          [0, -1],
        ];

    let visited = 0;

    while (open.size > 0 && visited < maxIterations) {
      const current = open.pop()!;
      visited++;

      if (current.key === goalKey) {
        const rawPath: GridNode3D[] = [];
        let curKey: string | undefined = goalKey;
        while (curKey) {
          rawPath.push(nodeMap.get(curKey)!);
          curKey = cameFrom.get(curKey);
        }
        rawPath.reverse();
        const simplified = simplifyCollinearPath(rawPath);
        const vias = extractVias(simplified);
        return {
          success: true,
          path: simplified,
          vias,
          totalCost: current.g,
          visitedNodes: visited,
        };
      }

      if (current.g > (gScore.get(current.key) ?? Number.POSITIVE_INFINITY)) {
        continue;
      }

      // 1. Planar neighbors on the same layer
      for (const [dx, dy] of dirs2D) {
        const nx = current.node.x + dx * step;
        const ny = current.node.y + dy * step;
        if (
          nx < opts.bounds.minX ||
          nx > opts.bounds.maxX ||
          ny < opts.bounds.minY ||
          ny > opts.bounds.maxY
        ) {
          continue;
        }
        if (
          this.isSegmentBlocked(
            current.node.x,
            current.node.y,
            nx,
            ny,
            current.node.layer,
            opts.clearance,
            opts.netId,
          )
        ) {
          continue;
        }

        const moveDist = dx !== 0 && dy !== 0 ? Math.SQRT2 * step : step;
        const turnExtra =
          (current.dirX !== 0 || current.dirY !== 0) &&
          (current.dirX !== dx || current.dirY !== dy)
            ? bendPenalty
            : 0;
        const tentativeG = current.g + moveDist + turnExtra;

        const nextNode: GridNode3D = {
          x: nx,
          y: ny,
          layer: current.node.layer,
        };
        const nKey = nodeKey(nextNode);
        if (tentativeG + 1e-9 < (gScore.get(nKey) ?? Number.POSITIVE_INFINITY)) {
          gScore.set(nKey, tentativeG);
          cameFrom.set(nKey, current.key);
          nodeMap.set(nKey, nextNode);
          open.push({
            key: nKey,
            node: nextNode,
            g: tentativeG,
            f: tentativeG + heuristic(nextNode),
            dirX: dx,
            dirY: dy,
          });
        }
      }

      // 2. Via transitions to other layers
      for (const targetLayer of opts.layers) {
        if (targetLayer === current.node.layer) continue;
        if (
          this.isViaBlocked(
            current.node.x,
            current.node.y,
            current.node.layer,
            targetLayer,
            opts.clearance,
            opts.netId,
          )
        ) {
          continue;
        }
        const tentativeG = current.g + opts.viaCost;
        const nextNode: GridNode3D = {
          x: current.node.x,
          y: current.node.y,
          layer: targetLayer,
        };
        const nKey = nodeKey(nextNode);
        if (tentativeG + 1e-9 < (gScore.get(nKey) ?? Number.POSITIVE_INFINITY)) {
          gScore.set(nKey, tentativeG);
          cameFrom.set(nKey, current.key);
          nodeMap.set(nKey, nextNode);
          open.push({
            key: nKey,
            node: nextNode,
            g: tentativeG,
            f: tentativeG + heuristic(nextNode),
            dirX: 0,
            dirY: 0,
          });
        }
      }
    }

    return {
      success: false,
      path: [],
      vias: [],
      totalCost: Number.POSITIVE_INFINITY,
      visitedNodes: visited,
    };
  }
}

export function simplifyCollinearPath(path: GridNode3D[]): GridNode3D[] {
  if (path.length <= 2) return [...path];
  const out: GridNode3D[] = [path[0]];
  for (let i = 1; i < path.length - 1; i++) {
    const prev = out[out.length - 1];
    const curr = path[i];
    const next = path[i + 1];
    if (prev.layer === curr.layer && curr.layer === next.layer) {
      const dx1 = curr.x - prev.x;
      const dy1 = curr.y - prev.y;
      const dx2 = next.x - curr.x;
      const dy2 = next.y - curr.y;
      if (Math.abs(dx1 * dy2 - dy1 * dx2) <= 1e-9 && dx1 * dx2 + dy1 * dy2 >= 0) {
        continue;
      }
    }
    out.push(curr);
  }
  out.push(path[path.length - 1]);
  return out;
}

export function extractVias(
  path: GridNode3D[],
): { x: number; y: number; fromLayer: number; toLayer: number }[] {
  const vias: { x: number; y: number; fromLayer: number; toLayer: number }[] = [];
  for (let i = 1; i < path.length; i++) {
    if (path[i].layer !== path[i - 1].layer) {
      vias.push({
        x: path[i].x,
        y: path[i].y,
        fromLayer: path[i - 1].layer,
        toLayer: path[i].layer,
      });
    }
  }
  return vias;
}
