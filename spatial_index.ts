/**
 * 2D & 3D Spatial Hash Grid and AABB Bounding-Box Index
 * Zero-dependency, deterministic O(1) cell lookup & fast AABB queries.
 */

export interface Box2D {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

export interface Box3D extends Box2D {
  minZ: number;
  maxZ: number;
}

export interface SpatialItem3D<T> {
  id: string;
  bounds: Box3D;
  data: T;
}

export function boxesOverlap2D(a: Box2D, b: Box2D, margin = 0): boolean {
  return (
    a.minX - margin <= b.maxX &&
    a.maxX + margin >= b.minX &&
    a.minY - margin <= b.maxY &&
    a.maxY + margin >= b.minY
  );
}

export function boxesOverlap3D(a: Box3D, b: Box3D, margin = 0): boolean {
  return (
    boxesOverlap2D(a, b, margin) &&
    a.minZ - margin <= b.maxZ &&
    a.maxZ + margin >= b.minZ
  );
}

export function expandBox3D(box: Box3D, margin: number): Box3D {
  return {
    minX: box.minX - margin,
    minY: box.minY - margin,
    minZ: box.minZ - margin,
    maxX: box.maxX + margin,
    maxY: box.maxY + margin,
    maxZ: box.maxZ + margin,
  };
}

export class SpatialHashGrid3D<T> {
  private readonly cellSize: number;
  private readonly cells = new Map<string, Set<string>>();
  private readonly items = new Map<string, SpatialItem3D<T>>();

  constructor(cellSize = 10) {
    if (cellSize <= 0 || !Number.isFinite(cellSize)) {
      throw new Error("cellSize must be a positive finite number");
    }
    this.cellSize = cellSize;
  }

  private cellCoord(v: number): number {
    return Math.floor(v / this.cellSize);
  }

  private key(cx: number, cy: number, cz: number): string {
    return `${cx},${cy},${cz}`;
  }

  insert(id: string, bounds: Box3D, data: T): void {
    if (this.items.has(id)) {
      this.remove(id);
    }
    const item: SpatialItem3D<T> = { id, bounds: { ...bounds }, data };
    this.items.set(id, item);

    const minCx = this.cellCoord(bounds.minX);
    const maxCx = this.cellCoord(bounds.maxX);
    const minCy = this.cellCoord(bounds.minY);
    const maxCy = this.cellCoord(bounds.maxY);
    const minCz = this.cellCoord(bounds.minZ);
    const maxCz = this.cellCoord(bounds.maxZ);

    for (let cx = minCx; cx <= maxCx; cx++) {
      for (let cy = minCy; cy <= maxCy; cy++) {
        for (let cz = minCz; cz <= maxCz; cz++) {
          const k = this.key(cx, cy, cz);
          let bucket = this.cells.get(k);
          if (!bucket) {
            bucket = new Set<string>();
            this.cells.set(k, bucket);
          }
          bucket.add(id);
        }
      }
    }
  }

  remove(id: string): boolean {
    const existing = this.items.get(id);
    if (!existing) return false;
    const { bounds } = existing;
    const minCx = this.cellCoord(bounds.minX);
    const maxCx = this.cellCoord(bounds.maxX);
    const minCy = this.cellCoord(bounds.minY);
    const maxCy = this.cellCoord(bounds.maxY);
    const minCz = this.cellCoord(bounds.minZ);
    const maxCz = this.cellCoord(bounds.maxZ);

    for (let cx = minCx; cx <= maxCx; cx++) {
      for (let cy = minCy; cy <= maxCy; cy++) {
        for (let cz = minCz; cz <= maxCz; cz++) {
          const k = this.key(cx, cy, cz);
          const bucket = this.cells.get(k);
          if (bucket) {
            bucket.delete(id);
            if (bucket.size === 0) {
              this.cells.delete(k);
            }
          }
        }
      }
    }
    this.items.delete(id);
    return true;
  }

  query(searchBounds: Box3D, margin = 0): SpatialItem3D<T>[] {
    const padded = expandBox3D(searchBounds, margin);
    const minCx = this.cellCoord(padded.minX);
    const maxCx = this.cellCoord(padded.maxX);
    const minCy = this.cellCoord(padded.minY);
    const maxCy = this.cellCoord(padded.maxY);
    const minCz = this.cellCoord(padded.minZ);
    const maxCz = this.cellCoord(padded.maxZ);

    const seen = new Set<string>();
    const results: SpatialItem3D<T>[] = [];

    for (let cx = minCx; cx <= maxCx; cx++) {
      for (let cy = minCy; cy <= maxCy; cy++) {
        for (let cz = minCz; cz <= maxCz; cz++) {
          const bucket = this.cells.get(this.key(cx, cy, cz));
          if (!bucket) continue;
          for (const id of bucket) {
            if (seen.has(id)) continue;
            seen.add(id);
            const item = this.items.get(id);
            if (item && boxesOverlap3D(item.bounds, searchBounds, margin)) {
              results.push(item);
            }
          }
        }
      }
    }
    return results;
  }

  size(): number {
    return this.items.size;
  }

  clear(): void {
    this.cells.clear();
    this.items.clear();
  }
}
