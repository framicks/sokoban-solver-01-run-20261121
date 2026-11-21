// Sokoban solver for make.fun's sokoban-solver-01-run-20261121.
//
// Strategy: the generator builds every level as a 1-wide hallway with one to
// four rooms off it, each room holding 3-4 boxes and as many goals, and a
// level's push-optimal count P* is the sum of its rooms' optima. So this
// solver decomposes a level into its rooms (articulation points of the floor
// graph: each room hangs off the hallway by a single door cell), solves each
// room on its own with a push-based A* (admissible min-assignment heuristic
// over BFS distances, static dead-square pruning, 2x2 frozen-block pruning,
// transposition table), and stitches the rooms together with plain walks
// through the hallway. Budgets are counted in expanded states, never in wall
// clock time, so two executions write byte-identical answers.
//
// Node 22 runs this file as-is (erasable TypeScript syntax only, no
// dependencies, no network, no clock reads).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Four moves: [dx, dy, letter]. Fixed order keeps every run deterministic.
const DIRS: [number, number, string][] = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
// Opposite direction index for DIRS entries above.
const OPP = [1, 0, 3, 2];

// Work budgets (expanded states). Rooms are small (a slot is at most 6x6 plus
// its door), so a generous per-room budget still fits the 60 s wall budget
// for a 5-instance set; the global cap bounds the whole run. A room gets a
// first attempt at ROOM_BUDGET and, when that fails, one retry at
// ROOM_BUDGET2, both inside TOTAL_BUDGET: at the measured ~29k expansions
// a second the whole run stays well inside the wall even on a slower CPU.
const ROOM_BUDGET = 60_000;
const ROOM_BUDGET2 = 300_000;
const TOTAL_BUDGET = 600_000;
let totalExpanded = 0;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall, per cell y*w+x
  goals: number[]; // sorted cell indices
  goalSet: Uint8Array;
  boxes: number[]; // sorted cell indices
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const goalSet = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch !== '#') wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') {
        goals.push(i);
        goalSet[i] = 1;
      }
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, goalSet, boxes, player };
}

// Breadth-first walk from `from` over floor cells, avoiding walls and the
// cells marked in `blocked`. Returns the path (list of cells after `from`,
// empty when from === to) or null when `to` is unreachable. Deterministic:
// neighbours are visited in DIRS order.
function bfsPath(
  w: number,
  wall: Uint8Array,
  blocked: Uint8Array,
  from: number,
  to: number,
): number[] | null {
  if (from === to) return [];
  const prev = new Int32Array(wall.length).fill(-1);
  const queue: number[] = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const [dx, dy] = DIRS[d]!;
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= hOf(wall, w)) continue;
      const n = ny * w + nx;
      if (wall[n] || blocked[n] || prev[n] !== -1) continue;
      prev[n] = c;
      if (n === to) {
        const path: number[] = [];
        for (let t = n; t !== from; t = prev[t]!) path.push(t);
        path.reverse();
        return path;
      }
      queue.push(n);
    }
  }
  return null;
}

function hOf(wall: Uint8Array, w: number): number {
  return wall.length / w;
}

// Reachable set from `from` over floor cells, avoiding walls and `blocked`
// (a Uint8Array indexed by cell). Returns a sorted array of cells.
function reachable(
  w: number,
  wall: Uint8Array,
  blocked: Uint8Array,
  from: number,
): number[] {
  const seen = new Uint8Array(wall.length);
  const out: number[] = [from];
  seen[from] = 1;
  const h = hOf(wall, w);
  for (let i = 0; i < out.length; i++) {
    const c = out[i]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const [dx, dy] = DIRS[d]!;
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!wall[n] && !blocked[n] && !seen[n]) {
        seen[n] = 1;
        out.push(n);
      }
    }
  }
  out.sort((a, b) => a - b);
  return out;
}

// All-pairs BFS distances over the given floor cells (a Uint8Array mask of
// the region). dist[a * N + b] is the shortest walk from cell a to cell b
// staying inside the region, or -1 when unreachable. N is the board size.
function allPairsDist(
  w: number,
  wall: Uint8Array,
  region: Uint8Array,
): Int32Array {
  const n = wall.length;
  const dist = new Int32Array(n * n).fill(-1);
  const h = hOf(wall, w);
  const queue: number[] = [];
  for (let s = 0; s < n; s++) {
    if (!region[s]) continue;
    dist[s * n + s] = 0;
    queue.length = 0;
    queue.push(s);
    for (let qi = 0; qi < queue.length; qi++) {
      const c = queue[qi]!;
      const cx = c % w;
      const cy = (c / w) | 0;
      const dc = dist[s * n + c]!;
      for (let d = 0; d < 4; d++) {
        const [dx, dy] = DIRS[d]!;
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nn = ny * w + nx;
        if (!region[nn] || dist[s * n + nn] !== -1) continue;
        dist[s * n + nn] = dc + 1;
        queue.push(nn);
      }
    }
  }
  return dist;
}

// Static dead squares for one region: a non-goal cell is dead when no push
// path leads from it to any goal. A push moves a box from c to a neighbour n
// only when the cell behind c (2c - n) is floor, so build that directed graph
// and mark every cell that cannot reach a goal in it. A box on a dead square
// can never reach a goal, whatever the other boxes do, so pruning it is
// sound. Returns a Uint8Array, 1 = dead.
function deadSquares(
  w: number,
  wall: Uint8Array,
  region: Uint8Array,
  goalSet: Uint8Array,
): Uint8Array {
  const n = wall.length;
  const h = hOf(wall, w);
  // Reverse edges: from a cell, which cells can push INTO it? A box at p
  // can be pushed to cur = p + d when the player can stand at p - d, so
  // from cur the predecessors are the cells p = cur - d whose p - d is
  // floor. A cell that cannot reach any goal in this graph can never hold
  // a box that reaches a goal.
  const canReach = new Uint8Array(n);
  const stack: number[] = [];
  for (let g = 0; g < n; g++) {
    if (region[g] && goalSet[g]) {
      canReach[g] = 1;
      stack.push(g);
    }
  }
  while (stack.length > 0) {
    const cur = stack.pop()!;
    const cx = cur % w;
    const cy = (cur / w) | 0;
    for (let d = 0; d < 4; d++) {
      const [dx, dy] = DIRS[d]!;
      // Predecessor p = cur - d: a box at p can be pushed to cur, with the
      // player standing at p - d = cur - 2d. Both must be floor.
      const px = cx - dx;
      const py = cy - dy;
      const qx = cx - 2 * dx;
      const qy = cy - 2 * dy;
      if (px < 0 || py < 0 || px >= w || py >= h) continue;
      if (qx < 0 || qy < 0 || qx >= w || qy >= h) continue;
      const p = py * w + px;
      const q = qy * w + qx;
      if (!region[p] || !region[q] || canReach[p]) continue;
      canReach[p] = 1;
      stack.push(p);
    }
  }
  const dead = new Uint8Array(n);
  for (let c = 0; c < n; c++) {
    if (region[c] && !goalSet[c] && !canReach[c]) dead[c] = 1;
  }
  return dead;
}

// Articulation points of the floor graph (iterative Tarjan), returned as a
// set of cell indices. A room hangs off the hallway by one door cell, and
// that door is an articulation point: removing it detaches the room.
function articulationPoints(w: number, wall: Uint8Array): Set<number> {
  const n = wall.length;
  const h = hOf(wall, w);
  const disc = new Int32Array(n).fill(-1);
  const low = new Int32Array(n);
  const isArt = new Uint8Array(n);
  let time = 0;
  const neighbours = (c: number): number[] => {
    const out: number[] = [];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const [dx, dy] = DIRS[d]!;
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const nn = ny * w + nx;
      if (!wall[nn]) out.push(nn);
    }
    return out;
  };
  // Iterative DFS with an explicit stack of frames.
  for (let start = 0; start < n; start++) {
    if (wall[start] || disc[start] !== -1) continue;
    // Frame: [cell, parent, next neighbour index, child count].
    const stack: [number, number, number, number][] = [[start, -1, 0, 0]];
    disc[start] = low[start] = time++;
    while (stack.length > 0) {
      const frame = stack[stack.length - 1]!;
      const [u, parent, ni, children] = frame;
      const nbs = neighbours(u);
      if (ni < nbs.length) {
        frame[2] = ni + 1;
        const v = nbs[ni]!;
        if (v === parent) continue;
        if (disc[v] !== -1) {
          if (disc[v] < low[u]!) low[u] = disc[v];
        } else {
          frame[3] = children + 1;
          disc[v] = low[v] = time++;
          stack.push([v, u, 0, 0]);
        }
      } else {
        stack.pop();
        if (parent !== -1) {
          if (low[u]! < low[parent]!) low[parent] = low[u]!;
          // u is an articulation point for its parent when the parent is
          // not the DFS root and low[u] >= disc[parent].
          if (parent !== start && low[u]! >= disc[parent]!) isArt[parent] = 1;
          // Root of this DFS tree: articulation when it has >1 child; the
          // child count is kept on the root's own frame, handled below.
        } else {
          if (children > 1) isArt[u] = 1;
        }
      }
    }
  }
  const out = new Set<number>();
  for (let c = 0; c < n; c++) if (isArt[c]) out.add(c);
  return out;
}

interface Room {
  cells: Uint8Array; // region mask: interior + door cell
  door: number;
  boxes: number[];
  goals: number[];
}

// Decompose a level into rooms. Primary method: the generator lays one
// 1-wide hallway across a single row, the player starts on it, and every
// room hangs off that row through one door cell in the neighbouring wall
// row. So the rooms are exactly the connected components of the floor
// outside the player's row. Falls back to the articulation-point method
// when that does not fit (an unexpected shape), and to null when neither
// fits (the caller then tries a whole-level search).
function decompose(lv: Level): Room[] | null {
  return decomposeByHallway(lv) ?? decomposeByArticulation(lv);
}

function decomposeByHallway(lv: Level): Room[] | null {
  const { w, wall, boxes, goals, player } = lv;
  const n = wall.length;
  const h = hOf(wall, w);
  const hallRow = (player / w) | 0;
  const inHallRow = (c: number): boolean => ((c / w) | 0) === hallRow;
  const boxSet = new Uint8Array(n);
  for (const b of boxes) boxSet[b] = 1;
  const seen = new Uint8Array(n);
  const rooms: Room[] = [];
  const covered = new Uint8Array(n);
  for (let s = 0; s < n; s++) {
    if (wall[s] || inHallRow(s) || seen[s]) continue;
    const comp: number[] = [s];
    seen[s] = 1;
    for (let i = 0; i < comp.length; i++) {
      const c = comp[i]!;
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const [dx, dy] = DIRS[d]!;
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nn = ny * w + nx;
        if (wall[nn] || inHallRow(nn) || seen[nn]) continue;
        seen[nn] = 1;
        comp.push(nn);
      }
    }
    const rBoxes = comp.filter((c) => boxSet[c]);
    const rGoals = comp.filter((c) => lv.goalSet[c]);
    if (rBoxes.length === 0 && rGoals.length === 0) continue; // stray pocket: ignore it
    if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) return null;
    // The door: the component's one cell touching a hallway-row floor cell.
    const doors = comp.filter((c) => {
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const [dx, dy] = DIRS[d]!;
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nn = ny * w + nx;
        if (inHallRow(nn) && !wall[nn]) return true;
      }
      return false;
    });
    if (doors.length !== 1) return null;
    const cells = new Uint8Array(n);
    for (const c of comp) {
      cells[c] = 1;
      covered[c] = 1;
    }
    rooms.push({ cells, door: doors[0]!, boxes: rBoxes, goals: rGoals });
  }
  if (rooms.length === 0) return null;
  for (const b of boxes) if (!covered[b]) return null;
  for (const g of goals) if (!covered[g]) return null;
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// Fallback decomposition by articulation points of the floor graph: each
// room hangs off the hallway by a single door cell, and that door is an
// articulation point. A box may sit on the door cell itself, so the door's
// own box/goal count toward the room. Candidates are taken smallest first
// (a hallway cut can detach two rooms at once as one oversized candidate)
// and a candidate is kept only when none of its cells is claimed yet.
function decomposeByArticulation(lv: Level): Room[] | null {
  const { w, wall, boxes, goals, player } = lv;
  const n = wall.length;
  const h = hOf(wall, w);
  const boxSet = new Uint8Array(n);
  for (const b of boxes) boxSet[b] = 1;
  const arts = articulationPoints(w, wall);
  interface Cand {
    cells: Uint8Array;
    door: number;
    size: number;
  }
  const cands: Cand[] = [];
  for (const v of arts) {
    const seen = new Uint8Array(n);
    seen[v] = 1;
    for (let s = 0; s < n; s++) {
      if (wall[s] || seen[s]) continue;
      const comp: number[] = [s];
      seen[s] = 1;
      let hasPlayer = s === player;
      let nb = boxSet[s] ? 1 : 0;
      let ng = lv.goalSet[s] ? 1 : 0;
      for (let i = 0; i < comp.length; i++) {
        const c = comp[i]!;
        const cx = c % w;
        const cy = (c / w) | 0;
        for (let d = 0; d < 4; d++) {
          const [dx, dy] = DIRS[d]!;
          const nx = cx + dx;
          const ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const nn = ny * w + nx;
          if (wall[nn] || seen[nn]) continue;
          seen[nn] = 1;
          comp.push(nn);
          if (nn === player) hasPlayer = true;
          if (boxSet[nn]) nb++;
          if (lv.goalSet[nn]) ng++;
        }
      }
      // Count the door cell's own box/goal too: a box may sit on the door.
      if (boxSet[v]) nb++;
      if (lv.goalSet[v]) ng++;
      if (!hasPlayer && nb > 0 && ng > 0 && nb === ng) {
        const cells = new Uint8Array(n);
        for (const c of comp) cells[c] = 1;
        cells[v] = 1;
        // A room's floor fits a 6x6 slot plus its door: at most 37 cells.
        if (comp.length + 1 <= 37) cands.push({ cells, door: v, size: comp.length + 1 });
      }
    }
  }
  cands.sort((a, b) => a.size - b.size);
  const rooms: Room[] = [];
  const covered = new Uint8Array(n);
  for (const c of cands) {
    let overlap = false;
    for (let i = 0; i < n; i++) {
      if (c.cells[i] && covered[i]) {
        overlap = true;
        break;
      }
    }
    if (overlap) continue;
    const rBoxes = boxes.filter((b) => c.cells[b]);
    const rGoals = goals.filter((g) => c.cells[g]);
    if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) continue;
    for (let i = 0; i < n; i++) if (c.cells[i]) covered[i] = 1;
    rooms.push({ cells: c.cells, door: c.door, boxes: rBoxes, goals: rGoals });
  }
  if (rooms.length === 0) return null;
  for (const b of boxes) if (!covered[b]) return null;
  for (const g of goals) if (!covered[g]) return null;
  for (const r of rooms) {
    for (let i = 0; i < n; i++) {
      if (r.cells[i] && i !== r.door && i === player) return null;
    }
  }
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// Permutations of [0..k-1], generated once per room for the assignment
// heuristic (k is 3 or 4, so 6 or 24 of them).
function permutations(k: number): number[][] {
  const out: number[][] = [];
  const cur: number[] = [];
  const used = new Uint8Array(k);
  const rec = (): void => {
    if (cur.length === k) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < k; i++) {
      if (used[i]) continue;
      used[i] = 1;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = 0;
    }
  };
  rec();
  return out;
}

interface RegionSolution {
  moves: string;
  pushes: number;
  expansions: number;
  endPlayer: number;
}

// Push-based A* over one region (a room, or a whole level as fallback).
// A state is the sorted box cells plus the player's square (the square the
// last push left the player on); the transposition key is those together,
// so two states with the same boxes and the same player square are one.
// Cost is pushes; the heuristic is the cheapest assignment of boxes to
// goals by BFS distance, which never overestimates: one push moves one box
// one square.
// `region` is where the player may walk (a room's cells plus the hallway
// square outside its door: the player may step out to push a box sitting on
// the door); `boxRegion` is where boxes may be pushed (the room's cells
// only: a box pushed into the hallway is lost). `door` is the cell the player
// must be able to reach when the boxes are all on goals (-1: no such
// requirement, for the whole-level fallback).
function solveRegion(
  lv: Level,
  region: Uint8Array,
  boxRegion: Uint8Array,
  boxes0: number[],
  goals: number[],
  startPlayer: number,
  door: number,
  budget: number,
): RegionSolution | null {
  const { w, wall, goalSet } = lv;
  const n = wall.length;
  const h = hOf(wall, w);
  const k = boxes0.length;
  const dist = allPairsDist(w, wall, region);
  const dead = deadSquares(w, wall, region, goalSet);
  const perms = permutations(k);
  const goalSorted = goals.slice().sort((a, b) => a - b);

  // Heuristic: cheapest box->goal assignment; Infinity when no assignment
  // has every box reaching a distinct goal.
  const heuristic = (bs: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      let ok = true;
      for (let i = 0; i < k; i++) {
        const d = dist[bs[i]! * n + goalSorted[p[i]!]]!;
        if (d < 0) {
          ok = false;
          break;
        }
        s += d;
      }
      if (ok && s < best) best = s;
    }
    return best;
  };

  // 2x2 frozen block: four cells all wall, outside the region, or box,
  // holding a box that is not on a goal. No box inside such a block can
  // ever move (every push needs an empty destination or a free square to
  // stand on, and all four are taken), so the state is dead. A block can
  // only newly appear around the cell a box just moved into.
  const frozenAt = (moved: number, blocked: Uint8Array): boolean => {
    const mx = moved % w;
    const my = (moved / w) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        const x0 = mx + ox;
        const y0 = my + oy;
        if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h) continue;
        let filled = 0;
        let bad = false;
        for (let dy = 0; dy < 2; dy++) {
          for (let dx = 0; dx < 2; dx++) {
            const c = (y0 + dy) * w + (x0 + dx);
            if (blocked[c]) {
              filled++;
              if (!goalSet[c]) bad = true;
            } else if (wall[c] || !region[c]) {
              filled++;
            }
          }
        }
        if (filled === 4 && bad) return true;
      }
    }
    return false;
  };

  const startBoxes = boxes0.slice().sort((a, b) => a - b);
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;

  // State storage, indexed by state id.
  const sp: number[] = [startPlayer];
  const sb: number[][] = [startBoxes];
  const sg: number[] = [0];
  const sf: number[] = [h0];
  const parent: number[] = [-1];
  const pFrom: number[] = [-1];
  const pDir: number[] = [-1];

  // Transposition table: numeric key -> state id with the smallest g seen.
  // Boxes are sorted cell indices (< 256: boards are at most 15x17) and the
  // player is the square the last push left it on. Two states with the same
  // boxes and the same stored player square are one state; states with the
  // same boxes but another stored player in the same reachable region are
  // kept apart (at most one per box pushed last), which costs a few extra
  // expansions but avoids a flood fill per successor just to canonicalise
  // the player.
  const keyOf = (pl: number, bs: number[]): number => {
    let key = pl;
    for (let i = 0; i < 4; i++) {
      key = key * 256 + (i < bs.length ? bs[i]! : 255);
    }
    return key;
  };
  const best = new Map<number, number>();
  best.set(keyOf(startPlayer, startBoxes), 0);

  // Binary heap of state ids, ordered by (f, -g, id): smallest f first, then
  // deepest, then earliest created. Deterministic throughout.
  const heap: number[] = [0];
  const less = (a: number, b: number): boolean =>
    sf[a]! < sf[b]! ||
    (sf[a]! === sf[b]! && (sg[a]! > sg[b]! || (sg[a]! === sg[b]! && a < b)));
  const heapPush = (id: number): void => {
    let i = heap.length;
    heap.push(id);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      const t = heap[i]!;
      heap[i] = heap[p]!;
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        const t = heap[i]!;
        heap[i] = heap[c]!;
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const blockedCur = new Uint8Array(n);
  const blockedNext = new Uint8Array(n);
  const fillBlocked = (arr: Uint8Array, bs: number[]): Uint8Array => {
    arr.fill(0);
    for (const b of bs) arr[b] = 1;
    return arr;
  };
  const reachMark = new Uint8Array(n);
  let expansions = 0;
  let goalId = -1;

  while (heap.length > 0) {
    if (expansions >= budget || totalExpanded >= TOTAL_BUDGET) return null;
    const s = heapPop();
    const bs = sb[s]!;
    // Stale heap entry (a better path to this key was found since).
    if (best.get(keyOf(sp[s]!, bs)) !== s) continue;
    // Reachable region from the stored player cell, boxes as obstacles;
    // computed once and used for the goal test and the pushes below.
    fillBlocked(blockedCur, bs);
    const regionCells = reachable(w, wall, blockedCur, sp[s]!);
    // Goal: every box on a goal, and the player can still reach the door.
    if (bs.every((b) => goalSet[b])) {
      if (door < 0 || regionCells.includes(door)) {
        goalId = s;
        break;
      }
      continue; // Boxes done but the player is shut in: never expand.
    }
    expansions++;
    totalExpanded++;
    reachMark.fill(0);
    for (const c of regionCells) reachMark[c] = 1;
    for (let bi = 0; bi < k; bi++) {
      const b = bs[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const [dx, dy] = DIRS[d]!;
        const nx = bx + dx;
        const ny = by + dy;
        const px = bx - dx;
        const py = by - dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (px < 0 || py < 0 || px >= w || py >= h) continue;
        const dest = ny * w + nx;
        const behind = py * w + px;
        if (!boxRegion[dest] || wall[dest] || blockedCur[dest]) continue;
        if (!reachMark[behind]) continue;
        if (dead[dest]) continue;
        const nb = bs.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        fillBlocked(blockedNext, nb);
        if (frozenAt(dest, blockedNext)) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        const g = sg[s]! + 1;
        // The successor's stored player square is the pushed box's old cell.
        const key = keyOf(b, nb);
        const seenId = best.get(key);
        if (seenId !== undefined && sg[seenId]! <= g) continue;
        const id = sp.length;
        sp.push(b);
        sb.push(nb);
        sg.push(g);
        sf.push(g + hh);
        parent.push(s);
        pFrom.push(b);
        pDir.push(d);
        best.set(key, id);
        heapPush(id);
      }
    }
  }
  if (goalId < 0) return null;

  // Reconstruct the push list, then walk it forward into moves.
  const pushes: [number, number][] = []; // [box cell before the push, dir]
  for (let t = goalId; t > 0; t = parent[t]!) pushes.push([pFrom[t]!, pDir[t]!]);
  pushes.reverse();
  let moves = '';
  let playerPos = startPlayer;
  let curBoxes = startBoxes.slice();
  const walkBlocked = new Uint8Array(n);
  for (const [from, d] of pushes) {
    const [dx, dy] = DIRS[d]!;
    const stand = from - (dy * w + dx);
    fillBlocked(walkBlocked, curBoxes);
    const path = bfsPath(w, wall, walkBlocked, playerPos, stand);
    if (path === null) return null; // Cannot happen: the search only used reachable stands.
    let prev = playerPos;
    for (const cell of path) {
      moves += stepLetter(prev, cell, w);
      prev = cell;
    }
    moves += DIRS[d]![2];
    const at = curBoxes.indexOf(from);
    curBoxes[at] = from + (dy * w + dx);
    curBoxes.sort((a, b) => a - b);
    playerPos = from;
  }
  return { moves, pushes: pushes.length, expansions, endPlayer: playerPos };
}

// The move letter for one step from cell a to adjacent cell b.
function stepLetter(a: number, b: number, w: number): string {
  const d = b - a;
  if (d === -w) return 'u';
  if (d === w) return 'd';
  if (d === -1) return 'l';
  return 'r';
}

// Solve one level: decompose into rooms and stitch, else a whole-level
// search as fallback. Returns the move string ('' when unsolved).
function solveLevel(lv: Level): string {
  const n = lv.wall.length;
  const rooms = decompose(lv);
  if (rooms !== null) {
    const solved = solveByRooms(lv, rooms);
    if (solved !== null) return solved;
  }
  // Fallback: one push-A* over the whole level, but only for small levels:
  // a whole-level search over 8+ boxes explodes, and a decomposed level
  // that failed per-room is not rescued by it.
  if (lv.boxes.length > 4) return '';
  const allFloor = new Uint8Array(n);
  for (let i = 0; i < n; i++) if (!lv.wall[i]) allFloor[i] = 1;
  const whole = solveRegion(
    lv,
    allFloor,
    allFloor,
    lv.boxes,
    lv.goals,
    lv.player,
    -1,
    Math.min(ROOM_BUDGET2, TOTAL_BUDGET - totalExpanded),
  );
  return whole === null ? '' : whole.moves;
}

// The hallway square just outside a room's door: the player stands here to
// enter the room (and to push a box that sits on the door itself). Falls
// back to the door when the room has no such neighbour.
function doorOutside(lv: Level, room: Room): number {
  const { w, wall } = lv;
  const dx0 = room.door % w;
  const dy0 = (room.door / w) | 0;
  for (let d = 0; d < 4; d++) {
    const [ddx, ddy] = DIRS[d]!;
    const nx = dx0 + ddx;
    const ny = dy0 + ddy;
    if (nx < 0 || ny < 0 || nx >= w || ny >= hOf(wall, w)) continue;
    const nb = ny * w + nx;
    if (!wall[nb] && !room.cells[nb]) return nb;
  }
  return room.door;
}

// Solve each room in turn and stitch the walks between them. Returns null
// when any room cannot be solved within budget.
function solveByRooms(lv: Level, rooms: Room[]): string | null {
  const { w, wall } = lv;
  const n = wall.length;
  // Room interiors (room cells minus doors): the player never walks through
  // another room's interior between rooms.
  const interior = new Uint8Array(n);
  for (const r of rooms) {
    for (let i = 0; i < n; i++) {
      if (r.cells[i] && i !== r.door) interior[i] = 1;
    }
  }
  // Current box cells, updated as rooms are solved (a solved room's boxes
  // end on its goals).
  let curBoxes = lv.boxes.slice();
  let moves = '';
  let playerPos = lv.player;
  for (const room of rooms) {
    // The player enters the room from the hallway square outside its door.
    const outside = doorOutside(lv, room);
    // Walk through the hallway to the square outside this room's door.
    const walkBlocked = new Uint8Array(n);
    for (const b of curBoxes) walkBlocked[b] = 1;
    const hallPath = bfsPathHall(w, wall, walkBlocked, interior, playerPos, outside);
    if (hallPath === null) return null;
    let prev = playerPos;
    for (const cell of hallPath) {
      moves += stepLetter(prev, cell, w);
      prev = cell;
    }
    playerPos = outside;
    // Solve the room from its door, the player starting outside it. The
    // player may also stand on the hallway square just outside the door
    // (needed to push a box that sits on the door itself); boxes stay
    // inside the room's cells.
    const region = room.cells.slice();
    region[outside] = 1;
    let sol = solveRegion(
      lv,
      region,
      room.cells,
      room.boxes,
      room.goals,
      outside,
      room.door,
      ROOM_BUDGET,
    );
    if (sol === null && totalExpanded < TOTAL_BUDGET) {
      // One retry with a larger budget: a hard room can need it, and the
      // global cap still bounds the whole run.
      sol = solveRegion(
        lv,
        region,
        room.cells,
        room.boxes,
        room.goals,
        outside,
        room.door,
        Math.min(ROOM_BUDGET2, TOTAL_BUDGET - totalExpanded),
      );
    }
    if (sol === null) return null;
    moves += sol.moves;
    // Walk back out to the door through the solved room.
    const finalBlocked = new Uint8Array(n);
    for (const g of room.goals) finalBlocked[g] = 1;
    const roomWall = new Uint8Array(n);
    for (let i = 0; i < n; i++) roomWall[i] = wall[i] || !room.cells[i] ? 1 : 0;
    const outPath = bfsPath(w, roomWall, finalBlocked, sol.endPlayer, room.door);
    if (outPath === null) return null;
    prev = sol.endPlayer;
    for (const cell of outPath) {
      moves += stepLetter(prev, cell, w);
      prev = cell;
    }
    playerPos = room.door;
    // This room's boxes are now on its goals.
    curBoxes = curBoxes.filter((b) => !room.boxes.includes(b));
    for (const g of room.goals) curBoxes.push(g);
    curBoxes.sort((a, b) => a - b);
  }
  return moves;
}

// BFS path that never steps into another room's interior: `interior` marks
// room-interior cells, but the start and the target door are always allowed.
function bfsPathHall(
  w: number,
  wall: Uint8Array,
  blocked: Uint8Array,
  interior: Uint8Array,
  from: number,
  to: number,
): number[] | null {
  if (from === to) return [];
  const n = wall.length;
  const h = hOf(wall, w);
  const prev = new Int32Array(n).fill(-1);
  const queue: number[] = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const [dx, dy] = DIRS[d]!;
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const nn = ny * w + nx;
      if (wall[nn] || blocked[nn] || prev[nn] !== -1) continue;
      if (interior[nn] && nn !== to) continue;
      prev[nn] = c;
      if (nn === to) {
        const path: number[] = [];
        for (let t = nn; t !== from; t = prev[t]!) path.push(t);
        path.reverse();
        return path;
      }
      queue.push(nn);
    }
  }
  return null;
}

for (const id of readdirSync(I).sort()) {
  totalExpanded = 0;
  const moves = solveLevel(parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8')));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
