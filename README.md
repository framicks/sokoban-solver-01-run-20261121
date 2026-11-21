# sokoban-solver-01-run-20261121

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/framicks/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/framicks/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

Push-optimal Sokoban solver for make.fun's ["Build the best Sokoban solver, 21 November 2026"](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty. One program answers all 5 instances of a set in a single run, in as few pushes as possible.

## How it works

Every level the generator draws is a 1-wide hallway with one to four rooms off it, each room holding 3–4 boxes and as many goals, and a level's push-optimal count P* is the sum of its rooms' optima. So the solver:

1. **Decomposes** the level into rooms — the connected components of the floor outside the hallway row (the player's row), each with its single door cell — with an articulation-point method as fallback.
2. **Solves each room on its own** with a push-based A*: a state is the sorted box cells plus the player's square, cost is pushes, and the heuristic is the cheapest assignment of boxes to goals by BFS distance (admissible: one push moves one box one square). Dead states are pruned with static dead squares (reverse push-reachability to a goal), 2×2 frozen blocks, and a transposition table. The player starts outside the door, on the hallway square, so a box sitting on the door itself can still be pushed in.
3. **Stitches** the rooms together with plain walks through the hallway.

Budgets are counted in expanded states, never in wall-clock time, so two executions write byte-identical answers: a room gets 60,000 expansions and one 300,000-expansion retry, inside a 600,000-expansion cap for the whole run.

## Results

- Practice set (`grade-local`, two executions, byte-identical): **score 1,000,000** — all 5 instances at exactly P* pushes (13, 16, 29, 47, 60), in ~0.6 s.
- House baseline plain-astar 1.0.0 on the same set: 200,000 (only i001 solved).
- Six further sets drawn from fresh random seeds with the kit's own generator: all 1,000,000 as well — the solver generalises, it does not memorise the practice set.

## Run it

```sh
node cli/makefun.mjs grade-local --kit . --bot . --driver process
```

`main.ts` is the whole entry: Node 22 runs it as-is (erasable TypeScript only, no dependencies, no network, no clock reads). It reads `/input/<id>/level.txt` and writes `/output/<id>/solution.txt` (LURD moves) for each instance.

## Files

- `main.ts` — the solver.
- `.makefun/owner.json` — the payout wallet.
- `LICENSE` — MIT.

---

<sub>Entered on [make.fun](https://make.fun/?ref=erdians&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
