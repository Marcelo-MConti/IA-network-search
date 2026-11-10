#!/usr/bin/env python3

"""
For each parameter set (n, k, p) and each search algorithm, generates the
deterministic small-world network once, runs search algorithm on 10 randomly
selected start/goal node pairs and measures how long the algorithm took to run.

Jobs = (parameter set x algorithm) are dispatched to a thread pool controlled
by -j / --jobs. Each job is a single swipl invocation that runs one algorithm
across all 10 pairs, amortising graph generation across the pairs.

Outputs one JSON file per parameter set in gen/.
"""

import argparse
import concurrent.futures
import datetime
import json
import random
import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
SEARCHS = BASE / "searchs"
HELPER = BASE / "scripts" / "benchmark.pl"
RESULTS = BASE / "gen"

PARAMS = [
    (2500, 7, 0.10),  # a. p = 10%
    (2500, 7, 0.05),  # b. p = 5%
    (2500, 7, 0.01),  # c. p = 1%
]

ALGOS = ["bfs", "dfs", "dijkstra", "astar", "best_first", "hill_climb"]
# ALGOS = ["hill_climb"]
N_PAIRS = 10

## Seeds tested, in (pair, graph) format:
##
##  (334871, 31)
##  (334872, 31)
##  (334873, 31)
##  (937325428328, 31)
##  (284529364, 31)
##  (7, 31)
##  (131071, 2147483647)
##  (437436970, 2147483647)
##  (12974362111, 2147483647)
##  (89162, 111)
##  (936381, 111)
##  
##
## Best seeds seen so far for Hill Climbing:
##
##  (7, 31)             found:  1/10,  1/10,  1/10
##  (89162, 111)        found:  0/10,  0/10,  1/10
##  (936381, 111)       found:  1/10,  0/10,  0/10
##  (334871, 31)        found:  1/10,  0/10,  0/10
##  (937325428328, 31)  found:  1/10,  0/10,  0/10


def gen_pairs(n: int, rng: random.Random):
    """10 distinct (start, goal) vertex pairs, start != goal."""
    pairs, seen = [], set()
    while len(pairs) < N_PAIRS:
        s, g = rng.randrange(n), rng.randrange(n)
        if s == g or (s, g) in seen:
            continue
        seen.add((s, g))
        pairs.append([s, g])
    return pairs


def pct(p: float) -> int:
    return int(round(p * 100))


def run_job(n, k, p, algo, pairs, graph_seed):
    """
    Run one algorithm across all pairs for one parameter set.
    Returns the parsed JSON object from the Prolog helper.
    """
    pairs_text = json.dumps(pairs)  # valid Prolog term: list of [int, int]
    cmd = ["swipl", "-q", "-f", str(HELPER), "--",
           str(graph_seed), str(n), str(k), repr(p), pairs_text, algo]
    proc = subprocess.run(cmd, cwd=str(SEARCHS), capture_output=True,
                          text=True, timeout=1800)
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise RuntimeError(
            f"swipl failed for n={n}, k={k}, p={p}, algo={algo}")
    return json.loads(proc.stdout)


def summarize(runs):
    found = [r for r in runs if r["found"]]
    times = [r["time_ms"] for r in runs]
    dists = [r["distance"] for r in found if r["distance"] is not None]
    return {
        "runs": len(runs),
        "found": len(found),
        "not_found": sum(1 for r in runs if not r["found"]),
        "mean_time_ms": sum(times) / len(times) if times else None,
        "min_time_ms": min(times) if times else None,
        "max_time_ms": max(times) if times else None,
        "mean_distance": sum(dists) / len(dists) if dists else None,
        "mean_hops": (sum(r["hops"] for r in found) / len(found)) if found else None,
    }


def main():
    ap = argparse.ArgumentParser(
        description="Benchmark graph search algorithms on small-world networks.")
    ap.add_argument("-j", "--jobs", type=int, required=True,
                    help="number of parallel swipl jobs (work queue size; no default)")
    ap.add_argument("-p", "--pair-seed", type=int, default=31,
                    help="seed used to generate start/goal node pairs")
    ap.add_argument("-s", "--graph-seed", type=int, default=31,
                    help="seed used to generate the graph")
    args = ap.parse_args()

    RESULTS.mkdir(parents=True, exist_ok=True)

    # Deterministic start/goal pairs per parameter set.
    param_pairs = {}
    for n, k, p in PARAMS:
        param_pairs[(n, k, p)] = gen_pairs(n, random.Random(args.pair_seed))

    # Build one job per (parameter set, algorithm): 3 * 6 = 18 jobs.
    jobs = []
    for n, k, p in PARAMS:
        pairs = param_pairs[(n, k, p)]
        for algo in ALGOS:
            jobs.append((n, k, p, algo, pairs, args.graph_seed))

    # Collect per-parameter-set results: {(n,k,p): {algo: data}}
    per_param = {(n, k, p): {"pairs": param_pairs[(n, k, p)],
                             "algorithms": {},
                             "graph": None,
                             "pair_count": None}
                 for (n, k, p) in PARAMS}

    def do_job(job):
        n, k, p, algo, pairs, graph_seed = job
        data = run_job(n, k, p, algo, pairs, graph_seed)
        return (n, k, p, algo, data)

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futures = [ex.submit(do_job, job) for job in jobs]
        for fut in concurrent.futures.as_completed(futures):
            n, k, p, algo, data = fut.result()
            bucket = per_param[(n, k, p)]
            if bucket["graph"] is None:
                bucket["graph"] = data["graph"]
                bucket["pair_count"] = data["pair_count"]
            bucket["algorithms"][algo] = data["algorithms"][algo]
            print(f"done n={n} k={k} p={p} algo={algo}", flush=True)

    # Write one JSON file per parameter set.
    for (n, k, p), bucket in per_param.items():
        out = {
            "graph": bucket["graph"],
            "pairs": bucket["pairs"],
            "algorithms": {},
        }
        for algo in ALGOS:
            algo_data = bucket["algorithms"][algo]
            algo_data["summary"] = summarize(algo_data["runs"])
            out["algorithms"][algo] = algo_data

        out_path = RESULTS / f"sw_n{n}_k{k}_p{pct(p)}.json"
        out_path.write_text(json.dumps(out, indent=2) + "\n")
        print(f"wrote {out_path}", flush=True)

    print("all done", flush=True)


if __name__ == "__main__":
    main()
