#!/usr/bin/env python3
"""Apply test/coverage_waivers.txt to a verilator coverage.dat.

    cov_waive.py --build drone --rtl tt_um_hyphen133_drone_detection.sv \\
                 --waivers test/coverage_waivers.txt IN.dat OUT.dat

Reads every point in IN.dat, drops the ones a waiver for this build matches,
and writes the rest to OUT.dat for verilator_coverage to summarise. Prints one
row per waiver: how many uncovered and how many covered points it removed.

Exit 1 if a waiver removes no uncovered point. A waiver that hides nothing is
stale -- the code under it changed, or a test now reaches it -- and the proof
written beside it no longer describes the design. Delete the waiver rather
than carry it.
"""

import argparse
import os
import re
import sys

TYPES = ("line", "branch", "expr", "toggle")
FIELD_RE = re.compile("\x01(\\w+)\x02([^\x01]*)")


def parse_point(line):
    """One `C '...' count` record to a dict of its fields plus the count."""
    body, count = line[3:].rsplit("' ", 1)
    fields = dict(FIELD_RE.findall(body))
    return {**fields, "count": int(count)}


def load_waivers(path, build):
    waivers = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(" :: ", 3)]
            if len(parts) != 4:
                sys.exit(f"{path}:{n}: expected 'builds :: types :: regex :: reason'")
            builds, types, regex, reason = parts
            try:
                pattern = re.compile(regex)
            except re.error as e:
                sys.exit(f"{path}:{n}: bad regex {regex!r}: {e}")
            if builds != "all" and build not in builds.split(","):
                continue
            if types == "all":
                types = ",".join(TYPES)
            for t in types.split(","):
                if t not in TYPES:
                    sys.exit(f"{path}:{n}: unknown type {t!r}")
            waivers.append({"line": n, "types": types.split(","),
                            "re": pattern, "reason": reason,
                            "uncovered": 0, "covered": 0})
    return waivers


def match_key(point, src_lines):
    """The string a waiver regex sees: '<type> <point> @ <source line>'."""
    try:
        src = src_lines[int(point["l"]) - 1].strip()
    except (IndexError, KeyError, ValueError):
        src = ""
    return f"{point.get('t', '')} {point.get('o', '')} @ {src}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--build", required=True)
    ap.add_argument("--rtl", required=True, help="the DUT source the points refer to")
    ap.add_argument("--waivers", required=True)
    ap.add_argument("inp")
    ap.add_argument("out")
    args = ap.parse_args()

    with open(args.rtl) as f:
        src_lines = f.read().split("\n")
    rtl_name = os.path.basename(args.rtl)
    waivers = load_waivers(args.waivers, args.build)

    kept, header, dropped = [], [], 0
    with open(args.inp, errors="replace") as f:
        for line in f:
            if not line.startswith("C '"):
                header.append(line)
                continue
            p = parse_point(line)
            if p.get("f") != rtl_name:
                kept.append(line)
                continue
            key = match_key(p, src_lines)
            hit = next((w for w in waivers
                        if p.get("t") in w["types"] and w["re"].search(key)), None)
            if hit is None:
                kept.append(line)
                continue
            hit["uncovered" if p["count"] == 0 else "covered"] += 1
            dropped += 1

    with open(args.out, "w") as f:
        f.writelines(header)
        f.writelines(kept)

    print(f"waivers for the {args.build} build ({args.waivers}):")
    print(f"  {'uncov':>5} {'cov':>4}  reason")
    stale = []
    for w in waivers:
        print(f"  {w['uncovered']:>5} {w['covered']:>4}  {w['reason']}")
        if w["uncovered"] == 0:
            stale.append(w)
    print(f"  removed {dropped} of {dropped + len(kept)} points")
    if stale:
        print()
        print("STALE WAIVER: removes no uncovered point, so its proof no longer "
              "describes the design. Delete it:")
        for w in stale:
            print(f"  {args.waivers}:{w['line']}  {w['reason']}")
        sys.exit(1)


if __name__ == "__main__":
    main()
