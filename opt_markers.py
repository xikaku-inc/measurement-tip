"""Optimise the 5-marker layout of the printed measurement tip.

Frame: origin at the front face of the printed nose, +X toward the handle,
+Z out of the marker face (= print bed normal).

Two properties decide whether a marker set is usable, and they pull against
each other:

1. All 10 pairwise marker distances must be distinct. The smallest gap between
   two of them is the margin the tracker has for labelling markers.

2. Every 4-marker subset must be non-coplanar. A planar set has a mirror-flip
   pose ambiguity, so if the body is only non-planar thanks to one raised
   marker, losing that marker to occlusion doesn't degrade the pose - it makes
   it ambiguous, and the solver can jump to the mirrored solution. Measured as
   the smallest singular value of the centred subset (its out-of-plane extent
   in mm), which must stay far above the tracker's marker noise.

Requirement 2 is what forces all five posts to distinct heights. It also spreads
the out-of-plane information across all five markers rather than resting it on
one, which matters because tip error ~ orientation error x lever arm, and the
lever arm from the marker centroid to the point is >100 mm.

The per-marker height bounds encode where the structure can actually carry a
tall post: the nose is tapered and the arms are cantilevered, so those markers
stay low; the mid-shaft marker is on the stiffest section and carries the
tallest post.
"""
import itertools

import numpy as np
from scipy.optimize import differential_evolution

SV_FLOOR = 6.0     # mm, out-of-plane extent required of every 4-subset
MIN_DIST = 45.0    # mm, closest two markers
MIN_DZ = 4.0       # mm, minimum height difference between any two markers
POST_BASE = 20.0   # marker centre height minus exposed post length

#            x1        xc        arm_neg   arm_pos   x4         x5
BOUNDS = [(30, 50), (60, 90), (28, 58), (28, 58), (95, 125), (130, 145),
          #  z1        z2        z3        z4        z5
          (28, 40), (28, 44), (28, 44), (32, 58), (28, 46)]
NAMES = "x1 xc arm_neg arm_pos x4 x5 z1 z2 z3 z4 z5".split()


def markers(p):
    x1, xc, a2, a3, x4, x5 = p[:6]
    z1, z2, z3, z4, z5 = p[6:]
    return np.array([[x1, 0.0, z1], [xc, -a2, z2], [xc, a3, z3],
                     [x4, 0.0, z4], [x5, 0.0, z5]])


def out_of_plane(pts):
    """Smallest singular value of the centred cloud = its out-of-plane extent."""
    return np.linalg.svd(pts - pts.mean(0), compute_uv=False)[2]


def metrics(m):
    d = np.sort([np.linalg.norm(m[i] - m[j])
                 for i, j in itertools.combinations(range(5), 2)])
    return {
        "min_gap": np.diff(d).min(),
        "min_dist": d[0],
        "sv_all": out_of_plane(m),
        "sv_worst4": min(out_of_plane(m[list(s)])
                         for s in itertools.combinations(range(5), 4)),
    }


def cost(p):
    """Maximise the distance gap; constraints enter as steep penalties."""
    m = markers(p)
    k = metrics(m)
    pen = 20.0 * (max(0.0, SV_FLOOR - k["sv_worst4"])
                  + max(0.0, MIN_DIST - k["min_dist"])
                  + sum(max(0.0, MIN_DZ - abs(a - b))
                        for a, b in itertools.combinations(p[6:], 2)))
    return -k["min_gap"] + pen


def feasible(p):
    k = metrics(markers(p))
    return (k["sv_worst4"] >= SV_FLOOR and k["min_dist"] >= MIN_DIST
            and min(abs(a - b) for a, b in itertools.combinations(p[6:], 2)) >= MIN_DZ)


def solve(seeds=range(12)):
    best, best_k = None, -1.0
    for s in seeds:
        r = differential_evolution(cost, BOUNDS, seed=s, popsize=45, maxiter=1200,
                                   tol=1e-8, mutation=(0.4, 1.2), recombination=0.85,
                                   polish=True)
        p = np.round(r.x * 2) / 2                       # 0.5 mm grid
        if not feasible(p):
            p = r.x
            if not feasible(p):
                continue
        g = metrics(markers(p))["min_gap"]
        if g > best_k:
            best, best_k = p, g
    return best, best_k


def report(label, m):
    k = metrics(m)
    print(f"\n{label}")
    for i, r in enumerate(m, 1):
        print(f"  M{i}: [{r[0]:7.2f}, {r[1]:7.2f}, {r[2]:7.2f}]   post {r[2] - POST_BASE:5.1f} mm")
    d = sorted(np.linalg.norm(m[i] - m[j])
               for i, j in itertools.combinations(range(5), 2))
    print("  distances: " + "  ".join(f"{v:.1f}" for v in d))
    print(f"  min gap {k['min_gap']:5.2f} mm | closest pair {k['min_dist']:6.2f} mm")
    print(f"  out-of-plane: full set {k['sv_all']:5.2f} mm, worst 4-subset {k['sv_worst4']:5.2f} mm")


SHIPPED = np.array([[30.0, 0.0, 36.0], [73.5, -42.5, 40.0], [73.5, 57.0, 44.0],
                    [104.5, 0.0, 57.5], [144.0, 0.0, 29.0]])

if __name__ == "__main__":
    import sys

    report("v1, four markers at one height (rejected):",
           np.array([[30.0, 0, 30], [64.5, -47, 30], [64.5, 58, 30],
                     [110.5, 0, 63], [145.0, 0, 30]]))
    report("v2, all heights distinct (shipped, matches measurement_tip.py):", SHIPPED)
    if "--solve" in sys.argv:                    # ~10 min
        p, g = solve()
        print("\nparameters:")
        for n, v in zip(NAMES, p):
            print(f"  {n:8s} = {v:7.2f}")
        report("re-solved:", markers(p))
