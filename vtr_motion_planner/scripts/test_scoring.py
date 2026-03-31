#!/usr/bin/env python
# test_scoring.py
# ---------------------------------------------------------------------------
# Offline unit-test (no ROS required) that validates the trajectory scoring
# logic and occupancy grid math using pure Python.
#
# Run directly:  python scripts/test_scoring.py
# ---------------------------------------------------------------------------

import math
import sys

# ============================================================
# Minimal Python re-implementation of the scoring function
# (mirrors trajectory_scorer.cpp for cross-validation)
# ============================================================

def precompute_arcs(K=21, omega_min=-0.6, omega_max=0.6,
                    v=0.3, T=1.0, dt=0.1):
    arcs = []
    for k in range(K):
        omega = omega_min + k / (K - 1) * (omega_max - omega_min) if K > 1 else 0.0
        pts = []
        x, y, theta = 0.0, 0.0, 0.0
        steps = int(round(T / dt))
        for _ in range(steps):
            x     += v * math.cos(theta) * dt
            y     += v * math.sin(theta) * dt
            theta += omega * dt
            pts.append((x, y))
        arcs.append({'omega': omega, 'pts': pts})
    return arcs


def angle_degrees(endpoint, goal):
    ax, ay = endpoint
    bx, by = goal
    dot    = ax * bx + ay * by
    mag_a  = math.sqrt(ax*ax + ay*ay)
    mag_b  = math.sqrt(bx*bx + by*by)
    if mag_a < 1e-9 or mag_b < 1e-9:
        return 0.0
    cos_t  = max(-1.0, min(1.0, dot / (mag_a * mag_b)))
    return math.degrees(math.acos(cos_t))


def score_arcs(arcs, goals, goals_to_use=3, obstacles=None):
    """
    Returns list of (score, feasible, omega) for each arc.
    obstacles: list of (ox, oy, radius) circles in robot frame.
    """
    M = min(goals_to_use, len(goals))
    results = []
    for arc in arcs:
        # Feasibility
        feasible = True
        if obstacles:
            for (x, y) in arc['pts']:
                for (ox, oy, r) in obstacles:
                    if math.sqrt((x-ox)**2 + (y-oy)**2) < r:
                        feasible = False
                        break
                if not feasible:
                    break

        if not feasible:
            results.append((0.0, False, arc['omega']))
            continue

        if M == 0:
            results.append((0.0, True, arc['omega']))
            continue

        endpoint = arc['pts'][-1]
        total = 0.0
        for m in range(M):
            theta = angle_degrees(endpoint, goals[m])
            term  = 1.0 - math.sqrt(0.005 * theta)
            total += max(0.0, term)
        results.append((total / goals_to_use, True, arc['omega']))
    return results


# ============================================================
# Tests
# ============================================================

def test_straight_goals():
    """Straight goals ahead: arc with omega≈0 should score highest."""
    arcs   = precompute_arcs()
    goals  = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    result = score_arcs(arcs, goals)
    best_i = max(range(len(result)), key=lambda i: result[i][0] if result[i][1] else -1)
    best_omega = result[best_i][2]
    assert abs(best_omega) < 0.1, \
        f"Expected omega≈0 for straight goals, got {best_omega:.3f}"
    print(f"  PASS: straight goals → omega={best_omega:.4f} rad/s (expected ≈0)")


def test_left_turn():
    """Goals to the left: best arc should have positive omega."""
    arcs   = precompute_arcs()
    goals  = [(0.5, 1.0), (0.5, 2.0), (0.5, 3.0)]
    result = score_arcs(arcs, goals)
    best_i = max(range(len(result)), key=lambda i: result[i][0] if result[i][1] else -1)
    best_omega = result[best_i][2]
    assert best_omega > 0.0, \
        f"Expected positive omega for left-side goals, got {best_omega:.3f}"
    print(f"  PASS: left-turn goals → omega={best_omega:.4f} rad/s (expected >0)")


def test_right_turn():
    """Goals to the right: best arc should have negative omega."""
    arcs   = precompute_arcs()
    goals  = [(0.5, -1.0), (0.5, -2.0), (0.5, -3.0)]
    result = score_arcs(arcs, goals)
    best_i = max(range(len(result)), key=lambda i: result[i][0] if result[i][1] else -1)
    best_omega = result[best_i][2]
    assert best_omega < 0.0, \
        f"Expected negative omega for right-side goals, got {best_omega:.3f}"
    print(f"  PASS: right-turn goals → omega={best_omega:.4f} rad/s (expected <0)")


def test_obstacle_blocks_straight():
    """Obstacle on the straight-ahead path should eliminate the straight arc
    while leaving strongly-turning arcs free."""
    arcs  = precompute_arcs()
    goals = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]

    # Place obstacle at arc endpoint of the straight trajectory (x≈0.30, y=0).
    # Use a small radius so only arcs very close to straight are blocked.
    straight_i  = len(arcs) // 2
    ex, ey      = arcs[straight_i]['pts'][-1]   # (0.30, 0.0)
    obstacles   = [(ex, ey, 0.05)]              # 5 cm radius – hits straight arc
    result      = score_arcs(arcs, goals, obstacles=obstacles)

    # Find the arc with omega closest to 0
    straight_i = min(range(len(arcs)),
                     key=lambda i: abs(arcs[i]['omega']))
    assert not result[straight_i][1], \
        "Expected straight arc to be infeasible due to obstacle"
    print(f"  PASS: obstacle ahead → straight arc (ω={arcs[straight_i]['omega']:.3f}) blocked")

    # Best remaining arc should be non-zero
    best_i = max(range(len(result)), key=lambda i: result[i][0] if result[i][1] else -1)
    assert result[best_i][1], "Expected at least one feasible arc to remain"
    print(f"  PASS: best feasible arc has ω={result[best_i][2]:.4f} rad/s")


def test_all_blocked():
    """All arcs blocked → no feasible trajectory."""
    arcs      = precompute_arcs()
    goals     = [(1.0, 0.0)]
    # Giant obstacle covering everything within 1 m
    obstacles = [(0.0, 0.0, 1.0)]
    result    = score_arcs(arcs, goals, obstacles=obstacles)
    feasible  = [r for r in result if r[1]]
    assert len(feasible) == 0, \
        f"Expected zero feasible arcs, got {len(feasible)}"
    print(f"  PASS: all trajectories blocked → safety override would trigger")


def test_arc_count():
    """Exactly K=21 arcs are generated."""
    arcs = precompute_arcs(K=21)
    assert len(arcs) == 21, f"Expected 21 arcs, got {len(arcs)}"
    print(f"  PASS: {len(arcs)} arcs generated")


def test_arc_omega_range():
    """Arc angular velocities span [-0.6, +0.6] rad/s."""
    arcs   = precompute_arcs()
    omegas = [a['omega'] for a in arcs]
    assert abs(omegas[0]  - (-0.6)) < 1e-9, f"Min omega wrong: {omegas[0]}"
    assert abs(omegas[-1] -   0.6)  < 1e-9, f"Max omega wrong: {omegas[-1]}"
    print(f"  PASS: omega range [{omegas[0]:.3f}, {omegas[-1]:.3f}] rad/s")


def test_arc_endpoint_distance():
    """Each arc endpoint should be ≈ v*T = 0.3 m from origin (straight arc)."""
    arcs = precompute_arcs()
    # Straight arc is the middle one (omega = 0)
    mid = len(arcs) // 2
    x, y = arcs[mid]['pts'][-1]
    dist = math.sqrt(x*x + y*y)
    expected = 0.3 * 1.0  # v * T
    assert abs(dist - expected) < 0.01, \
        f"Straight arc endpoint distance {dist:.4f} != {expected:.4f}"
    print(f"  PASS: straight arc endpoint at ({x:.4f}, {y:.4f}), dist={dist:.4f} m")


# ============================================================
# Runner
# ============================================================

TESTS = [
    test_arc_count,
    test_arc_omega_range,
    test_arc_endpoint_distance,
    test_straight_goals,
    test_left_turn,
    test_right_turn,
    test_obstacle_blocks_straight,
    test_all_blocked,
]

if __name__ == '__main__':
    print("=" * 55)
    print("  VTR Phase 4 – Scoring & Trajectory Unit Tests")
    print("=" * 55)
    passed = 0
    failed = 0
    for test in TESTS:
        print(f"\n[{test.__name__}]")
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"  FAIL: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR: {e}")
            failed += 1

    print("\n" + "=" * 55)
    print(f"  Results: {passed}/{len(TESTS)} passed, {failed} failed")
    print("=" * 55)
    sys.exit(0 if failed == 0 else 1)
