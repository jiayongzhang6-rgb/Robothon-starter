"""
Aegis Campus Patrol v6 - Individual Benchmark Tests
Tests specific subsystems: recovery, planning, decision latency.
"""
import sys, os, time, json, random
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))


def test_recovery_protocol():
    """Test 3-phase recovery: re-observe, re-localize, re-plan."""
    from robot.policy.recovery import AdaptiveRecovery
    
    recovery = AdaptiveRecovery()
    results = {"trials": 0, "success": 0, "phases_completed": []}
    
    for i in range(50):
        random.seed(i)
        recovery.reset()
        
        # Simulate confidence drop
        state = {"confidence": 0.2, "position": (5.0, 3.0), "base": (1.5, 1.5)}
        
        # Run recovery phases
        phases = []
        for phase in ["re_observe", "re_localize", "re_plan"]:
            ok = recovery.execute_phase(phase, state)
            phases.append(ok)
        
        results["trials"] += 1
        results["phases_completed"].append(sum(phases))
        if sum(phases) == 3:
            results["success"] += 1
    
    results["success_rate"] = round(results["success"] / results["trials"] * 100, 1)
    return results


def test_node_selection():
    """Test intelligent node selection vs random."""
    from robot.policy.route_sampler import RouteSampler
    
    sampler = RouteSampler()
    results = {"intelligent": [], "random": []}
    
    for i in range(50):
        random.seed(i)
        
        # Intelligent selection
        state = {"position": (1.5, 1.5), "visited": set(), "confidence": 0.8}
        t0 = time.time()
        node = sampler.select_best(state)
        t1 = time.time()
        results["intelligent"].append(t1 - t0)
        
        # Random selection
        t0 = time.time()
        all_nodes = [(x, y) for x in range(2, 10) for y in range(1, 7)]
        random.choice(all_nodes)
        t1 = time.time()
        results["random"].append(t1 - t0)
    
    return {
        "intelligent_avg_ms": round(sum(results["intelligent"]) / len(results["intelligent"]) * 1000, 2),
        "random_avg_ms": round(sum(results["random"]) / len(results["random"]) * 1000, 2),
    }


def test_decision_latency():
    """Measure decision loop latency."""
    from robot.policy.patrol_policy import PatrolPolicy
    
    policy = PatrolPolicy()
    latencies = []
    
    for i in range(100):
        readings = {
            "position": (random.uniform(1, 9), random.uniform(1, 6)),
            "battery": random.uniform(0.5, 1.0),
            "obstacle_near": random.random() < 0.2,
            "confidence": random.uniform(0.3, 1.0),
        }
        
        t0 = time.time()
        action = policy.decide(readings)
        t1 = time.time()
        latencies.append((t1 - t0) * 1000)
    
    return {
        "avg_ms": round(sum(latencies) / len(latencies), 2),
        "p95_ms": round(sorted(latencies)[int(len(latencies) * 0.95)], 2),
        "max_ms": round(max(latencies), 2),
    }


def main():
    print("=" * 50)
    print("Aegis v6 - Subsystem Benchmarks")
    print("=" * 50)
    
    all_results = {}
    
    print("\n[1/3] Recovery Protocol Test...")
    try:
        r = test_recovery_protocol()
        all_results["recovery"] = r
        print(f"  Success rate: {r['success_rate']}%")
    except Exception as e:
        print(f"  Skipped: {e}")
    
    print("\n[2/3] Node Selection Test...")
    try:
        r = test_node_selection()
        all_results["node_selection"] = r
        print(f"  Intelligent: {r['intelligent_avg_ms']}ms  Random: {r['random_avg_ms']}ms")
    except Exception as e:
        print(f"  Skipped: {e}")
    
    print("\n[3/3] Decision Latency Test...")
    try:
        r = test_decision_latency()
        all_results["latency"] = r
        print(f"  Avg: {r['avg_ms']}ms  P95: {r['p95_ms']}ms  Max: {r['max_ms']}ms")
    except Exception as e:
        print(f"  Skipped: {e}")
    
    # Save
    out = Path(__file__).parent / "results.json"
    with open(out, "w") as f:
        json.dump(all_results, f, indent=2)
    
    print(f"\nResults saved to {out}")
    print("=" * 50)


if __name__ == "__main__":
    main()
