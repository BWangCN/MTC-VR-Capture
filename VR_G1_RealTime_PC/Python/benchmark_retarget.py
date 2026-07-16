#!/usr/bin/env python3
"""
Benchmark GMR retarget performance for real-time use
"""

import sys
import time
import numpy as np

# Add GMR path
GMR_PATH = r"C:\Users\Wayne\Desktop\GMU\PhD\GMR"
sys.path.insert(0, GMR_PATH)

from general_motion_retargeting import GeneralMotionRetargeting

def generate_test_frame():
    """Generate a dummy PICO frame (T-pose)"""
    positions = np.random.rand(24, 3) * 0.1 + np.array([0, 1, 0])
    rotations = np.tile([1.0, 0.0, 0.0, 0.0], (24, 1))  # Identity quaternions (wxyz)

    joint_names = [
        "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
        "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
        "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
        "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand"
    ]

    frame = {}
    for i, name in enumerate(joint_names):
        frame[name] = [positions[i].tolist(), rotations[i].tolist()]

    return frame

def benchmark_retarget(num_frames=100):
    """Benchmark GMR retarget performance"""

    print(f"Initializing GMR...")
    start_init = time.time()
    gmr = GeneralMotionRetargeting(
        src_human="xrobot",
        tgt_robot="unitree_g1",
        verbose=False,
    )
    init_time = time.time() - start_init
    print(f"Initialization: {init_time:.3f}s")

    # Warmup
    print(f"\nWarming up (10 frames)...")
    test_frame = generate_test_frame()
    for _ in range(10):
        gmr.retarget(test_frame)

    # Benchmark
    print(f"\nBenchmarking {num_frames} frames...")
    times = []

    for i in range(num_frames):
        frame = generate_test_frame()  # Simulate varying input

        start = time.perf_counter()
        qpos = gmr.retarget(frame)
        end = time.perf_counter()

        times.append((end - start) * 1000)  # Convert to ms

    # Statistics
    times = np.array(times)
    mean_time = np.mean(times)
    std_time = np.std(times)
    min_time = np.min(times)
    max_time = np.max(times)
    p95_time = np.percentile(times, 95)

    print(f"\n{'='*50}")
    print(f"BENCHMARK RESULTS ({num_frames} frames)")
    print(f"{'='*50}")
    print(f"Mean:   {mean_time:.2f} ms/frame")
    print(f"Std:    {std_time:.2f} ms")
    print(f"Min:    {min_time:.2f} ms")
    print(f"Max:    {max_time:.2f} ms")
    print(f"P95:    {p95_time:.2f} ms")
    print(f"\n{'='*50}")
    print(f"REAL-TIME CAPABILITY")
    print(f"{'='*50}")

    # Calculate achievable frame rates
    fps_mean = 1000 / mean_time
    fps_p95 = 1000 / p95_time

    print(f"Average FPS: {fps_mean:.1f} Hz")
    print(f"P95 FPS:     {fps_p95:.1f} Hz")

    # Check real-time requirements
    print(f"\n{'='*50}")
    print(f"LATENCY BUDGET")
    print(f"{'='*50}")

    targets = [
        (72, "PICO Native (72Hz)", 13.9),
        (60, "High Quality (60Hz)", 16.7),
        (30, "Acceptable (30Hz)", 33.3),
    ]

    for target_fps, desc, budget_ms in targets:
        if mean_time <= budget_ms:
            status = "✅ CAN ACHIEVE"
        elif p95_time <= budget_ms:
            status = "⚠️  MARGINAL (p95 fits)"
        else:
            status = "❌ TOO SLOW"

        print(f"{desc:25} | Budget: {budget_ms:5.1f}ms | {status}")

    print(f"\n{'='*50}")
    print(f"RECOMMENDATION")
    print(f"{'='*50}")

    if mean_time < 14:
        print("✅ Excellent! Can achieve 72Hz real-time retarget.")
        print("   No optimization needed.")
    elif mean_time < 20:
        print("✅ Good! Can achieve 50-60Hz real-time retarget.")
        print("   Consider minor optimizations for 72Hz.")
    elif mean_time < 33:
        print("⚠️  Acceptable. Can achieve 30Hz real-time retarget.")
        print("   Recommend frame skipping or interpolation for 72Hz.")
    else:
        print("❌ Too slow for real-time use.")
        print("   Need significant optimization or async processing.")

if __name__ == "__main__":
    benchmark_retarget(num_frames=100)
