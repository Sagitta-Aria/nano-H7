"""Validate the NX OCR GPU environment using source text supplied on stdin.

Input is a JSON object mapping original module names to their Python source.
Run only in the NX vision environment after building its TensorRT engine.
This benchmark uses generated pictures, never cameras or robot commands;
timings include Python/host transfers and are not live-camera FPS claims.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import time
import types

import cv2
import numpy as np

from nx_vision_trt import accelerate


def load_modules(sources):
    """Load trusted workspace source in dependency order without calling main()."""
    modules = {}
    for name in ("black_ring", "digit_core", "ring_digit"):
        module = types.ModuleType(name)
        sys.modules[name] = module
        exec(compile(sources[name], name + ".py", "exec"), module.__dict__)
        modules[name] = module
    return modules


def sample_frame(digit, font=cv2.FONT_HERSHEY_SIMPLEX):
    """Generate one white 640x480 frame with a centered black ring and digit."""
    image = np.full((480, 640, 3), 255, np.uint8)
    cv2.circle(image, (320, 240), 90, (0, 0, 0), 8)
    size, _ = cv2.getTextSize(digit, font, 2, 4)
    cv2.putText(image, digit, (320 - size[0] // 2, 240 + size[1] // 2),
                font, 2, (0, 0, 0), 4, cv2.LINE_AA)
    return image


def summary(milliseconds):
    """Summarize measured wall times; reciprocal throughput excludes camera/display."""
    mean = float(np.mean(milliseconds))
    return {"samples": len(milliseconds), "mean_ms": mean,
            "median_ms": float(np.median(milliseconds)),
            "p95_ms": float(np.percentile(milliseconds, 95)),
            "max_ms": float(np.max(milliseconds)),
            "compute_only_fps": 1000 / mean}


def measure(operation):
    """Measure a completed synchronous operation with a monotonic host timer."""
    start = time.perf_counter()
    result = operation()
    return result, (time.perf_counter() - start) * 1000


def benchmark(modules, count):
    """Compare CPU and TensorRT correctness, worker-thread use and full frame costs."""
    core, ring, combined = (modules[k] for k in ("digit_core", "black_ring", "ring_digit"))
    cv2.setNumThreads(1)
    cpu = core.make_engine()
    gpu = accelerate(core.make_engine())
    report = {"opencv": cv2.__version__, "numpy": np.__version__,
              "frame_size": [640, 480], "cpu_opencv_threads": 1,
              "camera_used": False, "gpu_backend": "TensorRT FP16"}
    samples = []
    try:
        for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX,
                     cv2.FONT_HERSHEY_TRIPLEX):
            for digit in "123":
                frame = sample_frame(digit, font)
                detected = ring.find_smallest_ring(frame)
                assert detected is not None, "Synthetic ring not detected"
                crop = combined.extract_ring_interior(frame, detected)
                samples.append((digit, frame, crop))
        report["correctness"] = []
        for expected, _, crop in samples:
            cpu_result = core.recognize(cpu, crop, 0.60, 0.30)
            gpu_result = core.recognize(gpu, crop, 0.60, 0.30)
            row = {"expected": expected, "cpu_digit": cpu_result[0],
                   "gpu_digit": gpu_result[0], "cpu_score": cpu_result[1],
                   "gpu_score": gpu_result[1]}
            report["correctness"].append(row)
            print("CORRECTNESS", row, flush=True)
            assert cpu_result[0] == gpu_result[0], "FP16 changed decoded output"
        assert all(row["cpu_digit"] == row["expected"]
                   for row in report["correctness"]), "Synthetic reference recognition failed"
        # The real application creates its engine before submitting OCR to a worker.
        with ThreadPoolExecutor(max_workers=1) as pool:
            value = pool.submit(core.recognize, gpu, samples[0][2], 0.60, 0.30).result(timeout=10)
            assert value[0] == samples[0][0]
        report["worker_thread"] = "PASS"
        # Check that changing width is supported, and invalid shapes fail explicitly.
        for width in (8, 17, 48, 96, 320):
            output = gpu.nx_trt_session(np.zeros((1, 3, 48, width), np.float32))[0]
            assert output.ndim == 3 and output.shape[0] == 1 and output.shape[-1] == 6625
            assert np.all(np.isfinite(output))
        try:
            gpu.nx_trt_session(np.zeros((1, 3, 48, 321), np.float32))
        except ValueError:
            pass
        else:
            raise AssertionError("Out-of-profile input was accepted")
        report["dynamic_widths"] = "PASS"
        times = {"cpu_ocr": [], "gpu_ocr": [], "ring_and_crop": [],
                 "cpu_sequential_pipeline": [], "gpu_sequential_pipeline": []}
        for index in range(count):
            expected, frame, crop = samples[index % len(samples)]
            # Alternate ordering to reduce simple warmup/clock ordering bias.
            pairs = (("cpu_ocr", cpu), ("gpu_ocr", gpu))
            if index % 2:
                pairs = pairs[::-1]
            for name, engine in pairs:
                result, elapsed = measure(lambda: core.recognize(engine, crop, 0.60, 0.30))
                assert result[0] == expected
                times[name].append(elapsed)

            def geometry():
                """Run original full-frame detection and ring-interior extraction."""
                detected = ring.find_smallest_ring(frame)
                return combined.extract_ring_interior(frame, detected)

            _, elapsed = measure(geometry)
            times["ring_and_crop"].append(elapsed)
            for name, engine in (("cpu_sequential_pipeline", cpu),
                                 ("gpu_sequential_pipeline", gpu)):
                def pipeline():
                    """Measure geometry plus OCR in sequence, without camera or GUI."""
                    return core.recognize(engine, geometry(), 0.60, 0.30)

                result, elapsed = measure(pipeline)
                assert result[0] == expected
                times[name].append(elapsed)
        report["timings"] = {key: summary(values) for key, values in times.items()}
        report["ocr_speedup"] = (report["timings"]["cpu_ocr"]["mean_ms"]
                                 / report["timings"]["gpu_ocr"]["mean_ms"])
        report["status"] = "PASS"
        return report
    finally:
        gpu.nx_trt_session.close()


def main():
    """Read the source payload, validate on GPU and persist reproducible measurements."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--output", default="/home/a/vision-setup/gpu-benchmark.json")
    args = parser.parse_args()
    if args.samples < 3:
        parser.error("--samples must be at least 3")
    sources = json.load(sys.stdin)
    report = benchmark(load_modules(sources), args.samples)
    report["source_sha256"] = {name: hashlib.sha256(source.encode("utf-8")).hexdigest()
                               for name, source in sources.items()}
    Path(args.output).write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
