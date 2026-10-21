"""Build the original RapidOCR recognizer for this NX's TensorRT GPU runtime.

Use in the NX vision environment; builds may take several minutes. No cameras,
robot commands or system configuration are touched. The source ONNX is read
only. Output is device/version specific; do not distribute it as a portable model.
"""

import hashlib
import json
from pathlib import Path
import sysconfig
import time

import onnx
import tensorrt as trt


def build():
    """Build FP16 with independent height/width names and persist timing cache.

    RapidOCR's ONNX assigns '?' to both spatial axes. TensorRT interprets equal
    names as equal dimensions; rename them before optimization to keep the
    original arbitrary-width OCR input. Only input metadata changes in memory.
    """
    root = Path("/home/a/vision-setup")
    model = (Path(sysconfig.get_paths()["purelib"]) / "rapidocr_onnxruntime"
             / "models" / "ch_PP-OCRv3_rec_infer.onnx")
    output = root / "engines" / "ppocrv3-rec-fp16.plan"
    cache_path = root / "engines" / "ppocrv3-rec-timing.cache"
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    # Repair names before parsing: TRT has already tied equal symbols together
    # once its parser finishes, so renaming the parsed ITensor is too late.
    graph = onnx.load(str(model))
    shape = graph.graph.input[0].type.tensor_type.shape.dim
    shape[0].dim_value = 1
    shape[2].dim_value = 48
    shape[3].dim_param = "width"
    onnx.checker.check_model(graph)
    if not parser.parse(graph.SerializeToString()):
        raise RuntimeError("\n".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    tensor = network.get_input(0)
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 512 * 1024 * 1024)
    if not builder.platform_has_fast_fp16:
        raise RuntimeError("This GPU does not provide fast FP16")
    config.set_flag(trt.BuilderFlag.FP16)
    config.avg_timing_iterations = 1
    profile = builder.create_optimization_profile()
    profile.set_shape(tensor.name, (1, 3, 48, 8), (1, 3, 48, 64), (1, 3, 48, 320))
    if not profile:
        raise RuntimeError("Invalid TensorRT optimization profile")
    config.add_optimization_profile(profile)
    cache = config.create_timing_cache(cache_path.read_bytes() if cache_path.exists() else b"")
    config.set_timing_cache(cache, False)
    start = time.monotonic()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT engine build failed")
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        raise RuntimeError("Cannot deserialize the new engine")
    actual = [tuple(shape) for shape in engine.get_profile_shape(0, 0)]
    expected = [(1, 3, 48, 8), (1, 3, 48, 64), (1, 3, 48, 320)]
    if actual != expected:
        raise RuntimeError("TensorRT changed dynamic-width profile: " + str(actual))
    output.parent.mkdir(exist_ok=True)
    temporary = output.with_suffix(".pending")
    temporary.write_bytes(bytes(serialized))
    temporary.replace(output)
    cache_path.write_bytes(bytes(config.get_timing_cache().serialize()))
    report = {"status": "PASS", "tensorrt": trt.__version__,
              "model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
              "engine_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
              "input_profile": actual, "precision": "FP32+FP16",
              "build_seconds": time.monotonic() - start,
              "renamed_spatial_dimensions": True}
    (root / "engine-build.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    build()
