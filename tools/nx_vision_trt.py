"""TensorRT adapter and launcher for the NX RapidOCR 1.2.3 environment.

Use on Xavier NX with TensorRT 8.5 and the locally built recognition engine.
Only the OCR recognition session runs on GPU; OpenCV geometry stays on CPU.
This module does not select a camera, own robot I/O, or change OCR decoding.
Launch an already deployed script with ``python -m nx_vision_trt SCRIPT ARGS``.
"""

import ctypes
import os
from pathlib import Path
import runpy
import sys
import threading

import numpy as np
import tensorrt as trt


DEFAULT_ENGINE = "/home/a/vision-setup/engines/ppocrv3-rec-fp16.plan"


class CudaRuntime:
    """Own checked CUDA runtime bindings; use with this device's primary context.

    No PyCUDA context is created. Calls raise on CUDA errors instead of silently
    using CPU. This is a small TensorRT transport helper, not a general CUDA API.
    """

    def __init__(self):
        """Load JetPack's runtime and declare pointer-safe C signatures."""
        self.lib = ctypes.CDLL("/usr/local/cuda/lib64/libcudart.so")
        signatures = {
            "cudaSetDevice": [ctypes.c_int],
            "cudaMalloc": [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t],
            "cudaFree": [ctypes.c_void_p],
            "cudaMemcpy": [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int],
        }
        for name, arguments in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes = arguments
            function.restype = ctypes.c_int
        self.lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p

    def call(self, name, *arguments):
        """Run a bound function and expose failures with their CUDA explanation."""
        status = getattr(self.lib, name)(*arguments)
        if status:
            message = self.lib.cudaGetErrorString(status).decode("utf-8", "replace")
            raise RuntimeError("{}: {} ({})".format(name, message, status))


class TensorRTSession:
    """Replace RapidOCR's single-input recognition session with GPU inference.

    Calls accept NCHW float32 arrays and return the original output list format.
    One lock protects the execution context and reusable device buffers. The
    built engine fixes batch=1, height=48 and permits widths 8 through 320.
    Do not use for detector/classifier networks, batching or other TRT versions.
    """

    def __init__(self, engine_path=DEFAULT_ENGINE):
        """Deserialize a trusted local engine; allocate buffers lazily on first use."""
        self.lock = threading.Lock()
        self.buffers = {}
        self.closed = False
        self.cuda = CudaRuntime()
        self.cuda.call("cudaSetDevice", 0)
        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(engine_path).read_bytes())
        if self.engine is None:
            raise RuntimeError("Cannot load TensorRT engine: " + str(engine_path))
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("Cannot create TensorRT execution context")
        if self.engine.num_optimization_profiles != 1:
            raise ValueError("Expected one TensorRT optimization profile")
        self.names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.input_indices = [i for i, name in enumerate(self.names)
                              if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT]
        self.output_indices = [i for i, name in enumerate(self.names)
                               if self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT]
        if len(self.input_indices) != 1 or len(self.output_indices) != 1:
            raise ValueError("Expected one recognition input and one output")
        self.minimum, _, self.maximum = self.engine.get_tensor_profile_shape(
            self.names[self.input_indices[0]], 0)
        if tuple(self.minimum) != (1, 3, 48, 8) or tuple(self.maximum) != (1, 3, 48, 320):
            raise ValueError("Unexpected engine profile; rebuild with build_nx_vision_engine.py")
        # TRT 8.5's nptype helper references np.bool, removed in NumPy 1.24.
        self.dtypes = {trt.DataType.FLOAT: np.float32, trt.DataType.HALF: np.float16,
                       trt.DataType.INT32: np.int32, trt.DataType.INT8: np.int8,
                       trt.DataType.BOOL: np.bool_}

    def _buffer(self, index, byte_count):
        """Reuse or grow one binding's CUDA allocation; never expose it to callers."""
        pointer, capacity = self.buffers.get(index, (ctypes.c_void_p(), 0))
        if capacity < byte_count:
            if pointer.value:
                self.cuda.call("cudaFree", pointer)
                self.buffers.pop(index, None)
            pointer = ctypes.c_void_p()
            self.cuda.call("cudaMalloc", ctypes.byref(pointer), byte_count)
            self.buffers[index] = (pointer, byte_count)
        return pointer

    def __call__(self, tensor):
        """Execute synchronously on GPU and return host output, including transfers.

        Select the CUDA primary context in the calling thread, so the original
        ThreadPoolExecutor may invoke an engine constructed on the main thread.
        Out-of-profile input raises; there is no hidden CPU fallback or resize.
        """
        with self.lock:
            if self.closed:
                raise RuntimeError("TensorRT session is closed")
            self.cuda.call("cudaSetDevice", 0)
            input_index = self.input_indices[0]
            input_name = self.names[input_index]
            dtype = self.dtypes[self.engine.get_tensor_dtype(input_name)]
            tensor = np.ascontiguousarray(tensor, dtype=dtype)
            if (len(tensor.shape) != len(self.minimum)
                    or any(n < lo or n > hi for n, lo, hi in
                           zip(tensor.shape, self.minimum, self.maximum))):
                raise ValueError("Input {} outside TensorRT profile {}..{}".format(
                    tensor.shape, tuple(self.minimum), tuple(self.maximum)))
            if not self.context.set_input_shape(input_name, tensor.shape):
                raise RuntimeError("TensorRT rejected input shape")
            bindings = [0] * len(self.names)
            device_input = self._buffer(input_index, tensor.nbytes)
            bindings[input_index] = device_input.value
            outputs = []
            for index in self.output_indices:
                shape = tuple(self.context.get_tensor_shape(self.names[index]))
                if any(n <= 0 for n in shape):
                    raise RuntimeError("Unresolved TensorRT output shape: " + str(shape))
                output = np.empty(shape, dtype=self.dtypes[self.engine.get_tensor_dtype(self.names[index])])
                bindings[index] = self._buffer(index, output.nbytes).value
                outputs.append((index, output))
            self.cuda.call("cudaMemcpy", device_input, ctypes.c_void_p(tensor.ctypes.data),
                           tensor.nbytes, 1)
            if not self.context.execute_v2(bindings):
                raise RuntimeError("TensorRT GPU execution failed")
            for index, output in outputs:
                self.cuda.call("cudaMemcpy", ctypes.c_void_p(output.ctypes.data),
                               self.buffers[index][0], output.nbytes, 2)
            return [output for _, output in outputs]

    def close(self):
        """Free device buffers after pending calls; the session cannot be reused."""
        with self.lock:
            if self.closed:
                return
            self.cuda.call("cudaSetDevice", 0)
            for pointer, _ in self.buffers.values():
                self.cuda.call("cudaFree", pointer)
            self.buffers.clear()
            self.closed = True
            self.context = None
            self.engine = None
            self.runtime = None


def accelerate(engine, engine_path=DEFAULT_ENGINE):
    """Replace only an initialized RapidOCR recognizer's inference session.

    Call after digit_core.make_engine() so its character table and custom CTC
    decoder are preserved. Keep the returned engine for normal recognize()
    calls. The GPU session is exposed as engine.nx_trt_session for close().
    """
    if getattr(engine, "nx_trt_session", None) is not None:
        raise ValueError("Engine is already using TensorRT")
    session = TensorRTSession(engine_path)
    if hasattr(engine, 'text_recognizer'):
        engine.text_recognizer.session = session
    else:
        # New DigitEngine uses the ONNX Runtime run(output_names, feeds) API.
        class OnnxAdapter:
            """Bridge the existing TensorRT tensor callable to the new OCR engine."""
            def run(self, output_names, feeds):
                return session(next(iter(feeds.values())))
        engine.session = OnnxAdapter()
    engine.nx_trt_session = session
    return engine


def main():
    """Launch an existing digit_core/ring_digit script using the GPU factory.

    Source scripts are untouched. All arguments after SCRIPT pass through to
    that script, including camera selection and GUI/offline options. This
    launcher starts whatever script the user supplies; it is not a service.
    """
    if len(sys.argv) < 2:
        raise SystemExit("Usage: python -m nx_vision_trt SCRIPT.py [script arguments]")
    script = Path(sys.argv[1]).resolve(strict=True)
    sys.path.insert(0, str(script.parent))
    import digit_core
    original_factory = digit_core.make_engine
    sessions = []

    def gpu_factory():
        """Preserve the original OCR setup, then require the local GPU engine."""
        engine = accelerate(original_factory(), os.environ.get("NX_VISION_TRT_ENGINE", DEFAULT_ENGINE))
        sessions.append(engine.nx_trt_session)
        print("OCR backend: TensorRT GPU (FP16-enabled engine)", file=sys.stderr)
        return engine

    digit_core.make_engine = gpu_factory
    sys.argv = sys.argv[1:]
    try:
        if script.name == "digit_core.py":
            digit_core.main()
        else:
            runpy.run_path(str(script), run_name="__main__")
    finally:
        for session in sessions:
            session.close()


if __name__ == "__main__":
    main()
