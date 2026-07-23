# Third-party runtime dependencies

This project intentionally does not version architecture-specific ONNX Runtime
binaries.  Download ONNX Runtime **1.22.0** for the same architecture as the
machine that will execute the controller, unpack it locally, and point CMake at
the resulting directory.

Expected local layouts are:

```text
thirdparty/
├── onnxruntime-linux-x64-1.22.0/       # x86_64 development / simulation host
└── onnxruntime-linux-aarch64-1.22.0/   # Unitree G1 PC2 real robot
```

The default path is selected from CMake's target processor.  An installation in
another location is supported:

```bash
cmake -S . -B build \
  -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.22.0
```

Use the official ONNX Runtime release package and retain its license notice
when redistributing a runnable image.  The `lib/` versus `lib64/` package
layout is detected by this project's CMake configuration.

