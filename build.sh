#!/usr/bin/env bash
# Build on Linux / macOS. Uses nvcc (GPU planner) if available, otherwise a CPU-only build.
set -e
cd "$(dirname "$0")"
mkdir -p bin
# macOS: some Command Line Tools ship an SDK newer than their linker; use the default SDK link
if [ "$(uname)" = "Darwin" ] && [ -z "$SDKROOT" ] && [ -d /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk ]; then
  export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
fi
if command -v nvcc >/dev/null 2>&1; then
  nvcc -O3 -std=c++17 -Xcompiler -pthread -o bin/firealert src/firealert.cu
  echo "Built bin/firealert with CUDA (GPU planner)"
else
  ${CXX:-c++} -O2 -std=c++17 -pthread -Wall -x c++ src/firealert.cu -o bin/firealert
  echo "nvcc not found: built bin/firealert CPU-only (same algorithm, no GPU)"
fi
