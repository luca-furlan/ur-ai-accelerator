#!/usr/bin/env bash

set -euo pipefail

MODEL_URL="https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx"
MODEL_FILE="yolov5n.onnx"
CHECKSUM="04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}"

if [ -f "${MODEL_FILE}" ]; then
  echo "${MODEL_FILE} already exists. Skipping download."
else
  echo "Downloading ${MODEL_FILE}..."
  wget -q --show-progress "${MODEL_URL}" -O "${MODEL_FILE}"

  if command -v sha256sum >/dev/null 2>&1; then
    echo "Verifying checksum..."
    actual_checksum="$(sha256sum "${MODEL_FILE}" | awk '{print $1}')"
    if [ "${actual_checksum}" != "${CHECKSUM}" ]; then
      echo "Checksum mismatch for ${MODEL_FILE}" >&2
      exit 1
    fi
  fi
fi

# Convert to FP32 if python + onnx are available
if command -v python3 >/dev/null 2>&1; then
  if python3 -c "import onnx" >/dev/null 2>&1; then
    echo "Ensuring FP32 weights..."
    python3 - <<'PY'
import onnx
from onnx import numpy_helper
import shutil

model_path = 'yolov5n.onnx'
output_path = 'yolov5n_fp32.onnx'

model = onnx.load(model_path)
updated = False
for tensor in model.graph.initializer:
    if tensor.data_type == onnx.TensorProto.FLOAT16:
        arr = numpy_helper.to_array(tensor).astype('float32')
        tensor.CopyFrom(numpy_helper.from_array(arr, tensor.name))
        tensor.data_type = onnx.TensorProto.FLOAT
        updated = True

if updated:
    onnx.save(model, output_path)
    print(f'Converted to FP32 -> {output_path}')
else:
    shutil.copy(model_path, output_path)
    print(f'Model already FP32 -> {output_path}')
PY
  else
    echo "python3-onnx not installed; keeping original model in ${MODEL_FILE}." >&2
  fi
else
  echo "python3 not available; keeping original model in ${MODEL_FILE}." >&2
fi
