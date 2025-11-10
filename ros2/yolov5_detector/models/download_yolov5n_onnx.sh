#!/usr/bin/env bash

set -euo pipefail

MODEL_URL="https://raw.githubusercontent.com/doleron/yolov5-opencv-cpp-python/main/config_files/yolov5n.onnx"
MODEL_FILE="yolov5n.onnx"
CHECKSUM="4a109040f024ad5e0d9f32836f2ef960ca89fdc4"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}"

if [ -f "${MODEL_FILE}" ]; then
  echo "${MODEL_FILE} already exists. Skipping download."
  exit 0
fi

echo "Downloading ${MODEL_FILE} (OpenCV-ready)..."
wget -q --show-progress "${MODEL_URL}" -O "${MODEL_FILE}.tmp"

if command -v sha256sum >/dev/null 2>&1; then
  echo "Verifying checksum..."
  actual_checksum="$(sha256sum "${MODEL_FILE}.tmp" | awk '{print $1}')"
  if [ "${actual_checksum}" != "${CHECKSUM}" ]; then
    echo "Checksum mismatch for ${MODEL_FILE}" >&2
    rm -f "${MODEL_FILE}.tmp"
    exit 1
  fi
fi

mv "${MODEL_FILE}.tmp" "${MODEL_FILE}"
echo "Saved ${MODEL_FILE}"
