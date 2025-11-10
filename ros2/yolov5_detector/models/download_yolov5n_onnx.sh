#!/usr/bin/env bash

set -euo pipefail

MODEL_URL="https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx"
MODEL_FILE="yolov5n.onnx"
CHECKSUM="04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}"

if [ -f "${MODEL_FILE}" ]; then
  echo "${MODEL_FILE} already exists. Skipping download."
  exit 0
fi

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

echo "Download complete: ${MODEL_FILE}"
