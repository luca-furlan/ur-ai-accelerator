# YOLOv5 Models

This folder stores lightweight metadata for YOLOv5 object detection.

## Files

- `coco_labels.txt` – 80 labels for the COCO dataset.
- `download_yolov5n_onnx.sh` – helper script to download the YOLOv5 Nano ONNX weights.
- `yolov5n.onnx` – **not committed**; download with the script or place your custom weights here.

## Download instructions

```bash
cd $(dirname "$0")
./download_yolov5n_onnx.sh
```

The script downloads the YOLOv5 nano ONNX model (~8 MB) from the Ultralytics release page and verifies the checksum.

You can replace the weights with any YOLOv5 ONNX model, updating the ROS parameter `weights_path` accordingly.
