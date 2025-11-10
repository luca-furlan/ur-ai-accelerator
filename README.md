# Progetto Robot e AI Accelerator

Repository per il progetto con robot e AI accelerator.

## Informazioni di Rete

Le informazioni di rete e le credenziali SSH sono disponibili nel file `network_info.txt`.

## Documentazione

- `network_info.txt` - IP e credenziali SSH per Robot e AI Accelerator
- `Documento Ai Accelerator - Avanamanto Progetto Demo.txt` - Guida setup UR AI Accelerator (formato testo)
- `Documento Ai Accelerator - Avanamanto Progetto Demo.docx` - Documentazione completa del progetto (formato Word)

## Camera Streaming

- Il driver Orbbec Gemini 335Lg è documentato in `Documento Ai Accelerator - Avanamanto Progetto Demo.txt`
- Utilizzare `scripts/view_orbbec_camera.sh` sul Jetson per aprire rapidamente `rqt_image_view` o `image_tools showimage`

## Object Recognition

- Pacchetto ROS2 Python: `ros2/yolov5_detector`
  - Sottoscrive `/camera/color/image_raw`, effettua inferenza YOLOv5 (OpenCV DNN) e pubblica
    - `detections` (`vision_msgs/Detection2DArray`)
    - `/camera/color/yolov5_annotated` (immagine con bounding box)
  - Configurazione via `config/params.yaml` o parametri launch.
- Modelli
  - Scaricare `yolov5n.onnx` eseguendo `ros2/yolov5_detector/models/download_yolov5n_onnx.sh` (richiede `python3 -m pip install onnx onnxruntime` per conversione FP32)
  - Classi COCO in `ros2/yolov5_detector/models/coco_labels.txt`
- Lancio rapito (Jetson)
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  colcon build --packages-select yolov5_detector
  ros2 launch yolov5_detector yolov5_detector.launch.py
  ```
- Parametri principali
  - `image_topic`: topic camera (default `/camera/color/image_raw`)
  - `weights_path`: percorso ONNX (default share package)
  - `use_cuda`: `true` per DNN backend CUDA sul Jetson

