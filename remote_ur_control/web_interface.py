"""
Simple web interface for driving the UR robot from the AI Accelerator.

Run with:
    export UR_ROBOT_IP=192.168.10.194  # adjust to your robot
    python -m remote_ur_control.web_interface
"""

from __future__ import annotations

import os
import sys
import traceback
from dataclasses import asdict, dataclass
from typing import Dict, List

import threading
import time
import logging
import math
import numpy as np
from collections import deque
from datetime import datetime

# Aggiungi path per ros2_bridge
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import Flask, jsonify, render_template_string, request, Response
import socket
import io
import base64

from .remote_ur_controller import MoveParameters, RemoteURController, DashboardClient

# ROS2 bridge (SOLUZIONE PRINCIPALE)
_ros2_bridge = None
_ros2_import_warned = False
try:
    from ros2_bridge_fixed import ROS2Bridge
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    ROS2Bridge = None
    # Log solo una volta all'import
    if not _ros2_import_warned:
        print(f"[WARN] ROS2 bridge non disponibile: {e}")
        print(f"   Suggerimento: avvia web interface con: bash avvia_web_interface.sh")
        _ros2_import_warned = True


def get_ros2_bridge():
    """Ottiene il bridge ROS2 (singleton)."""
    global _ros2_bridge
    if ROS2_AVAILABLE and not _ros2_bridge:
        try:
            _ros2_bridge = ROS2Bridge()
        except Exception as e:
            # Log solo una volta
            if not hasattr(get_ros2_bridge, '_error_logged'):
                print(f"[WARN] Errore creazione ROS2 bridge: {e}")
                get_ros2_bridge._error_logged = True
            _ros2_bridge = None
    return _ros2_bridge

app = Flask(__name__)

# ROS2 bridge singleton
_ros2_bridge = None

# Camera stream subscriber (per Orbbec)
_camera_frame = None
_camera_frame_lock = threading.Lock()
_camera_subscriber_node = None
_camera_subscriber_thread = None
_camera_bridge = None

# Vision detections subscriber (per YOLO)
_latest_detections = None
_detections_lock = threading.Lock()
_detections_subscriber_node = None
_detections_subscriber_thread = None

# Processi gestiti dalla web interface
_managed_processes = {
    'orbbec_camera': None,  # subprocess.Popen
    'yolo_detector': None,
    'moveit': None
}
_processes_lock = threading.Lock()

def _init_camera_subscriber():
    """Inizializza subscriber ROS2 per camera stream."""
    global _camera_subscriber_node, _camera_subscriber_thread, _camera_bridge
    
    if not ROS2_AVAILABLE or _camera_subscriber_node is not None:
        return
    
    try:
        import rclpy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import cv2
        
        # Assicurati che ROS2 sia inizializzato (il bridge potrebbe averlo già fatto)
        # Usa lo stesso approccio del bridge ROS2 per gestire rclpy.init()
        try:
            try:
                if not rclpy.ok():
                    rclpy.init()
            except RuntimeError as e:
                if 'must only be called once' in str(e) or 'already initialized' in str(e).lower():
                    # ROS2 già inizializzato - va bene, continuiamo
                    pass
                else:
                    raise
        except Exception as e:
            app.logger.warning(f"[CAMERA] Errore inizializzazione rclpy: {e}")
            return
        
        try:
            _camera_bridge = CvBridge()
        except Exception as e:
            app.logger.warning(f"[CAMERA] cv_bridge non disponibile: {e}")
            return
        
        class CameraSubscriberNode:
            def __init__(self):
                try:
                    self.node = rclpy.create_node('camera_stream_subscriber')
                    self.subscription = self.node.create_subscription(
                        Image,
                        '/camera/color/image_raw',
                        self.image_callback,
                        10
                    )
                    app.logger.info("[CAMERA] Subscriber inizializzato per /camera/color/image_raw")
                except Exception as e:
                    app.logger.error(f"[CAMERA] Errore creazione subscriber node: {e}")
                    raise
            
            def image_callback(self, msg):
                global _camera_frame
                try:
                    cv_image = _camera_bridge.imgmsg_to_cv2(msg, "bgr8")
                    _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    with _camera_frame_lock:
                        _camera_frame = buffer.tobytes()
                except Exception as e:
                    app.logger.error(f"[CAMERA] Errore conversione frame: {e}")
        
        _camera_subscriber_node = CameraSubscriberNode()
        
        def spin_node():
            try:
                rclpy.spin(_camera_subscriber_node.node)
            except Exception as e:
                app.logger.error(f"[CAMERA] Errore spin node: {e}")
        
        _camera_subscriber_thread = threading.Thread(target=spin_node, daemon=True)
        _camera_subscriber_thread.start()
        app.logger.info("[CAMERA] Thread subscriber avviato")
        
    except ImportError as e:
        app.logger.warning(f"[CAMERA] Import error (cv_bridge o rclpy non disponibili): {e}")
    except Exception as e:
        app.logger.error(f"[CAMERA] Errore inizializzazione subscriber: {e}")

def _init_detections_subscriber():
    """Inizializza subscriber ROS2 per detections YOLO."""
    global _detections_subscriber_node, _detections_subscriber_thread
    
    if not ROS2_AVAILABLE or _detections_subscriber_node is not None:
        return
    
    try:
        import rclpy
        from std_msgs.msg import String
        import json
        
        try:
            try:
                if not rclpy.ok():
                    rclpy.init()
            except RuntimeError as e:
                if 'must only be called once' in str(e) or 'already initialized' in str(e).lower():
                    pass
                else:
                    raise
        except Exception as e:
            app.logger.warning(f"[DETECTIONS] Errore inizializzazione rclpy: {e}")
            return
        
        class DetectionsSubscriberNode:
            def __init__(self):
                self.node = rclpy.create_node('detections_subscriber')
                self.subscription = self.node.create_subscription(
                    String,
                    '/vision/detections_3d',
                    self.detections_callback,
                    10
                )
                app.logger.info("[DETECTIONS] Subscriber inizializzato per /vision/detections_3d")
            
            def detections_callback(self, msg):
                global _latest_detections
                try:
                    data = json.loads(msg.data)
                    with _detections_lock:
                        _latest_detections = data.get('detections', [])
                except Exception as e:
                    app.logger.error(f"[DETECTIONS] Errore parsing detections: {e}")
        
        _detections_subscriber_node = DetectionsSubscriberNode()
        
        def spin_node():
            try:
                rclpy.spin(_detections_subscriber_node.node)
            except Exception as e:
                app.logger.error(f"[DETECTIONS] Errore spin node: {e}")
        
        _detections_subscriber_thread = threading.Thread(target=spin_node, daemon=True)
        _detections_subscriber_thread.start()
        app.logger.info("[DETECTIONS] Thread subscriber avviato")
        
    except ImportError as e:
        app.logger.warning(f"[DETECTIONS] Import error (rclpy non disponibile): {e}")
    except Exception as e:
        app.logger.error(f"[DETECTIONS] Errore inizializzazione subscriber: {e}")

# Sistema di logging centralizzato
_log_buffer = deque(maxlen=1000)  # Mantieni ultimi 1000 log
_log_lock = threading.Lock()

class WebInterfaceLogHandler(logging.Handler):
    """Handler personalizzato per catturare tutti i log."""
    def emit(self, record):
        try:
            msg = self.format(record)
            level = record.levelname
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            with _log_lock:
                _log_buffer.append({
                    'timestamp': timestamp,
                    'level': level,
                    'message': msg,
                    'module': record.module if hasattr(record, 'module') else 'unknown'
                })
        except:
            pass

# Configura logging Flask
logging.basicConfig(level=logging.INFO)
web_log_handler = WebInterfaceLogHandler()
web_log_handler.setFormatter(logging.Formatter('%(message)s'))
app.logger.addHandler(web_log_handler)
app.logger.setLevel(logging.INFO)

# Cattura anche print() dal bridge ROS2
class PrintCapture:
    """Cattura print() e li converte in log. Implementa interfaccia file-like completa."""
    def __init__(self, real_stdout):
        self.real_stdout = real_stdout
        self.buffer = []
        self._capturing = False  # Flag per evitare ricorsione
        # Attributi necessari per essere un file-like object
        self.mode = getattr(real_stdout, 'mode', 'w')
        self.name = getattr(real_stdout, 'name', '<stdout>')
        self.encoding = getattr(real_stdout, 'encoding', 'utf-8')
        self.errors = getattr(real_stdout, 'errors', 'strict')
        self.newlines = getattr(real_stdout, 'newlines', None)
        self.line_buffering = getattr(real_stdout, 'line_buffering', False)
        self.closed = False
    
    def write(self, text):
        # Evita ricorsione: se stiamo già catturando, scrivi direttamente
        if self._capturing:
            self.real_stdout.write(text)
            return
        
        # Converti bytes a string se necessario
        if isinstance(text, bytes):
            text = text.decode('utf-8', errors='ignore')
        
        # Scrivi sempre sul vero stdout
        self.real_stdout.write(text)
        
        # Cattura solo se non è vuoto
        text_stripped = text.strip()
        if text_stripped:
            self._capturing = True
            try:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                level = 'INFO'
                text_lower = text_stripped.lower()
                if '[ERROR]' in text_stripped or 'error' in text_lower or 'ERROR' in text_stripped:
                    level = 'ERROR'
                elif '[OK]' in text_stripped or 'success' in text_lower or 'OK' in text_stripped:
                    level = 'INFO'
                elif '[WAIT]' in text_stripped or 'waiting' in text_lower or 'WAIT' in text_stripped:
                    level = 'WARNING'
                
                with _log_lock:
                    _log_buffer.append({
                        'timestamp': timestamp,
                        'level': level,
                        'message': text_stripped,
                        'module': 'ros2_bridge'
                    })
            except Exception as e:
                # Se c'è un errore nel logging, ignoralo per evitare ricorsione
                pass
            finally:
                self._capturing = False
    
    def flush(self):
        if hasattr(self.real_stdout, 'flush'):
            self.real_stdout.flush()
    
    def close(self):
        if hasattr(self.real_stdout, 'close'):
            self.real_stdout.close()
        self.closed = True
    
    def isatty(self):
        return hasattr(self.real_stdout, 'isatty') and self.real_stdout.isatty()
    
    def readable(self):
        return False
    
    def writable(self):
        return True
    
    def seekable(self):
        return False
    
    def fileno(self):
        if hasattr(self.real_stdout, 'fileno'):
            return self.real_stdout.fileno()
        raise OSError("fileno not available")
    
    def __getattr__(self, name):
        # Delega tutti gli altri attributi al vero stdout
        return getattr(self.real_stdout, name)

# Salva il vero stdout originale e installa cattura
_real_stdout = sys.stdout
_print_capture = PrintCapture(_real_stdout)
sys.stdout = _print_capture


@dataclass
class ControllerConfig:
    robot_ip: str
    port: int = 30002


HTML_TEMPLATE = """
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>UR Remote Control</title>
    <!-- Material Design Icons -->
    <link href="https://fonts.googleapis.com/icon?family=Material+Icons" rel="stylesheet">
    <!-- Material Design CSS -->
    <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@300;400;500;700&display=swap" rel="stylesheet">
    <style>
      * {
        box-sizing: border-box;
        margin: 0;
        padding: 0;
      }
      
      :root {
        --mdc-theme-primary: #1976d2;
        --mdc-theme-primary-dark: #1565c0;
        --mdc-theme-secondary: #00c853;
        --mdc-theme-error: #d32f2f;
        --mdc-theme-warning: #f57c00;
        --mdc-theme-surface: #ffffff;
        --mdc-theme-background: #f5f5f5;
        --mdc-theme-on-primary: #ffffff;
        --mdc-theme-on-secondary: #ffffff;
        --mdc-theme-on-surface: #212121;
        --mdc-theme-on-error: #ffffff;
        --mdc-shape-small: 4px;
        --mdc-shape-medium: 8px;
        --mdc-shape-large: 16px;
        --mdc-elevation-1: 0px 2px 1px -1px rgba(0, 0, 0, 0.2), 0px 1px 1px 0px rgba(0, 0, 0, 0.14), 0px 1px 3px 0px rgba(0, 0, 0, 0.12);
        --mdc-elevation-2: 0px 3px 1px -2px rgba(0, 0, 0, 0.2), 0px 2px 2px 0px rgba(0, 0, 0, 0.14), 0px 1px 5px 0px rgba(0, 0, 0, 0.12);
        --mdc-elevation-4: 0px 2px 4px -1px rgba(0, 0, 0, 0.2), 0px 4px 5px 0px rgba(0, 0, 0, 0.14), 0px 1px 10px 0px rgba(0, 0, 0, 0.12);
      }
      
      body {
        font-family: 'Roboto', -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif;
        margin: 0;
        padding: 24px;
        background: var(--mdc-theme-background);
        color: var(--mdc-theme-on-surface);
        line-height: 1.6;
      }
      
      .container {
        max-width: 1400px;
        margin: 0 auto;
      }
      
      /* Header */
      .header {
        margin-bottom: 32px;
        padding-bottom: 16px;
        border-bottom: 2px solid rgba(0, 0, 0, 0.12);
      }
      
      .header h1 {
        font-size: 32px;
        font-weight: 500;
        color: var(--mdc-theme-primary);
        margin-bottom: 8px;
        display: flex;
        align-items: center;
        gap: 12px;
      }
      
      .header .subtitle {
        font-size: 16px;
        color: rgba(0, 0, 0, 0.6);
        font-weight: 400;
      }
      
      /* Material Design Card */
      .mdc-card {
        background: var(--mdc-theme-surface);
        border-radius: var(--mdc-shape-medium);
        box-shadow: var(--mdc-elevation-1);
        padding: 24px;
        margin-bottom: 24px;
        transition: box-shadow 0.2s ease;
      }
      
      .mdc-card:hover {
        box-shadow: var(--mdc-elevation-2);
      }
      
      .mdc-card__title {
        font-size: 20px;
        font-weight: 500;
        margin-bottom: 16px;
        color: var(--mdc-theme-primary);
        display: flex;
        align-items: center;
        gap: 8px;
      }
      
      /* Material Design Button */
      .mdc-button {
        display: inline-flex;
        align-items: center;
        justify-content: center;
        gap: 8px;
        padding: 10px 24px;
        font-size: 14px;
        font-weight: 500;
        text-transform: uppercase;
        letter-spacing: 0.0892857143em;
        border: none;
        border-radius: var(--mdc-shape-small);
        cursor: pointer;
        transition: all 0.2s ease;
        box-shadow: var(--mdc-elevation-2);
        min-width: 64px;
        height: 36px;
      }
      
      .mdc-button--raised {
        background-color: var(--mdc-theme-primary);
        color: var(--mdc-theme-on-primary);
      }
      
      .mdc-button--raised:hover {
        background-color: var(--mdc-theme-primary-dark);
        box-shadow: var(--mdc-elevation-4);
      }
      
      .mdc-button--raised:active {
        box-shadow: var(--mdc-elevation-1);
      }
      
      .mdc-button--outlined {
        background-color: transparent;
        color: var(--mdc-theme-primary);
        border: 1px solid var(--mdc-theme-primary);
        box-shadow: none;
      }
      
      .mdc-button--outlined:hover {
        background-color: rgba(25, 118, 210, 0.04);
      }
      
      .mdc-button--danger {
        background-color: var(--mdc-theme-error);
        color: var(--mdc-theme-on-error);
      }
      
      .mdc-button--danger:hover {
        background-color: #c62828;
      }
      
      .mdc-button:disabled {
        opacity: 0.38;
        cursor: not-allowed;
        box-shadow: none;
      }
      
      /* Material Design Progress Bar */
      .mdc-linear-progress {
        width: 100%;
        height: 4px;
        background-color: rgba(0, 0, 0, 0.12);
        border-radius: 2px;
        overflow: hidden;
        position: relative;
        margin: 16px 0;
      }
      
      .mdc-linear-progress__bar {
        height: 100%;
        background-color: var(--mdc-theme-primary);
        transform-origin: left;
        transition: transform 0.25s ease;
        position: relative;
      }
      
      .mdc-linear-progress__bar::after {
        content: '';
        position: absolute;
        top: 0;
        left: 0;
        bottom: 0;
        right: 0;
        background: linear-gradient(90deg, transparent, rgba(255,255,255,0.4), transparent);
        animation: shimmer 1.5s infinite;
      }
      
      .mdc-linear-progress--indeterminate .mdc-linear-progress__bar {
        width: 30%;
        animation: indeterminate 2s infinite linear;
      }
      
      @keyframes shimmer {
        0% { transform: translateX(-100%); }
        100% { transform: translateX(100%); }
      }
      
      @keyframes indeterminate {
        0% { transform: translateX(-100%) scaleX(0.3); }
        50% { transform: translateX(0%) scaleX(0.3); }
        100% { transform: translateX(100%) scaleX(0.3); }
      }
      
      /* Material Design Chip */
      .mdc-chip {
        display: inline-flex;
        align-items: center;
        padding: 6px 12px;
        border-radius: 16px;
        font-size: 13px;
        font-weight: 500;
        background-color: rgba(0, 0, 0, 0.08);
        color: var(--mdc-theme-on-surface);
      }
      
      .mdc-chip--success {
        background-color: rgba(0, 200, 83, 0.12);
        color: #2e7d32;
      }
      
      .mdc-chip--error {
        background-color: rgba(211, 47, 47, 0.12);
        color: #c62828;
      }
      
      .mdc-chip--warning {
        background-color: rgba(245, 124, 0, 0.12);
        color: #e65100;
      }
      
      /* Wizard Steps */
      .wizard-container {
        display: flex;
        gap: 12px;
        flex-wrap: wrap;
        margin-bottom: 24px;
      }
      
      .wizard-step {
        flex: 1;
        min-width: 180px;
        padding: 16px;
        border-radius: var(--mdc-shape-medium);
        background: var(--mdc-theme-surface);
        border: 2px solid rgba(0, 0, 0, 0.12);
        transition: all 0.3s ease;
        position: relative;
        opacity: 0.5;
      }
      
      .wizard-step.active {
        opacity: 1;
        border-color: var(--mdc-theme-primary);
        background: rgba(25, 118, 210, 0.04);
        box-shadow: var(--mdc-elevation-2);
      }
      
      .wizard-step.completed {
        opacity: 1;
        border-color: var(--mdc-theme-secondary);
        background: rgba(0, 200, 83, 0.04);
      }
      
      .wizard-step.error {
        opacity: 1;
        border-color: var(--mdc-theme-error);
        background: rgba(211, 47, 47, 0.04);
      }
      
      .wizard-step__header {
        display: flex;
        align-items: center;
        gap: 8px;
        margin-bottom: 12px;
      }
      
      .wizard-step__number {
        width: 32px;
        height: 32px;
        border-radius: 50%;
        background: var(--mdc-theme-primary);
        color: var(--mdc-theme-on-primary);
        display: flex;
        align-items: center;
        justify-content: center;
        font-weight: 500;
        font-size: 16px;
      }
      
      .wizard-step__title {
        flex: 1;
        font-size: 14px;
        font-weight: 500;
      }
      
      .wizard-step__status {
        width: 20px;
        height: 20px;
        border-radius: 50%;
        display: flex;
        align-items: center;
        justify-content: center;
      }
      
      .wizard-step__status.waiting {
        background: rgba(0, 0, 0, 0.12);
      }
      
      .wizard-step__status.success {
        background: var(--mdc-theme-secondary);
        color: var(--mdc-theme-on-secondary);
      }
      
      .wizard-step__status.error {
        background: var(--mdc-theme-error);
        color: var(--mdc-theme-on-error);
      }
      
      .wizard-step__message {
        margin-top: 8px;
        padding: 8px;
        background: rgba(0, 0, 0, 0.04);
        border-radius: var(--mdc-shape-small);
        font-size: 12px;
        color: rgba(0, 0, 0, 0.6);
        min-height: 30px;
      }
      
      /* Progress Indicator */
      .progress-indicator {
        display: flex;
        align-items: center;
        gap: 12px;
        margin: 16px 0;
      }
      
      .progress-indicator__label {
        font-size: 14px;
        color: rgba(0, 0, 0, 0.6);
        min-width: 120px;
      }
      
      .progress-indicator__bar {
        flex: 1;
        height: 8px;
        background: rgba(0, 0, 0, 0.12);
        border-radius: 4px;
        overflow: hidden;
        position: relative;
      }
      
      .progress-indicator__fill {
        height: 100%;
        background: linear-gradient(90deg, var(--mdc-theme-primary), var(--mdc-theme-secondary));
        border-radius: 4px;
        transition: width 0.3s ease;
        position: relative;
      }
      
      .progress-indicator__fill::after {
        content: '';
        position: absolute;
        top: 0;
        left: 0;
        bottom: 0;
        right: 0;
        background: linear-gradient(90deg, transparent, rgba(255,255,255,0.4), transparent);
        animation: shimmer 1.5s infinite;
      }
      
      .progress-indicator__value {
        font-size: 14px;
        font-weight: 500;
        color: var(--mdc-theme-primary);
        min-width: 50px;
        text-align: right;
      }
      
      /* Toast Notifications */
      .toast-container {
        position: fixed;
        top: 24px;
        right: 24px;
        z-index: 10000;
        display: flex;
        flex-direction: column;
        gap: 12px;
        max-width: 400px;
      }
      
      .toast {
        padding: 16px 20px;
        border-radius: var(--mdc-shape-medium);
        box-shadow: var(--mdc-elevation-4);
        display: flex;
        align-items: center;
        gap: 12px;
        font-weight: 500;
        min-width: 300px;
        animation: slideInRight 0.3s ease-out;
        background: var(--mdc-theme-surface);
        color: var(--mdc-theme-on-surface);
      }
      
      .toast.success {
        border-left: 4px solid var(--mdc-theme-secondary);
      }
      
      .toast.error {
        border-left: 4px solid var(--mdc-theme-error);
      }
      
      .toast.warning {
        border-left: 4px solid var(--mdc-theme-warning);
      }
      
      .toast.info {
        border-left: 4px solid var(--mdc-theme-primary);
      }
      
      .toast.fade-out {
        animation: slideOutRight 0.3s ease-out;
        opacity: 0;
      }
      
      @keyframes slideInRight {
        from {
          opacity: 0;
          transform: translateX(100%);
        }
        to {
          opacity: 1;
          transform: translateX(0);
        }
      }
      
      @keyframes slideOutRight {
        from {
          opacity: 1;
          transform: translateX(0);
        }
        to {
          opacity: 0;
          transform: translateX(100%);
        }
      }
      
      /* Joystick Panel */
      .joystick-panel {
        border-radius: var(--mdc-shape-medium);
        padding: 24px;
        background: var(--mdc-theme-surface);
        box-shadow: var(--mdc-elevation-1);
        display: flex;
        flex-direction: column;
        gap: 16px;
        align-items: center;
        justify-content: center;
      }
      
      #joystick, #joystick2 {
        position: relative;
        width: 220px;
        height: 220px;
        border-radius: 50%;
        background: radial-gradient(circle at center, #f5f8fd 0%, #d9e4f7 70%);
        border: 2px solid rgba(25, 118, 210, 0.25);
        box-shadow: inset 0 4px 12px rgba(0, 0, 0, 0.1);
        touch-action: none;
        user-select: none;
      }
      
      #joystick-base, #joystick2-base {
        position: absolute;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        width: 110px;
        height: 110px;
        border-radius: 50%;
        background: rgba(25, 118, 210, 0.12);
        border: 1px solid rgba(25, 118, 210, 0.2);
      }
      
      #joystick-handle, #joystick2-handle {
        position: absolute;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        width: 84px;
        height: 84px;
        border-radius: 50%;
        background: radial-gradient(circle at 30% 30%, #ffffff 0%, #7aa8d6 85%);
        border: 1px solid rgba(25, 118, 210, 0.35);
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.25);
        cursor: grab;
      }
      
      #joystick-handle:active {
        cursor: grabbing;
      }
      
      .joystick-readout {
        font-family: "Courier New", monospace;
        font-size: 16px;
        color: var(--mdc-theme-on-surface);
        text-align: center;
        font-weight: 500;
        background: rgba(0, 0, 0, 0.04);
        padding: 8px 12px;
        border-radius: var(--mdc-shape-small);
      }
      
      /* Layout */
      .layout {
        display: grid;
        grid-template-columns: 1fr;
        gap: 24px;
      }
      
      @media (min-width: 768px) {
        .layout {
          grid-template-columns: repeat(2, 1fr);
        }
      }
      
      /* Status Bar */
      .status-bar {
        margin-top: 24px;
        padding: 16px 20px;
        border-radius: var(--mdc-shape-medium);
        background: var(--mdc-theme-surface);
        box-shadow: var(--mdc-elevation-1);
        font-weight: 500;
        font-size: 16px;
      }
      
      .status-bar .status-ready {
        color: var(--mdc-theme-secondary);
        font-weight: 500;
      }
      
      .status-bar .status-error {
        color: var(--mdc-theme-error);
        font-weight: 500;
      }
      
      /* Material Icons */
      .material-icons {
        font-family: 'Material Icons';
        font-weight: normal;
        font-style: normal;
        font-size: 24px;
        line-height: 1;
        letter-spacing: normal;
        text-transform: none;
        display: inline-block;
        white-space: nowrap;
        word-wrap: normal;
        direction: ltr;
        -webkit-font-feature-settings: 'liga';
        -webkit-font-smoothing: antialiased;
      }
      
      .material-icons.md-18 { font-size: 18px; }
      .material-icons.md-24 { font-size: 24px; }
      .material-icons.md-36 { font-size: 36px; }
      .material-icons.md-48 { font-size: 48px; }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="header">
        <h1>
          UR5e Robot Control
        </h1>
        <p class="subtitle">Follow the setup steps below to configure the robot. When ready, use the joysticks to control it.</p>
      </div>

      <!-- Wizard Setup -->
      <div class="mdc-card">
        <div class="mdc-card__title">
          <span class="material-icons">settings</span>
          Setup Wizard
        </div>
        
        <div class="wizard-container">
          <!-- Step A -->
          <div class="wizard-step" id="step-a">
            <div class="wizard-step__header">
              <div class="wizard-step__number">A</div>
              <div class="wizard-step__title">Start Driver</div>
              <div class="wizard-step__status waiting" id="step-a-status">
                <span class="material-icons md-18">hourglass_empty</span>
              </div>
            </div>
            <button class="mdc-button mdc-button--raised" id="wizard-start-driver" style="width: 100%;">
              <span class="material-icons md-18">play_arrow</span>
              Start
            </button>
            <button class="mdc-button mdc-button--outlined" id="wizard-retry-a" style="width: 100%; margin-top: 8px; display: none;">
              <span class="material-icons md-18">refresh</span>
              Retry
            </button>
            <div class="wizard-step__message" id="step-a-message"></div>
            <div class="mdc-linear-progress mdc-linear-progress--indeterminate" id="step-a-progress" style="display: none; margin-top: 12px;">
              <div class="mdc-linear-progress__buffer"></div>
              <div class="mdc-linear-progress__bar mdc-linear-progress__primary-bar">
                <span class="mdc-linear-progress__bar-inner"></span>
              </div>
              <div class="mdc-linear-progress__bar mdc-linear-progress__secondary-bar">
                <span class="mdc-linear-progress__bar-inner"></span>
              </div>
            </div>
          </div>

          <!-- Step B -->
          <div class="wizard-step" id="step-b">
            <div class="wizard-step__header">
              <div class="wizard-step__number">B</div>
              <div class="wizard-step__title">Verify Driver</div>
              <div class="wizard-step__status waiting" id="step-b-status">
                <span class="material-icons md-18">pause</span>
              </div>
            </div>
            <button class="mdc-button mdc-button--outlined" id="wizard-retry-b" style="width: 100%; display: none;">
              <span class="material-icons md-18">refresh</span>
              Retry
            </button>
            <div class="wizard-step__message" id="step-b-message"></div>
          </div>

          <!-- Step C -->
          <div class="wizard-step" id="step-c">
            <div class="wizard-step__header">
              <div class="wizard-step__number">C</div>
              <div class="wizard-step__title">Activate Controller</div>
              <div class="wizard-step__status waiting" id="step-c-status">
                <span class="material-icons md-18">pause</span>
              </div>
            </div>
            <button class="mdc-button mdc-button--raised" id="wizard-activate-controller" style="width: 100%;">
              <span class="material-icons md-18">play_arrow</span>
              Activate Controller
            </button>
            <button class="mdc-button mdc-button--outlined" id="wizard-retry-c" style="width: 100%; margin-top: 8px; display: none;">
              <span class="material-icons md-18">refresh</span>
              Retry
            </button>
            <div class="wizard-step__message" id="step-c-message"></div>
          </div>

          <!-- Step D -->
          <div class="wizard-step" id="step-d">
            <div class="wizard-step__header">
              <div class="wizard-step__number">D</div>
              <div class="wizard-step__title">Teach Pendant</div>
              <div class="wizard-step__status waiting" id="step-d-status">
                <span class="material-icons md-18">pause</span>
              </div>
            </div>
            <details style="margin: 8px 0;">
              <summary style="cursor: pointer; font-size: 12px; color: rgba(0, 0, 0, 0.6); font-weight: 500;">
                <span class="material-icons md-18" style="vertical-align: middle;">info</span>
                Instructions
              </summary>
              <div style="margin-top: 8px; padding: 12px; background: rgba(0, 0, 0, 0.04); border-radius: var(--mdc-shape-small); font-size: 12px;">
                <ol style="margin: 0; padding-left: 20px;">
                  <li>Go to <strong>Program</strong></li>
                  <li>Open program with <strong>External Control</strong></li>
                  <li>IP: <strong>192.168.10.191</strong>, Port: <strong>50002</strong></li>
                  <li><strong>IMPORTANT:</strong> Enable <strong>Remote Control</strong> on Teach Pendant</li>
                  <li><strong>SAVE</strong> and press <strong>PLAY</strong></li>
                </ol>
              </div>
            </details>
            <button class="mdc-button mdc-button--raised" id="wizard-check-teach-pendant" style="width: 100%; display: none;">
              <span class="material-icons md-18">check_circle</span>
              Done
            </button>
            <button class="mdc-button mdc-button--outlined" id="wizard-retry-d" style="width: 100%; margin-top: 8px; display: none;">
              <span class="material-icons md-18">refresh</span>
              Retry
            </button>
            <div class="wizard-step__message" id="step-d-message"></div>
          </div>

          <!-- Step E -->
          <div class="wizard-step" id="step-e">
            <div class="wizard-step__header">
              <div class="wizard-step__number">E</div>
              <div class="wizard-step__title">Verify Connection</div>
              <div class="wizard-step__status waiting" id="step-e-status">
                <span class="material-icons md-18">pause</span>
              </div>
            </div>
            <button class="mdc-button mdc-button--outlined" id="wizard-retry-e" style="width: 100%; display: none;">
              <span class="material-icons md-18">refresh</span>
              Retry
            </button>
            <div class="wizard-step__message" id="step-e-message"></div>
          </div>
        </div>
      </div>

      <!-- Joystick Section -->
      <section id="joystick-section" class="mdc-card" style="display: none; border-left: 4px solid var(--mdc-theme-secondary);">
        <div class="mdc-card__title">
          <span class="material-icons">sports_esports</span>
          Robot Control - Ready
        </div>
        <p style="margin-bottom: 24px; color: rgba(0, 0, 0, 0.6);">
          Robot connected and ready. Use the joysticks below to move the robot. The robot will only move when you move the joysticks.
        </p>

        <div class="layout">
          <section class="joystick-panel">
            <h3 style="margin-bottom: 16px; font-weight: 500;">Joystick XY</h3>
            <div id="joystick">
              <div id="joystick-base"></div>
              <div id="joystick-handle"></div>
            </div>
            <div class="joystick-readout">
              X: <span id="joy-x">0.00</span> &nbsp;
              Y: <span id="joy-y">0.00</span>
            </div>
          </section>

          <section class="joystick-panel">
            <h3 style="margin-bottom: 16px; font-weight: 500;">Joystick Z / Rotation</h3>
            <div id="joystick2">
              <div id="joystick2-base"></div>
              <div id="joystick2-handle"></div>
            </div>
            <div class="joystick-readout">
              Z: <span id="joy2-x">0.00</span> &nbsp;
              Rz: <span id="joy2-y">0.00</span>
            </div>
          </section>

          <div style="grid-column: 1 / -1;">
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 24px; margin-bottom: 24px;">
              <div>
                <label style="display: block; margin-bottom: 8px; font-weight: 500;">
                  Speed: <input type="number" id="speed-input" value="100" min="1" step="1" style="width: 80px; padding: 6px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: var(--mdc-shape-small); font-size: 14px; margin-left: 8px;">%
                </label>
                <input type="range" id="speed-slider" min="1" max="1000" value="100" step="1" style="width: 100%;">
                <div style="display: flex; justify-content: space-between; font-size: 12px; color: rgba(0, 0, 0, 0.6); margin-top: 4px;">
                  <span>Slow (1%)</span>
                  <span>Fast (1000%)</span>
                </div>
              </div>
              <div>
                <label style="display: block; margin-bottom: 8px; font-weight: 500;">
                  Frequency: <input type="number" id="frequency-input" value="125" min="10" max="500" step="1" style="width: 80px; padding: 6px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: var(--mdc-shape-small); font-size: 14px; margin-left: 8px;"> Hz
                </label>
                <input type="range" id="frequency-slider" min="10" max="500" value="125" step="5" style="width: 100%;">
                <div style="display: flex; justify-content: space-between; font-size: 12px; color: rgba(0, 0, 0, 0.6); margin-top: 4px;">
                  <span>Slow (10Hz)</span>
                  <span>Fast (500Hz)</span>
                </div>
              </div>
            </div>
            
            <div class="progress-indicator">
              <div class="progress-indicator__label">Movement Fluidity</div>
              <div class="progress-indicator__bar">
                <div class="progress-indicator__fill" id="fluidity-bar" style="width: 0%;"></div>
              </div>
              <div class="progress-indicator__value" id="fluidity-value">—</div>
            </div>
            
            <div style="display: flex; gap: 12px; margin-top: 16px; flex-wrap: wrap;">
              <button class="mdc-button mdc-button--outlined mdc-button--danger" id="stop-joystick">
                <span class="material-icons md-18">stop</span>
                Emergency Stop
              </button>
              <button class="mdc-button mdc-button--outlined" id="check-processes">
                <span class="material-icons md-18">search</span>
                Check Processes
              </button>
              <button class="mdc-button mdc-button--outlined mdc-button--danger" id="restart-web-interface">
                <span class="material-icons md-18">refresh</span>
                Restart Interface
              </button>
            </div>
            <div id="process-status" style="margin-top: 12px; padding: 12px; background: rgba(0, 0, 0, 0.04); border-radius: var(--mdc-shape-small); font-size: 14px; display: none;"></div>
          </div>
        </div>

        <!-- Vision & MoveIt Section -->
        <section id="vision-moveit-section" class="mdc-card" style="margin-top: 24px; border-left: 4px solid #9c27b0; display: block;">
          <div class="mdc-card__title">
            <span class="material-icons">camera_alt</span>
            Vision System & MoveIt
          </div>
          
          <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 24px;">
            <!-- Orbbec Camera Status -->
            <div>
              <h3 style="font-size: 16px; font-weight: 500; margin-bottom: 12px; display: flex; align-items: center; gap: 8px;">
                <span class="material-icons md-18">videocam</span>
                Orbbec Camera
              </h3>
              <div id="orbbec-status" style="padding: 12px; background: rgba(0, 0, 0, 0.04); border-radius: var(--mdc-shape-small); margin-bottom: 12px;">
                <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                  <span>Status:</span>
                  <span id="orbbec-status-value" class="mdc-chip">Checking...</span>
                </div>
                <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                  <span>Topics:</span>
                  <span id="orbbec-topics-count">—</span>
                </div>
                <div style="display: flex; justify-content: space-between;">
                  <span>FPS:</span>
                  <span id="orbbec-fps">—</span>
                </div>
              </div>
              <div style="display: flex; gap: 8px; margin-bottom: 12px;">
                <button class="mdc-button mdc-button--outlined" id="start-orbbec" style="flex: 1;">
                  <span class="material-icons md-18">play_arrow</span>
                  Start Camera
                </button>
                <button class="mdc-button mdc-button--outlined" id="stop-orbbec" style="flex: 1;">
                  <span class="material-icons md-18">stop</span>
                  Stop Camera
                </button>
              </div>
              <!-- Camera Video Stream -->
              <div style="margin-top: 12px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: var(--mdc-shape-small); overflow: hidden; background: #000;">
                <img id="camera-stream" src="/api/vision/camera_stream" style="width: 100%; max-height: 300px; object-fit: contain; display: block;" alt="Camera stream not available">
                <div id="camera-stream-status" style="padding: 8px; background: rgba(0, 0, 0, 0.8); color: #fff; font-size: 12px; text-align: center; display: none;">
                  Camera stream loading...
                </div>
              </div>
            </div>
            
            <!-- MoveIt Status -->
            <div>
              <h3 style="font-size: 16px; font-weight: 500; margin-bottom: 12px; display: flex; align-items: center; gap: 8px;">
                <span class="material-icons md-18">route</span>
                MoveIt2 Motion Planning
              </h3>
              <div id="moveit-status" style="padding: 12px; background: rgba(0, 0, 0, 0.04); border-radius: var(--mdc-shape-small); margin-bottom: 12px;">
                <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                  <span>Status:</span>
                  <span id="moveit-status-value" class="mdc-chip">Checking...</span>
                </div>
                <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                  <span>Planning Group:</span>
                  <span id="moveit-planning-group">ur_manipulator</span>
                </div>
                <div style="display: flex; justify-content: space-between;">
                  <span>Last Plan:</span>
                  <span id="moveit-last-plan">—</span>
                </div>
              </div>
              <div style="display: flex; gap: 8px; margin-bottom: 12px;">
                <button class="mdc-button mdc-button--outlined" id="start-moveit" style="flex: 1;">
                  <span class="material-icons md-18">play_arrow</span>
                  Start MoveIt
                </button>
                <button class="mdc-button mdc-button--outlined" id="stop-moveit" style="flex: 1;">
                  <span class="material-icons md-18">stop</span>
                  Stop MoveIt
                </button>
              </div>
              <div style="display: flex; gap: 8px; margin-bottom: 12px;">
                <button class="mdc-button mdc-button--outlined" id="test-moveit" style="flex: 1;">
                  <span class="material-icons md-18">check_circle</span>
                  Test MoveIt
                </button>
                <button class="mdc-button mdc-button--outlined" id="plan-move" style="flex: 1;">
                  <span class="material-icons md-18">navigation</span>
                  Plan Move
                </button>
              </div>
              <!-- MoveIt Planning Interface -->
              <div style="margin-top: 12px; padding: 12px; background: rgba(0, 0, 0, 0.04); border-radius: var(--mdc-shape-small);">
                <h4 style="font-size: 14px; font-weight: 500; margin-bottom: 8px;">Target Pose (XYZ + RPY):</h4>
                <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; margin-bottom: 8px;">
                  <div>
                    <label style="font-size: 12px; display: block; margin-bottom: 4px;">X (m):</label>
                    <input type="number" id="moveit-x" step="0.01" value="0.3" style="width: 100%; padding: 4px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: 4px;">
                  </div>
                  <div>
                    <label style="font-size: 12px; display: block; margin-bottom: 4px;">Y (m):</label>
                    <input type="number" id="moveit-y" step="0.01" value="0.0" style="width: 100%; padding: 4px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: 4px;">
                  </div>
                  <div>
                    <label style="font-size: 12px; display: block; margin-bottom: 4px;">Z (m):</label>
                    <input type="number" id="moveit-z" step="0.01" value="0.3" style="width: 100%; padding: 4px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: 4px;">
                  </div>
                  <div>
                    <label style="font-size: 12px; display: block; margin-bottom: 4px;">Roll (rad):</label>
                    <input type="number" id="moveit-roll" step="0.01" value="0.0" style="width: 100%; padding: 4px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: 4px;">
                  </div>
                  <div>
                    <label style="font-size: 12px; display: block; margin-bottom: 4px;">Pitch (rad):</label>
                    <input type="number" id="moveit-pitch" step="0.01" value="0.0" style="width: 100%; padding: 4px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: 4px;">
                  </div>
                  <div>
                    <label style="font-size: 12px; display: block; margin-bottom: 4px;">Yaw (rad):</label>
                    <input type="number" id="moveit-yaw" step="0.01" value="0.0" style="width: 100%; padding: 4px; border: 1px solid rgba(0, 0, 0, 0.12); border-radius: 4px;">
                  </div>
                </div>
                <button class="mdc-button mdc-button--raised" id="execute-moveit-plan" style="width: 100%; margin-top: 8px;">
                  <span class="material-icons md-18">play_arrow</span>
                  Plan & Execute
                </button>
                <div id="moveit-plan-status" style="margin-top: 8px; padding: 8px; background: rgba(0, 0, 0, 0.04); border-radius: 4px; font-size: 12px; display: none;"></div>
              </div>
            </div>
          </div>
          
          <!-- Vision Detections -->
          <div style="margin-top: 24px; border-top: 1px solid rgba(0, 0, 0, 0.12); padding-top: 16px;">
            <h3 style="font-size: 16px; font-weight: 500; margin-bottom: 12px; display: flex; align-items: center; gap: 8px;">
              <span class="material-icons md-18">visibility</span>
              Object Detections
            </h3>
            <div id="detections-container" style="max-height: 200px; overflow-y: auto; padding: 12px; background: rgba(0, 0, 0, 0.04); border-radius: var(--mdc-shape-small);">
              <div style="color: rgba(0, 0, 0, 0.5); font-style: italic;">No detections yet. Start camera and vision system.</div>
            </div>
            <div style="display: flex; gap: 8px; margin-top: 12px;">
              <button class="mdc-button mdc-button--outlined" id="start-yolo" style="flex: 1;">
                <span class="material-icons md-18">play_arrow</span>
                Start YOLO Detector
              </button>
              <button class="mdc-button mdc-button--outlined" id="stop-yolo" style="flex: 1;">
                <span class="material-icons md-18">stop</span>
                Stop YOLO Detector
              </button>
            </div>
            <div style="display: flex; gap: 8px; margin-top: 8px;">
              <button class="mdc-button mdc-button--outlined" id="start-vision" style="flex: 1;">
                <span class="material-icons md-18">play_arrow</span>
                Start Vision System
              </button>
              <button class="mdc-button mdc-button--outlined" id="stop-vision" style="flex: 1;">
                <span class="material-icons md-18">stop</span>
                Stop Vision System
              </button>
            </div>
            <div style="margin-top: 12px;">
              <button class="mdc-button mdc-button--raised" id="approach-object" style="width: 100%;">
                <span class="material-icons md-18">near_me</span>
                Avvicinati al Pezzo (20cm)
              </button>
              <div id="approach-status" style="margin-top: 8px; padding: 8px; background: rgba(0, 0, 0, 0.04); border-radius: 4px; font-size: 12px; display: none;"></div>
            </div>
          </div>
        </section>

        <!-- Logging Section -->
        <div style="margin-top: 24px; border-top: 1px solid rgba(0, 0, 0, 0.12); padding-top: 16px;">
          <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px;">
            <h3 style="font-size: 18px; font-weight: 500; display: flex; align-items: center; gap: 8px;">
              <span class="material-icons">description</span>
              Logs & Errors
            </h3>
            <div>
              <button class="mdc-button mdc-button--outlined" id="clear-logs" style="padding: 6px 12px; height: 32px;">
                <span class="material-icons md-18">delete</span>
                Clear
              </button>
              <button class="mdc-button mdc-button--outlined" id="toggle-logs" style="padding: 6px 12px; height: 32px; margin-left: 8px;">
                <span class="material-icons md-18">pause</span>
                Pause
              </button>
            </div>
          </div>
          <div id="logs-container" style="background: #1e1e1e; color: #d4d4d4; font-family: 'Courier New', monospace; font-size: 12px; padding: 12px; border-radius: var(--mdc-shape-small); max-height: 400px; overflow-y: auto; min-height: 200px;">
            <div style="color: rgba(255, 255, 255, 0.5); font-style: italic;">Waiting for logs...</div>
          </div>
          <div style="margin-top: 8px; font-size: 12px; color: rgba(0, 0, 0, 0.6);">
            <span id="log-count">0</span> total logs | 
            <span id="log-errors">0</span> errors | 
            <span id="log-warnings">0</span> warnings
          </div>
        </div>
      </section>

      <div class="status-bar">
        Status: <span class="status-ready" id="status-text">Ready</span>
      </div>
    </div>

    <!-- Toast Container -->
    <div id="toast-container" class="toast-container"></div>

    <script>
      // GLOBAL ERROR HANDLER - Intercetta TUTTI gli errori JavaScript
      window.addEventListener('error', function(event) {
        console.error('[GLOBAL ERROR]', event.error);
        console.error('[GLOBAL ERROR] Message:', event.message);
        console.error('[GLOBAL ERROR] Source:', event.filename, 'Line:', event.lineno, 'Col:', event.colno);
        console.error('[GLOBAL ERROR] Stack:', event.error ? event.error.stack : 'N/A');
        
        // Mostra errore anche all'utente
        const errorMsg = 'Errore JavaScript: ' + (event.message || 'Unknown error') + ' (Linea ' + event.lineno + ')';
        if (typeof showToast === 'function') {
          showToast(errorMsg, 'error', 10000);
        } else {
          alert(errorMsg);
        }
      });
      
      // Intercetta anche errori non catturati nelle Promise
      window.addEventListener('unhandledrejection', function(event) {
        console.error('[UNHANDLED PROMISE REJECTION]', event.reason);
        const errorMsg = 'Errore Promise: ' + (event.reason ? event.reason.toString() : 'Unknown');
        if (typeof showToast === 'function') {
          showToast(errorMsg, 'error', 10000);
        }
      });
      
      // Toast Notification System (NO EMOJI)
      function showToast(message, type = 'info', duration = 3000) {
        const container = document.getElementById('toast-container');
        if (!container) return;
        
        const toast = document.createElement('div');
        toast.className = 'toast ' + type;
        
        const icon = {
          success: 'check_circle',
          error: 'error',
          warning: 'warning',
          info: 'info'
        }[type] || 'info';
        
        toast.innerHTML = '<span class="material-icons">' + icon + '</span>' +
          '<span style="flex: 1;">' + message + '</span>' +
          '<button onclick="this.parentElement.remove()" style="background: none; border: none; cursor: pointer; opacity: 0.6; padding: 0; width: 24px; height: 24px; display: flex; align-items: center; justify-content: center;">' +
            '<span class="material-icons md-18">close</span>' +
          '</button>';
        
        container.appendChild(toast);
        
        setTimeout(() => {
          toast.classList.add('fade-out');
          setTimeout(() => toast.remove(), 300);
        }, duration);
      }
      
      // Status management
      const statusBar = document.getElementById('status-text');
      function setStatus(text, ok = true) {
        if (statusBar) {
          statusBar.textContent = text;
          statusBar.className = ok ? 'status-ready' : 'status-error';
        }
        showToast(text, ok ? 'success' : 'error', ok ? 2000 : 4000);
      }

      const status = document.getElementById("status");
      const form = document.getElementById("move-form");
      const stepInput = document.getElementById("step-size");
      const jointInputs = form ? Array.from(form.querySelectorAll("input[name^='joint']")) : [];
      const rosBridgeState = document.getElementById("ros-bridge-state");
      const rosLoopState = document.getElementById("ros-loop-state");
      const rosLastCommand = document.getElementById("ros-last-command");
      const rosLastPublish = document.getElementById("ros-last-publish");
      const rosEnv = document.getElementById("ros-env");
      const rosTopicList = document.getElementById("ros-topic-list");
      const rosStatusJson = document.getElementById("ros-status-json");
      const rosWarning = document.getElementById("ros-warning");
      const rosInfo = document.getElementById("ros-info");
      const statusTimestamp = document.getElementById("status-timestamp");
      const refreshStatusBtn = document.getElementById("refresh-status");
      
      // Safety check: se gli elementi del monitor non esistono, non fare nulla
      const hasMonitor = rosBridgeState && rosLoopState && rosLastCommand && 
                         rosLastPublish && rosEnv && rosTopicList && 
                         rosStatusJson && rosWarning && rosInfo && statusTimestamp && refreshStatusBtn;

      if (form) {
        form.addEventListener("submit", async (event) => {
          event.preventDefault();
          const formData = new FormData(form);
        const payload = {};

        for (let [key, value] of formData.entries()) {
          if (key.startsWith("joint")) {
            payload[key] = parseFloat(value);
          }
        }

        payload.acceleration = parseFloat(formData.get("acceleration"));
        payload.velocity = parseFloat(formData.get("velocity"));
        payload.blend_radius = parseFloat(formData.get("blend_radius"));
        payload.async_move = formData.get("async_move") === "on";
        payload.step = parseFloat(stepInput.value);

        setStatus("Sending MoveJ command…");
        try {
          const response = await fetch("/api/movej", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
          });
          const data = await response.json();
          setStatus(data.message, true);
        } catch (err) {
          console.error(err);
          setStatus("Error: " + err, false);
        }
        });
      }

      async function sendStop() {
        setStatus("Sending stop…");
        try {
          const response = await fetch("/api/stop", { method: "POST" });
          const data = await response.json();
          setStatus(data.message, true);
        } catch (err) {
          console.error(err);
          setStatus("Error: " + err, false);
        }
      }

      function setStatus(text, ok = true) {
        if (status) {
          status.innerHTML = 'Stato: <span class="' + (ok ? 'ready' : 'error') + '">' + text + '</span>';
        }
        if (statusBar) {
          statusBar.textContent = text;
          statusBar.className = ok ? 'status-ready' : 'status-error';
        }
        showToast(text, ok ? 'success' : 'error', ok ? 2000 : 4000);
      }

      function describeAge(ageSeconds, isoString) {
        if (ageSeconds == null) return "—";
        const rounded = ageSeconds > 60 ? (ageSeconds / 60).toFixed(1) + ' min' : ageSeconds.toFixed(2) + ' s';
        return isoString ? rounded + ' fa (' + isoString + ')' : rounded + ' fa';
      }

      function renderTopics(publishers) {
        if (!publishers || Object.keys(publishers).length === 0) {
          return "<div class='topic-row error'><span>Nessun publisher</span><span>offline</span></div>";
        }
        return Object.entries(publishers).map(([name, ok]) => {
          const cls = ok ? "ok" : "error";
          const label = ok ? "online" : "missing";
          return '<div class="topic-row ' + cls + '"><span>' + name + '</span><span>' + label + '</span></div>';
        }).join("");
      }

      function renderRosStatus(payload) {
        if (!hasMonitor) {
          console.warn("Monitor ROS2 elements not found in DOM");
          return;
        }
        
        const bridge = payload && payload.ros2_bridge ? payload.ros2_bridge : null;
        const rosReady = payload && payload.ros2_available && bridge && bridge.ros_initialized;
        if (rosBridgeState) {
          rosBridgeState.textContent = rosReady ? "ROS2 pronto" : "ROS2 non pronto";
          rosBridgeState.className = 'monitor-value badge ' + (rosReady ? 'badge-ok' : 'badge-error');
        }

        const loopRunning = bridge && bridge.publish_loop_running;
        const publishRate = bridge && typeof bridge.publish_rate_hz === "number"
          ? bridge.publish_rate_hz.toFixed(0)
          : "0";
        if (rosLoopState) {
          rosLoopState.textContent = loopRunning ? publishRate + ' Hz' : 'fermo';
          rosLoopState.className = 'monitor-value badge ' + (loopRunning ? 'badge-ok' : 'badge-error');
        }

        if (rosLastCommand) {
          rosLastCommand.textContent = describeAge(
            bridge ? bridge.last_command_age_s : null,
            bridge ? bridge.last_command_time : null
          );
        }
        if (rosLastPublish) {
          rosLastPublish.textContent = describeAge(
            bridge ? bridge.last_publish_age_s : null,
            bridge ? bridge.last_publish_time : null
          );
        }

        const env = (bridge && bridge.env) || (payload && payload.env) || {};
        if (rosEnv) {
          rosEnv.textContent = [
            env.ROS_DISTRO ? 'ROS ' + env.ROS_DISTRO : 'ROS? n/d',
            env.LD_LIBRARY_PATH ? 'LD_LIB ✓' : 'LD_LIB ✗',
            env.PYTHONPATH ? 'PYTHONPATH ✓' : 'PYTHONPATH ✗',
            env.HOSTNAME ? 'Host: ' + env.HOSTNAME : null,
          ].filter(Boolean).join(" · ") || "n/d";
        }

        if (rosTopicList) {
          rosTopicList.innerHTML = renderTopics(bridge ? bridge.publishers : null);
        }
        if (rosWarning) {
          if (bridge && bridge.last_error) {
            rosWarning.innerHTML = '<span class=' + '"material-icons md-18"' + '>warning</span> Errore: ' + bridge.last_error;
            rosWarning.style.display = "block";
          } else {
            rosWarning.textContent = "";
            rosWarning.style.display = "none";
          }
        }
        
        // Mostra info utili per debug
        if (rosInfo) {
          const infoMessages = [];
          if (bridge) {
            const speeds = bridge.target_speeds || [0,0,0,0,0,0];
            const hasNonZeroSpeed = speeds.some(s => Math.abs(s) > 0.001);
            if (hasNonZeroSpeed) {
                infoMessages.push('<span class=' + '"material-icons md-18"' + '>sports_esports</span> Velocità target: [' + speeds.map(s => s.toFixed(3)).join(', ') + ']');
            }
            if (bridge.publish_loop_running && bridge.last_publish_age_s !== null) {
              const age = bridge.last_publish_age_s;
              if (age > 0.1) {
                infoMessages.push('<span class=' + '"material-icons md-18"' + '>warning</span> Ultimo publish ' + age.toFixed(2) + 's fa - potrebbe essere un problema');
              } else {
                infoMessages.push('<span class=' + '"material-icons md-18"' + '>check_circle</span> Pubblicazione attiva (' + bridge.publish_rate_hz + 'Hz)');
              }
            }
            if (!bridge.ros_initialized) {
              infoMessages.push('<span class=' + '"material-icons md-18"' + '>error</span> ROS2 non inizializzato - controlla la configurazione');
            }
            if (!bridge.publish_loop_running) {
              infoMessages.push('<span class=' + '"material-icons md-18"' + '>error</span> Loop di pubblicazione fermo - riavvia il servizio');
            }
          }
          rosInfo.textContent = infoMessages.length > 0 ? infoMessages.join(" | ") : "";
          rosInfo.style.display = infoMessages.length > 0 ? "block" : "none";
        }

        if (rosStatusJson) {
          rosStatusJson.textContent = JSON.stringify(payload, null, 2);
        }
        if (statusTimestamp) {
          statusTimestamp.textContent = 'Agg. ' + new Date().toLocaleTimeString();
        }
      }

      async function fetchRosStatus(showToast = false) {
        try {
          const response = await fetch("/api/status");
          const payload = await response.json();
          if (payload.status === "ok") {
            renderRosStatus(payload.data);
            if (showToast) {
              setStatus("Stato ROS2 aggiornato", true);
            }
          } else {
            setStatus("Status API error", false);
          }
        } catch (err) {
          console.error("Status fetch error", err);
          setStatus("Impossibile leggere lo stato ROS2", false);
        }
      }

      if (refreshStatusBtn) {
        refreshStatusBtn.addEventListener("click", () => fetchRosStatus(true));
      }
      if (hasMonitor) {
        fetchRosStatus();
        setInterval(fetchRosStatus, 3000);
      }

      // --- Robot Status Monitor -------------------------------------------------
      const robotMode = document.getElementById("robot-mode");
      const safetyMode = document.getElementById("safety-mode");
      const programState = document.getElementById("program-state");
      const remoteControl = document.getElementById("remote-control");
      const jointPositions = document.getElementById("joint-positions");
      const tcpPose = document.getElementById("tcp-pose");
      const robotStatusWarning = document.getElementById("robot-status-warning");
      const robotStatusInfo = document.getElementById("robot-status-info");
      const robotStatusJson = document.getElementById("robot-status-json");
      const robotStatusTimestamp = document.getElementById("robot-status-timestamp");
      const refreshRobotStatusBtn = document.getElementById("refresh-robot-status");

      function renderRobotStatus(payload) {
        if (!payload || !payload.data) return;

        const dashboard = payload.data.dashboard || {};
        const rtde = payload.data.rtde || {};

        // Dashboard status
        if (robotMode) {
          const mode = dashboard.robotmode || "unknown";
          robotMode.textContent = mode;
          robotMode.className = 'monitor-value badge ' + (mode === 'RUNNING' ? 'badge-ok' : 'badge-error');
        }

        if (safetyMode) {
          const mode = dashboard.safetymode || "unknown";
          safetyMode.textContent = mode;
          safetyMode.className = 'monitor-value badge ' + (mode === 'NORMAL' ? 'badge-ok' : 'badge-warning');
        }

        if (programState) {
          const state = dashboard.programState || "unknown";
          programState.textContent = state;
          programState.className = 'monitor-value badge ' + (state === 'PLAYING' ? 'badge-ok' : 'badge-error');
        }

        if (remoteControl) {
          const rc = dashboard.remote_control || "unknown";
          remoteControl.textContent = rc;
          remoteControl.className = 'monitor-value badge ' + (rc === 'true' ? 'badge-ok' : 'badge-error');
        }

        // RTDE data
        if (jointPositions && rtde.joints) {
          jointPositions.textContent = '[' + rtde.joints.map(j => j.toFixed(4)).join(', ') + ']';
        }

        if (tcpPose && rtde.tcp_pose) {
          tcpPose.textContent = '[' + rtde.tcp_pose.map(p => p.toFixed(4)).join(', ') + ']';
        }

        // Warnings
        if (robotStatusWarning) {
          const warnings = [];
          const robotModeClean = (dashboard.robotmode || "").replace(/^Robotmode:\\s*/i, "").trim();
          if (robotModeClean && robotModeClean !== "RUNNING") {
            warnings.push('<span class=' + '"material-icons md-18"' + '>warning</span> Robot non in RUNNING: ' + robotModeClean);
          }
          if (dashboard.programState && !dashboard.programState.includes("PLAYING")) {
            warnings.push('<span class=' + '"material-icons md-18"' + '>warning</span> Programma non in PLAYING: ' + dashboard.programState);
          }
          if (dashboard.remote_control && dashboard.remote_control !== "true") {
            warnings.push('<span class=' + '"material-icons md-18"' + '>warning</span> Remote control non attivo');
          }
          if (rtde.error) {
            warnings.push('<span class=' + '"material-icons md-18"' + '>warning</span> RTDE: ' + rtde.error);
          }
          if (dashboard.error) {
            warnings.push('<span class=' + '"material-icons md-18"' + '>warning</span> Dashboard: ' + dashboard.error);
          }
          robotStatusWarning.textContent = warnings.join(" | ");
          robotStatusWarning.style.display = warnings.length > 0 ? "block" : "none";
        }

        // Info
        if (robotStatusInfo) {
          const infos = [];
          if (rtde.joints && rtde.tcp_pose) {
            infos.push('<span class=' + '"material-icons md-18"' + '>check_circle</span> Dati RTDE disponibili');
          }
          if (dashboard.robotmode === "RUNNING" && dashboard.programState === "PLAYING") {
            infos.push('<span class=' + '"material-icons md-18"' + '>check_circle</span> Robot pronto per controllo');
          }
          robotStatusInfo.textContent = infos.join(" | ");
          robotStatusInfo.style.display = infos.length > 0 ? "block" : "none";
        }

        if (robotStatusJson) {
          robotStatusJson.textContent = JSON.stringify(payload.data, null, 2);
        }
        if (robotStatusTimestamp) {
          robotStatusTimestamp.textContent = 'Agg. ' + new Date().toLocaleTimeString();
        }
      }

      async function fetchRobotStatus(showToast = false) {
        try {
          const response = await fetch("/api/robot_status");
          const payload = await response.json();
          if (payload.status === "ok") {
            renderRobotStatus(payload);
            if (showToast) {
              setStatus("Stato robot aggiornato", true);
            }
          } else {
            setStatus("Robot Status API error", false);
          }
        } catch (err) {
          console.error("Robot status fetch error", err);
          setStatus("Impossibile leggere lo stato robot", false);
        }
      }

      if (refreshRobotStatusBtn) {
        refreshRobotStatusBtn.addEventListener("click", () => fetchRobotStatus(true));
      }
      if (robotMode) {
        fetchRobotStatus();
        setInterval(fetchRobotStatus, 2000);  // Aggiorna ogni 2 secondi
      }

      // --- Sistema Robot Control ---
      const startDriverBtn = document.getElementById("start-ros2-driver");
      const stopDriverBtn = document.getElementById("stop-ros2-driver");
      const switchControllerBtn = document.getElementById("switch-controller");
      const refreshSystemStatusBtn = document.getElementById("refresh-system-status");
      const systemMessages = document.getElementById("system-messages");
      const ros2DriverStatus = document.getElementById("ros2-driver-status");
      const controllerStatus = document.getElementById("controller-status");
      const robotModeStatus = document.getElementById("robot-mode-status");
      const portStatus = document.getElementById("port-status");
      const systemStatusTimestamp = document.getElementById("system-status-timestamp");

      function updateSystemMessage(msg, isError = false) {
        if (systemMessages) {
          systemMessages.innerHTML = '<div style="color: ' + (isError ? '#cc0000' : '#0066cc') + ';">' + msg + '</div>';
        }
      }

      async function fetchSystemStatus(showToast = false) {
        try {
          const response = await fetch("/api/system/status");
          const payload = await response.json();
          if (payload.status === "ok") {
            const data = payload.data;
            
            if (ros2DriverStatus) {
              if (data.ros2_driver.running) {
                ros2DriverStatus.innerHTML = '<span class=' + '"material-icons md-18"' + '>check_circle</span> Attivo';
                ros2DriverStatus.className = "monitor-value badge badge-success";
              } else {
                ros2DriverStatus.innerHTML = '<span class=' + '"material-icons md-18"' + '>error</span> Fermo';
                ros2DriverStatus.className = "monitor-value badge badge-error";
              }
            }
            
            if (controllerStatus) {
              if (data.controller.active) {
                controllerStatus.innerHTML = '<span class=' + '"material-icons md-18"' + '>check_circle</span> ' + data.controller.name;
                controllerStatus.className = "monitor-value badge badge-success";
              } else {
                controllerStatus.innerHTML = data.controller.error || '<span class=' + '"material-icons md-18"' + '>error</span> Nessuno';
                controllerStatus.className = "monitor-value badge badge-error";
              }
            }
            
            if (robotModeStatus) {
              const mode = data.robot_mode || "—";
              robotModeStatus.textContent = mode;
              robotModeStatus.className = "monitor-value badge " + (mode === "RUNNING" ? "badge-success" : "badge-error");
            }
            
            if (portStatus) {
              if (data.port_50002.listening) {
                portStatus.innerHTML = '<span class=' + '"material-icons md-18"' + '>check_circle</span> Aperta';
                portStatus.className = "monitor-value badge badge-success";
              } else {
                portStatus.innerHTML = '<span class=' + '"material-icons md-18"' + '>error</span> Chiusa';
                portStatus.className = "monitor-value badge badge-error";
              }
            }
            
            // Aggiorna status rapido
            const quickStatus = document.getElementById("quick-status");
            const ros2DriverQuick = document.getElementById("ros2-driver-status-quick");
            const controllerQuick = document.getElementById("controller-status-quick");
            const robotModeQuick = document.getElementById("robot-mode-status-quick");
            const portQuick = document.getElementById("port-status-quick");
            
            if (quickStatus && ros2DriverQuick && controllerQuick && robotModeQuick && portQuick) {
              quickStatus.style.display = "block";
              if (data.ros2_driver.running) {
                ros2DriverQuick.innerHTML = '<span class=' + '"material-icons md-18"' + '>check_circle</span> Attivo';
              } else {
                ros2DriverQuick.innerHTML = '<span class=' + '"material-icons md-18"' + '>error</span> Fermo';
              }
              if (data.controller.active) {
                controllerQuick.textContent = data.controller.name;
              } else {
                controllerQuick.innerHTML = '<span class=' + '"material-icons md-18"' + '>error</span> Nessuno';
              }
              robotModeQuick.textContent = data.robot_mode || "—";
              if (data.port_50002.listening) {
                portQuick.innerHTML = '<span class=' + '"material-icons md-18"' + '>check_circle</span> Aperta';
              } else {
                portQuick.innerHTML = '<span class=' + '"material-icons md-18"' + '>error</span> Chiusa';
              }
            }
            
            // NON aggiornare wizard se è già completato - evita reset indesiderati
            // Il wizard viene aggiornato solo manualmente dagli step
            // if (data.robot_mode === "RUNNING" && data.port_50002.listening) {
            //   const stepBStatus = document.getElementById("step-b-status");
            //   if (stepBStatus && stepBStatus.textContent.includes("Bloccato")) {
            //     updateWizardStep('a', 'success', 'Driver ROS2 attivo.');
            //     updateWizardStep('b', 'success', 'Driver verificato.');
            //   }
            // }
            
            if (systemStatusTimestamp) {
              systemStatusTimestamp.textContent = 'Agg. ' + new Date().toLocaleTimeString();
            }
            
            if (showToast) {
              setStatus("Stato sistema aggiornato", true);
            }
          }
        } catch (err) {
          console.error("System status fetch error", err);
          updateSystemMessage("Errore lettura stato sistema: " + err.message, true);
        }
      }

      // --- Wizard Guidato ---
      let wizardCurrentStep = 'a';
      let wizardCheckInterval = null;
      
      // Funzione per inizializzare event listeners del wizard
      function initWizardEventListeners() {
        console.log("[DEBUG] initWizardEventListeners chiamata");
        const wizardStartDriverBtn = document.getElementById("wizard-start-driver");
        const wizardCheckTeachPendantBtn = document.getElementById("wizard-check-teach-pendant");
        
        console.log("[DEBUG] wizardStartDriverBtn:", wizardStartDriverBtn);
        console.log("[DEBUG] wizardStepA definita:", typeof wizardStepA);
        
        if (wizardStartDriverBtn) {
          if (typeof wizardStepA === 'function') {
            wizardStartDriverBtn.addEventListener("click", (e) => {
              console.log("[DEBUG] Click su Start Driver button");
              e.preventDefault();
              wizardStepA();
            });
            console.log("[OK] Event listener aggiunto al pulsante Start Driver");
          } else {
            console.error("[ERROR] wizardStepA non è una funzione!");
          }
        } else {
          console.error("[ERROR] Pulsante wizard-start-driver non trovato!");
        }

        if (wizardCheckTeachPendantBtn) {
          wizardCheckTeachPendantBtn.addEventListener("click", () => {
            updateWizardStep('d', 'success', 'Verifica connessione in corso...');
            wizardStepE();
          });
        }

        // Pulsante Activate Controller (Step C)
        const activateControllerBtn = document.getElementById("wizard-activate-controller");
        if (activateControllerBtn) {
          activateControllerBtn.addEventListener("click", () => {
            wizardStepC();
          });
        }

        // Pulsanti Retry
        for (const step of ['a', 'b', 'c', 'd', 'e']) {
          const retryBtn = document.getElementById('wizard-retry-' + step);
          if (retryBtn) {
            retryBtn.addEventListener("click", () => {
              if (step === 'a') wizardStepA();
              else if (step === 'b') wizardStepB();
              else if (step === 'c') wizardStepC();
              else if (step === 'd') wizardStepD();
              else if (step === 'e') wizardStepE();
            });
          }
        }
      }

      function updateWizardStep(step, status, message) {
        const stepEl = document.getElementById('step-' + step);
        const statusEl = document.getElementById('step-' + step + '-status');
        const messageEl = document.getElementById('step-' + step + '-message');
        const retryBtn = document.getElementById('wizard-retry-' + step);
        const activateBtn = document.getElementById('wizard-activate-controller');
        
        if (stepEl) {
          stepEl.classList.remove('active', 'completed', 'error');
          stepEl.style.opacity = '1';
          if (status === 'success') {
            // Nascondi pulsante Activate quando completato
            if (activateBtn && step === 'c') {
              activateBtn.style.display = 'none';
            }
            stepEl.classList.add('completed');
          } else if (status === 'error') {
            stepEl.classList.add('error');
          } else if (status === 'active' || status === 'waiting') {
            stepEl.classList.add('active');
          } else {
            stepEl.style.opacity = '0.5';
          }
        }
        
        if (statusEl) {
          statusEl.className = 'wizard-step__status';
          if (status === 'success') {
            statusEl.innerHTML = '<span class=' + '"material-icons md-18"' + '>check_circle</span>';
            statusEl.classList.add('success');
          } else if (status === 'error') {
            statusEl.innerHTML = '<span class=' + '"material-icons md-18"' + '>error</span>';
            statusEl.classList.add('error');
          } else if (status === 'waiting') {
            statusEl.innerHTML = '<span class=' + '"material-icons md-18"' + '>hourglass_empty</span>';
            statusEl.classList.add('waiting');
          } else if (status === 'active') {
            statusEl.innerHTML = '<span class=' + '"material-icons md-18"' + '>refresh</span>';
            statusEl.classList.add('waiting');
          } else {
            statusEl.innerHTML = '<span class=' + '"material-icons md-18"' + '>pause</span>';
          }
        }
        
        // Mostra/nascondi barra di caricamento
        const progressEl = document.getElementById('step-' + step + '-progress');
        if (progressEl) {
          if (status === 'active') {
            progressEl.style.display = 'block';
          } else {
            progressEl.style.display = 'none';
          }
        }
        
        if (messageEl && message) {
          messageEl.innerHTML = message;
        }
        
        // Mostra pulsante retry se errore
        if (retryBtn) {
          retryBtn.style.display = (status === 'error') ? 'block' : 'none';
        }
        
        // Gestisci pulsante Activate Controller (solo per step C)
        if (activateBtn && step === 'c') {
          if (status === 'success') {
            activateBtn.style.display = 'none';  // Nascondi quando completato
          } else if (status === 'error' || status === 'waiting' || status === 'active') {
            activateBtn.style.display = 'block';  // Mostra quando serve attivazione
          } else {
            activateBtn.style.display = 'none';
          }
        }
      }

      async function wizardStepA() {
        console.log('[WIZARD STEP A] Inizio wizard step A');
        try {
          updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Verifica pre-avvio...');
          
          // STEP 1: Verifica robot raggiungibile
          console.log('[WIZARD STEP A] Verifica robot raggiungibile...');
          updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Verifica connessione robot...');
        } catch (err) {
          console.error('[WIZARD STEP A] Errore iniziale:', err);
          updateWizardStep('a', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Errore: ' + err.message);
          return;
        }
        
        try {
          const robotCheck = await fetch("/api/system/check_robot_connection", { method: "POST" });
          console.log('[WIZARD STEP A] Risposta check robot:', robotCheck.status);
          const robotData = await robotCheck.json();
          console.log('[WIZARD STEP A] Dati robot:', robotData);
          if (robotData.status !== "ok" || !robotData.data.reachable) {
            console.error('[WIZARD STEP A] Robot non raggiungibile');
            updateWizardStep('a', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Robot non raggiungibile!<br><small>Verifica che il robot sia acceso e connesso alla rete.<br>IP: 192.168.10.194</small>');
            return;
          }
          console.log('[WIZARD STEP A] Robot raggiungibile OK');
        } catch (err) {
          console.error('[WIZARD STEP A] Errore verifica robot:', err);
          updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>warning</span> Impossibile verificare robot. Procedo comunque...');
        }
        
        // STEP 2: Verifica e kill processi esistenti (IMPORTANTE per evitare crash)
        updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Pulizia processi esistenti...');
        try {
          const checkResponse = await fetch("/api/system/check_processes", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({kill_duplicates: true})
          });
          const checkPayload = await checkResponse.json();
          if (checkPayload.status === "ok") {
            if (checkPayload.data.duplicates_found) {
              updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Processi duplicati terminati. Attendo pulizia completa (5 secondi)...');
              await new Promise(resolve => setTimeout(resolve, 5000)); // Attendi 5 secondi per pulizia completa
            } else {
              updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Nessun processo duplicato. Procedo...');
              await new Promise(resolve => setTimeout(resolve, 1000));
            }
          }
        } catch (err) {
          console.warn("Errore verifica processi:", err);
          updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>warning</span> Impossibile verificare processi. Procedo comunque...');
          await new Promise(resolve => setTimeout(resolve, 2000)); // Attendi comunque per sicurezza
        }
        
        // STEP 3: Avvia driver con retry automatico e verifica stabilità
        updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Avvio driver ROS2...');
        let retryCount = 0;
        const maxRetries = 5; // Aumentato a 5 retry per maggiore stabilità
        
        // Funzione per verificare che il driver sia stabile
        const verifyDriverStable = async (maxChecks = 10, checkInterval = 2000) => {
          for (let i = 0; i < maxChecks; i++) {
            try {
              const statusResponse = await fetch("/api/system/status");
              const statusData = await statusResponse.json();
              if (statusData.status === "ok" && statusData.data.ros2_driver.running) {
                // Verifica anche che la porta 50002 sia aperta
                if (statusData.data.port_50002 && statusData.data.port_50002.listening) {
                  return true; // Driver stabile e porta aperta
                }
              }
              // Se non è ancora stabile, aspetta prima del prossimo check
              if (i < maxChecks - 1) {
                await new Promise(resolve => setTimeout(resolve, checkInterval));
              }
            } catch (err) {
              console.warn('[VERIFY DRIVER] Errore verifica:', err);
              if (i < maxChecks - 1) {
                await new Promise(resolve => setTimeout(resolve, checkInterval));
              }
            }
          }
          return false; // Driver non stabile dopo tutti i check
        };
        
        const tryStartDriver = async () => {
          try {
            const response = await fetch("/api/system/start_driver", { method: "POST" });
            const payload = await response.json();
            if (payload.status === "ok") {
              updateWizardStep('a', 'waiting', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Driver avviato! Verifico stabilità (20 secondi)...');
              
              // Verifica che il driver sia stabile prima di procedere
              const isStable = await verifyDriverStable(10, 2000); // 10 check ogni 2 secondi = 20 secondi totali
              
              if (isStable) {
                updateWizardStep('a', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Driver stabile e pronto!');
                await new Promise(resolve => setTimeout(resolve, 2000)); // Attesa aggiuntiva per sicurezza
                wizardStepB();
              } else {
                // Driver avviato ma non stabile - riprova se abbiamo ancora tentativi
                if (retryCount < maxRetries) {
                  retryCount++;
                  updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Driver non stabile. Riprovo (tentativo ' + retryCount + '/' + maxRetries + ')...<br><small>Pulizia completa in corso...</small>');
                  // Pulisci tutto e riprova
                  await fetch("/api/system/check_processes", {
                    method: "POST",
                    headers: {"Content-Type": "application/json"},
                    body: JSON.stringify({kill_duplicates: true})
                  });
                  await new Promise(resolve => setTimeout(resolve, 8000)); // Attendi pulizia più a lungo
                  await tryStartDriver(); // Retry
                } else {
                  updateWizardStep('a', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Driver non si stabilizza dopo ' + maxRetries + ' tentativi.<br><small>Verifica i log del driver e riprova manualmente.</small>');
                }
              }
            } else {
              // Se c'è un errore e abbiamo ancora tentativi, riprova
              const errorMsg = payload.message || '';
              const isCrash = errorMsg.includes("crashato") || errorMsg.includes("Segmentation fault") || errorMsg.includes("ERROR") || errorMsg.includes("died") || errorMsg.includes("failed");
              
              if (retryCount < maxRetries && isCrash) {
                retryCount++;
                updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Driver crashato. Riprovo automaticamente (tentativo ' + retryCount + '/' + maxRetries + ')...<br><small>Pulizia completa in corso...</small>');
                // Pulisci tutto e riprova
                await fetch("/api/system/check_processes", {
                  method: "POST",
                  headers: {"Content-Type": "application/json"},
                  body: JSON.stringify({kill_duplicates: true})
                });
                await new Promise(resolve => setTimeout(resolve, 8000)); // Attendi pulizia più a lungo
                await tryStartDriver(); // Retry
              } else {
                // Formatta messaggio errore in modo più leggibile
                let errorMsg = payload.message || 'Errore sconosciuto';
                // Rimuovi dettagli tecnici eccessivi se presenti
                if (errorMsg.length > 500) {
                  errorMsg = errorMsg.substring(0, 500) + '...' + String.fromCharCode(10) + String.fromCharCode(10) + '[Clicca ' + String.fromCharCode(39) + 'Riprova' + String.fromCharCode(39) + ' per vedere log completo]';
                }
                updateWizardStep('a', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Errore avvio driver:<br><small>' + errorMsg.replace(/\\n/g, '<br>') + '</small><br><br><small><strong>Soluzioni:</strong><br>1. Verifica robot acceso e raggiungibile<br>2. Clicca "Riprova" per riprovare<br>3. Se persiste, riavvia robot e riprova</small>');
              }
            }
          } catch (err) {
            if (retryCount < maxRetries) {
              retryCount++;
              updateWizardStep('a', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Errore connessione. Riprovo (tentativo ' + retryCount + '/' + maxRetries + ')...');
              await new Promise(resolve => setTimeout(resolve, 3000)); // Attesa più lunga tra retry
              await tryStartDriver();
            } else {
              updateWizardStep('a', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Errore di connessione: ' + err.message + '<br><small>Verifica che la web interface sia attiva e riprova.</small>');
            }
          }
        };
        
        await tryStartDriver();
      }

      async function wizardStepB() {
        updateWizardStep('b', 'active', '<span class="material-icons md-18">refresh</span> Verifica driver attivo e porta 50002...');
        let attempts = 0;
        const maxAttempts = 15; // Aumentato a 15 tentativi (30 secondi totali)
        
        const checkDriver = async () => {
          attempts++;
          try {
            const response = await fetch("/api/system/status");
            const payload = await response.json();
            if (payload.status === "ok") {
              const data = payload.data;
              if (data.ros2_driver.running && data.port_50002.listening) {
                updateWizardStep('a', 'success', 'Driver ROS2 attivo e porta 50002 aperta.');
                updateWizardStep('b', 'success', 'Driver verificato correttamente.');
                setTimeout(() => wizardStepC(), 1000);
                return true;
              } else {
                // Mostra progresso ogni 3 tentativi
                if (attempts % 3 === 0) {
                  const driverStatus = data.ros2_driver.running ? 'attivo' : 'non attivo';
                  const portStatus = data.port_50002.listening ? 'aperta' : 'chiusa';
                  updateWizardStep('b', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Verifica in corso... (' + attempts + '/' + maxAttempts + ')<br><small>Driver: ' + driverStatus + ', Porta 50002: ' + portStatus + '</small>');
                }
                
                if (attempts >= maxAttempts) {
                  let errorMsg = 'Driver non pronto dopo ' + maxAttempts + ' tentativi.<br>';
                  if (!data.ros2_driver.running) {
                    errorMsg += '<br><span class="material-icons md-18">error</span> Driver ROS2 non attivo.<br>';
                    errorMsg += '<small>Possibili cause:<br>- Driver crashato durante avvio<br>- Problemi di inizializzazione<br><br>Clicca "Riprova" nello step A per riavviare.</small>';
                  }
                  if (!data.port_50002.listening) {
                    errorMsg += '<br><span class="material-icons md-18">error</span> Porta 50002 non aperta.<br>';
                    errorMsg += '<small>Il driver potrebbe non essere completamente avviato.</small>';
                  }
                  updateWizardStep('b', 'error', errorMsg);
                  return true;
                }
              }
            }
          } catch (err) {
            console.error("Wizard step B error", err);
            if (attempts >= maxAttempts) {
              updateWizardStep('b', 'error', 'Errore verifica driver: ' + err.message + '<br><small>Clicca "Riprova" per riprovare.</small>');
              return true;
            }
          }
          return false;
        };
        
        const interval = setInterval(async () => {
          const done = await checkDriver();
          if (done) {
            clearInterval(interval);
          }
        }, 2000);
        
        // Prima verifica immediata
        await checkDriver();
      }

      // Flag per evitare toast ripetuti nel wizard
      let wizardStepCToastShown = false;
      
      async function wizardStepC() {
        updateWizardStep('c', 'active', '<span class="material-icons md-18">refresh</span> Verifica controller...');
        wizardStepCToastShown = false; // Reset flag all'inizio
        
        // Prima verifica quale controller è attivo
        let attempts = 0;
        const maxAttempts = 10;
        
        const checkAndActivate = async () => {
          try {
            // Verifica stato controller
            const statusResponse = await fetch("/api/system/status");
            const statusData = await statusResponse.json();
            
            if (statusData.status === "ok" && statusData.data.controller) {
              const controller = statusData.data.controller;
              
              // Se forward_velocity_controller è già attivo, passa al prossimo step
              // IMPORTANTE: Non verifichiamo se il robot risponde ai comandi (quello è compito dello step E)
              // Qui verifichiamo solo che il controller sia tecnicamente attivo
              if (controller.active && controller.name === 'forward_velocity_controller') {
                updateWizardStep('c', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Controller forward_velocity_controller attivo.<br><small>Ora procedi al passo D per attivare Remote Control sul Teach Pendant.</small>');
                if (!wizardStepCToastShown && typeof showToast === 'function') {
                  showToast('Forward velocity controller attivo', 'success', 2000);
                  wizardStepCToastShown = true;
                }
                // Verifica anche che il ROS2 bridge sia stabile prima di procedere
                try {
                  const bridgeStatus = await fetch("/api/system/ros2_bridge_status");
                  const bridgeData = await bridgeStatus.json();
                  if (bridgeData.status === "ok" && bridgeData.data && bridgeData.data.initialized) {
                    updateWizardStep('c', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Controller forward_velocity_controller attivo.<br><small>Ora procedi al passo D per attivare Remote Control sul Teach Pendant.</small>');
                    if (!wizardStepCToastShown && typeof showToast === 'function') {
                      showToast('Forward velocity controller attivo', 'success', 2000);
                      wizardStepCToastShown = true;
                    }
                    await new Promise(resolve => setTimeout(resolve, 2000)); // Attesa per stabilità
                    setTimeout(() => wizardStepD(), 1000);
                    return true;
                  }
                } catch (bridgeErr) {
                  console.warn('[WIZARD STEP C] Errore verifica bridge:', bridgeErr);
                  // Continua comunque se il controller è attivo
                }
                
                updateWizardStep('c', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Controller forward_velocity_controller attivo.<br><small>Ora procedi al passo D per attivare Remote Control sul Teach Pendant.</small>');
                if (!wizardStepCToastShown && typeof showToast === 'function') {
                  showToast('Forward velocity controller attivo', 'success', 2000);
                  wizardStepCToastShown = true;
                }
                await new Promise(resolve => setTimeout(resolve, 2000)); // Attesa per stabilità
                setTimeout(() => wizardStepD(), 1000);
                return true;
              }
              
              // Se non è attivo, prova ad attivarlo
              if (!controller.active || controller.name !== 'forward_velocity_controller') {
                updateWizardStep('c', 'active', '<span class=' + '"material-icons md-18"' + '>refresh</span> Attivazione forward_velocity_controller... (tentativo ' + (attempts + 1) + '/' + maxAttempts + ')');
                
                const switchResponse = await fetch("/api/system/switch_controller", {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({ use_scaled: false })
                });
                
                const switchData = await switchResponse.json();
                
                if (switchData.status === "ok") {
                  // Attendi un momento e verifica che sia stato attivato
                  await new Promise(resolve => setTimeout(resolve, 2000)); // Aumentato a 2 secondi
                  
                  // Verifica di nuovo - solo che sia in stato 'active', non che risponda ai comandi
                  const verifyResponse = await fetch("/api/system/status");
                  const verifyData = await verifyResponse.json();
                  
                  if (verifyData.status === "ok" && verifyData.data.controller) {
                    const newController = verifyData.data.controller;
                    // IMPORTANTE: Verifichiamo solo che sia attivo, non che il robot risponda
                    // Il robot potrebbe non essere in Remote Control ancora, ma il controller è comunque attivo
                    if (newController.active && newController.name === 'forward_velocity_controller') {
                      // Verifica anche che il ROS2 bridge sia stabile
                      try {
                        const bridgeStatus = await fetch("/api/system/ros2_bridge_status");
                        const bridgeData = await bridgeStatus.json();
                        if (bridgeData.status === "ok" && bridgeData.data && bridgeData.data.initialized) {
                          updateWizardStep('c', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Controller forward_velocity_controller attivato.<br><small>Ora procedi al passo D per attivare Remote Control sul Teach Pendant.</small>');
                          if (!wizardStepCToastShown && typeof showToast === 'function') {
                            showToast('Forward velocity controller attivato', 'success', 2000);
                            wizardStepCToastShown = true;
                          }
                          await new Promise(resolve => setTimeout(resolve, 2000)); // Attesa per stabilità
                          setTimeout(() => wizardStepD(), 1000);
                          return true;
                        }
                      } catch (bridgeErr) {
                        console.warn('[WIZARD STEP C] Errore verifica bridge:', bridgeErr);
                        // Continua comunque se il controller è attivo
                      }
                      
                      updateWizardStep('c', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Controller forward_velocity_controller attivato.<br><small>Ora procedi al passo D per attivare Remote Control sul Teach Pendant.</small>');
                      if (!wizardStepCToastShown && typeof showToast === 'function') {
                        showToast('Forward velocity controller attivato', 'success', 2000);
                        wizardStepCToastShown = true;
                      }
                      await new Promise(resolve => setTimeout(resolve, 2000)); // Attesa per stabilità
                      setTimeout(() => wizardStepD(), 1000);
                      return true;
                    }
                  }
                } else {
                  // Se l'errore è che il controller non può essere attivato perché il robot non è in Remote Control,
                  // consideriamo comunque un successo parziale se il servizio ha risposto
                  if (switchData.message && (switchData.message.includes('already active') || switchData.message.includes('già attivo'))) {
                    updateWizardStep('c', 'success', '<span class=' + '"material-icons md-18"' + '>check_circle</span> Controller già attivo.<br><small>Ora procedi al passo D per attivare Remote Control sul Teach Pendant.</small>');
                    await new Promise(resolve => setTimeout(resolve, 2000)); // Attesa per stabilità
                    setTimeout(() => wizardStepD(), 1000);
                    return true;
                  }
                  // Mostra errore solo una volta
                  if (!wizardStepCToastShown) {
                    updateWizardStep('c', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Errore attivazione: ' + switchData.message);
                    if (typeof showToast === 'function') {
                      showToast('Errore attivazione controller: ' + switchData.message, 'error', 4000);
                      wizardStepCToastShown = true;
                    }
                  }
                  return false;
                }
              }
            }
            
            attempts++;
            if (attempts >= maxAttempts) {
              updateWizardStep('c', 'error', '<span class="material-icons md-18">error</span> Controller non attivato dopo ' + maxAttempts + ' tentativi.<br><small>Verifica che il driver ROS2 sia attivo. Puoi comunque procedere al passo D se il controller è già attivo manualmente.</small>');
              if (!wizardStepCToastShown && typeof showToast === 'function') {
                showToast('Impossibile attivare forward_velocity_controller dopo ' + maxAttempts + ' tentativi', 'error', 5000);
                wizardStepCToastShown = true;
              }
              // Non blocchiamo il wizard - permette di procedere comunque
              setTimeout(() => wizardStepD(), 2000);
              return true; // Ferma il loop
            }
            
            return false; // Continua a provare
          } catch (err) {
            console.error("Wizard step C error", err);
            attempts++;
            if (attempts >= maxAttempts) {
              updateWizardStep('c', 'error', '<span class=' + '"material-icons md-18"' + '>error</span> Errore: ' + err.message);
              if (!wizardStepCToastShown && typeof showToast === 'function') {
                showToast('Errore durante attivazione controller: ' + err.message, 'error', 4000);
                wizardStepCToastShown = true;
              }
              return true;
            }
            return false;
          }
        };
        
        const interval = setInterval(async () => {
          const done = await checkAndActivate();
          if (done) {
            clearInterval(interval);
          }
        }, 2000);
        
        // Prima verifica immediata
        checkAndActivate();
      }
      
      // Verifica automatica forward_velocity_controller ogni 30 secondi
      let controllerCheckInterval = null;
      let lastControllerToastTime = 0;
      let controllerWasActive = false;
      function startControllerAutoCheck() {
        if (controllerCheckInterval) return; // Già attivo
        
        controllerCheckInterval = setInterval(async () => {
          try {
            const response = await fetch("/api/system/status");
            const payload = await response.json();
            
            if (payload.status === "ok" && payload.data.controller) {
              const controller = payload.data.controller;
              const isActive = controller.active && controller.name === 'forward_velocity_controller';
              
              // Se il controller è attivo, aggiorna il flag e non fare nulla
              if (isActive) {
                controllerWasActive = true;
                return; // Controller OK, non fare nulla
              }
              
              // Se il controller non è attivo, prova a riattivarlo SOLO se prima era attivo
              // (evita spam se il controller non può essere attivato)
              if (!isActive && controllerWasActive) {
                console.warn('[WARN] Forward velocity controller non attivo, tentativo riattivazione...');
                
                const switchResponse = await fetch("/api/system/switch_controller", {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({ use_scaled: false })
                });
                
                const switchData = await switchResponse.json();
                if (switchData.status === "ok") {
                  // Mostra toast solo una volta ogni 5 minuti per evitare spam
                  const now = Date.now();
                  if (now - lastControllerToastTime > 300000) { // 5 minuti
                    if (typeof showToast === 'function') {
                      showToast('Forward velocity controller riattivato automaticamente', 'info', 3000);
                      lastControllerToastTime = now;
                    }
                  }
                  controllerWasActive = true; // Aggiorna flag dopo riattivazione
                } else {
                  // Se la riattivazione fallisce, non mostrare toast ripetuti
                  controllerWasActive = false;
                }
              }
            }
          } catch (err) {
            console.error('Errore verifica automatica controller:', err);
          }
        }, 30000); // Ogni 30 secondi
      }
      
      // Avvia verifica automatica quando la pagina è caricata
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', startControllerAutoCheck);
      } else {
        startControllerAutoCheck();
      }

      function wizardStepD() {
        updateWizardStep('d', 'waiting', 'Attendi che attivi External Control sul Teach Pendant, poi clicca il pulsante qui sotto.');
        const btn = document.getElementById("wizard-check-teach-pendant");
        if (btn) {
          btn.style.display = "block";
        }
      }

      async function wizardStepE() {
        updateWizardStep('e', 'active', '<span class="material-icons md-18">refresh</span> Verifica connessione robot...');
        let attempts = 0;
        const maxAttempts = 15;
        
        const checkConnection = async () => {
          attempts++;
          try {
            const response = await fetch("/api/system/status");
            const payload = await response.json();
            if (payload.status === "ok") {
              const data = payload.data;
              
              // Verifica flessibile: se driver è attivo e porta è aperta, considera pronto
              // anche se alcuni dati dashboard non sono disponibili
              const driverReady = data.ros2_driver.running && data.port_50002.listening;
              // Gestisci controller.active che può essere null, undefined, true, o false
              const controllerReady = data.controller && (data.controller.active === true);
              
              // Verifica stato robot (più flessibile)
              const robotModeOk = data.robot_mode === "RUNNING" || 
                                 data.robot_mode === "unknown" || 
                                 data.robot_mode === null ||
                                 data.robot_mode === "";
              const safetyOk = data.robot_safety_mode === "NORMAL" || 
                              data.robot_safety_mode === "unknown" || 
                              data.robot_safety_mode === null ||
                              data.robot_safety_mode === "";
              const remoteOk = data.remote_control === true || 
                              data.program_state === "PLAYING" ||
                              data.program_state === "PLAYING remote_control.urp" ||
                              data.remote_control === null ||
                              data.remote_control === "unknown";
              
              // Considera pronto se driver e controller sono attivi
              // e almeno uno dei dati robot è disponibile/OK
              const isReady = driverReady && controllerReady && robotModeOk;
              
              // Debug info
              const driverIcon = driverReady ? '<span class=' + '"material-icons md-18"' + '>check_circle</span>' : '<span class=' + '"material-icons md-18"' + '>error</span>';
              const controllerIcon = controllerReady ? '<span class=' + '"material-icons md-18"' + '>check_circle</span>' : '<span class=' + '"material-icons md-18"' + '>error</span>';
              const debugInfo = 'Driver: ' + driverIcon + ', Controller: ' + controllerIcon + ', Mode: ' + (data.robot_mode || 'N/A') + ', Safety: ' + (data.robot_safety_mode || 'N/A') + ', Remote: ' + (data.remote_control || 'N/A') + ', Program: ' + (data.program_state || 'N/A');
              
              if (isReady) {
                let successMsg = '<span class="material-icons md-18">check_circle</span> <strong style="font-size: 16px; color: #00aa00;">Robot connesso e pronto!</strong><br>';
                if (data.robot_mode === "RUNNING" && data.robot_safety_mode === "NORMAL") {
                  successMsg += 'Tutti i controlli verificati correttamente.';
                } else {
                  successMsg += 'Driver e controller attivi. Alcuni dati robot non disponibili ma sistema operativo.';
                }
                successMsg += '<br><small style="color: #666;">Scorri in basso per vedere i joystick di controllo.</small>';
                
                updateWizardStep('e', 'success', successMsg);
                // Mostra sezione joystick con animazione
                const joystickSection = document.getElementById('joystick-section');
                if (joystickSection) {
                  joystickSection.style.display = 'block';
                  // NON fare scroll automatico - l'utente potrebbe voler scrollare manualmente
                  // joystickSection.scrollIntoView({ behavior: 'smooth', block: 'start' });
                  // Evidenzia la sezione
                  joystickSection.style.animation = 'pulse 2s ease-in-out';
                  setTimeout(() => {
                    joystickSection.style.animation = '';
                  }, 2000);
                }
                // Mostra sezione Vision/MoveIt
                const visionSection = document.getElementById('vision-moveit-section');
                if (visionSection) {
                  visionSection.style.display = 'block';
                }
                const advancedControls = document.getElementById('advanced-controls');
                if (advancedControls) {
                  advancedControls.style.display = 'block';
                }
                if (wizardCheckInterval) {
                  clearInterval(wizardCheckInterval);
                  wizardCheckInterval = null;
                }
                return true;
              } else if (attempts >= maxAttempts) {
                let errorMsg = 'Robot non pronto dopo ' + maxAttempts + ' tentativi.<br>';
                errorMsg += '<small style="color: #666;">Debug: ' + debugInfo + '</small><br>';
                if (!driverReady) errorMsg += '<br><span class=' + '"material-icons md-18"' + '>error</span> Driver ROS2 non attivo o porta 50002 chiusa.';
                if (!controllerReady) errorMsg += '<br><span class=' + '"material-icons md-18"' + '>error</span> Controller non attivo.';
                if (data.robot_mode && data.robot_mode !== "RUNNING" && data.robot_mode !== "unknown") {
                  errorMsg += '<br><span class=' + '"material-icons md-18"' + '>warning</span> Modalità robot: ' + data.robot_mode + ' (atteso: RUNNING).';
                }
                if (data.robot_safety_mode && data.robot_safety_mode !== "NORMAL" && data.robot_safety_mode !== "unknown") {
                  errorMsg += '<br><span class=' + '"material-icons md-18"' + '>warning</span> Safety mode: ' + data.robot_safety_mode + ' (atteso: NORMAL).';
                }
                if (data.remote_control !== true && data.program_state !== "PLAYING" && data.program_state !== "PLAYING remote_control.urp") {
                  errorMsg += '<br><span class=' + '"material-icons md-18"' + '>warning</span> Remote Control: ' + (data.remote_control || 'non disponibile') + ', Program: ' + (data.program_state || 'non disponibile');
                }
                errorMsg += '<br><br><small>Se il robot è effettivamente in esecuzione, puoi comunque provare a usare i joystick.</small>';
                updateWizardStep('e', 'error', errorMsg);
                // Mostra comunque i joystick se driver è OK (controller può essere null se non ancora verificato)
                if (driverReady) {
                  const joystickSection = document.getElementById('joystick-section');
                  if (joystickSection) {
                    joystickSection.style.display = 'block';
                  }
                  // Mostra anche sezione Vision/MoveIt
                  const visionSection = document.getElementById('vision-moveit-section');
                  if (visionSection) {
                    visionSection.style.display = 'block';
                  }
                }
                if (wizardCheckInterval) {
                  clearInterval(wizardCheckInterval);
                  wizardCheckInterval = null;
                }
                return true;
              } else {
                // Mostra progresso durante i tentativi
                if (attempts % 3 === 0) {
                  updateWizardStep('e', 'active', '<span class="material-icons md-18">refresh</span> Verifica connessione... (tentativo ' + attempts + '/' + maxAttempts + ')<br><small>' + debugInfo + '</small>');
                }
              }
            }
          } catch (err) {
            console.error("Wizard step E error", err);
          }
          return false;
        };
        
        wizardCheckInterval = setInterval(async () => {
          const done = await checkConnection();
          if (done && wizardCheckInterval) {
            clearInterval(wizardCheckInterval);
            wizardCheckInterval = null;
          }
        }, 2000);
        
        await checkConnection();
      }

      // --- Logging System Variables (dichiarate prima di initLogging) ---
      let lastLogId = -1;
      let logsPaused = false;
      let logUpdateInterval = null;

      // Inizializza wizard con verifica stato iniziale e pulizia processi
      async function initWizard() {
        // Prima verifica e kill processi esistenti
        try {
          const checkResponse = await fetch("/api/system/check_processes", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({kill_duplicates: true})
          });
          const checkPayload = await checkResponse.json();
          if (checkPayload.status === "ok" && checkPayload.data.duplicates_found) {
            console.log("Processi duplicati trovati e terminati:", checkPayload.data);
            await new Promise(resolve => setTimeout(resolve, 1500)); // Attendi pulizia
          }
        } catch (err) {
          console.warn("Errore verifica processi iniziale:", err);
        }
        
        fetchSystemStatus().then(() => {
          setTimeout(async () => {
            try {
              const response = await fetch("/api/system/status");
              const payload = await response.json();
              if (payload.status === "ok") {
                const data = payload.data;
                
                // Step A e B: Driver
                if (data.ros2_driver.running && data.port_50002.listening) {
                  updateWizardStep('a', 'success', 'Driver già attivo.');
                  updateWizardStep('b', 'success', 'Driver verificato.');
                  
                  // Step C: Controller
                  if (data.controller.active && data.controller.name === 'forward_velocity_controller') {
                    updateWizardStep('c', 'success', 'Controller già attivo.');
                    
                    // Step D: Teach Pendant
                    if (data.robot_mode === "RUNNING" && (data.remote_control === true || data.program_state === "PLAYING")) {
                      updateWizardStep('d', 'success', 'External Control già attivo.');
                      // Verifica anche step E
                      setTimeout(() => wizardStepE(), 1000);
                    } else {
                      updateWizardStep('d', 'waiting', 'Attiva External Control sul Teach Pendant.');
                      const btn = document.getElementById("wizard-check-teach-pendant");
                      if (btn) btn.style.display = "block";
                    }
                  } else {
                    updateWizardStep('c', 'waiting', 'Controller non attivo.');
                  }
                } else {
                  // Driver non attivo - tutto bloccato
                  updateWizardStep('a', 'waiting', 'Pronto per avviare. Clicca "Avvia" per iniziare.');
                }
              }
            } catch (err) {
              console.error("Init wizard error", err);
            }
          }, 1000);
        });
      }
      
      // --- Auto-Restart e Health Check System ---
      let healthCheckInterval = null;
      let lastHealthCheck = Date.now();
      let consecutiveFailures = 0;
      const MAX_CONSECUTIVE_FAILURES = 3;
      
      async function performHealthCheck() {
        try {
          const response = await fetch("/api/system/health_check", {
            method: "GET",
            headers: {"Content-Type": "application/json"}
          });
          
          if (response.ok) {
            const data = await response.json();
            if (data.status === "ok") {
              consecutiveFailures = 0;
              lastHealthCheck = Date.now();
              return true;
            }
          }
          
          consecutiveFailures++;
          if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
            console.warn("[HEALTH] Multiple health check failures, attempting auto-restart...");
            await autoRestartWebInterface();
          }
          return false;
        } catch (err) {
          console.error("[HEALTH] Health check error:", err);
          consecutiveFailures++;
          if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
            console.warn("[HEALTH] Multiple health check failures, attempting auto-restart...");
            await autoRestartWebInterface();
          }
          return false;
        }
      }
      
      async function autoRestartWebInterface() {
        try {
          console.log("[AUTO-RESTART] Attempting to restart web interface...");
          const response = await fetch("/api/system/auto_restart", {
            method: "POST",
            headers: {"Content-Type": "application/json"}
          });
          
          const data = await response.json();
          if (data.status === "ok") {
            showToast("Web interface restarting automatically...", "info", 3000);
            // Attendi 3 secondi poi ricarica la pagina
            setTimeout(() => {
              window.location.reload();
            }, 3000);
          } else {
            console.error("[AUTO-RESTART] Failed:", data.message);
            showToast("Auto-restart failed: " + data.message, "error", 5000);
          }
        } catch (err) {
          console.error("[AUTO-RESTART] Error:", err);
          showToast("Auto-restart error: " + err.message, "error", 5000);
        }
      }
      
      // Health check all'avvio della pagina
      async function initHealthCheck() {
        // Verifica immediata all'avvio
        const healthOk = await performHealthCheck();
        if (!healthOk) {
          console.warn("[HEALTH] Initial health check failed, attempting restart...");
          await autoRestartWebInterface();
          return;
        }
        
        // Health check periodico ogni 10 secondi
        healthCheckInterval = setInterval(performHealthCheck, 10000);
        
        // Verifica anche se la pagina è stata inattiva per troppo tempo
        document.addEventListener('visibilitychange', () => {
          if (!document.hidden) {
            // Pagina tornata visibile - verifica stato
            const timeSinceLastCheck = Date.now() - lastHealthCheck;
            if (timeSinceLastCheck > 30000) { // Più di 30 secondi
              performHealthCheck();
            }
          }
        });
      }
      
      // Inizializza tutto quando il DOM è pronto
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => {
          initHealthCheck(); // Prima di tutto, verifica health
          initWizardEventListeners();
          initWizard();
          initLogging();
        });
      } else {
        // DOM già caricato
        initHealthCheck(); // Prima di tutto, verifica health
        initWizardEventListeners();
        initWizard();
        initLogging();
      }

      if (startDriverBtn) {
        startDriverBtn.addEventListener("click", async () => {
          updateSystemMessage("Avvio driver ROS2...");
          try {
            const response = await fetch("/api/system/start_driver", { method: "POST" });
            const payload = await response.json();
            if (payload.status === "ok") {
              updateSystemMessage(payload.message);
              setTimeout(fetchSystemStatus, 2000);
            } else {
              updateSystemMessage("Errore: " + payload.message, true);
            }
          } catch (err) {
            updateSystemMessage("Errore: " + err.message, true);
          }
        });
      }

      if (stopDriverBtn) {
        stopDriverBtn.addEventListener("click", async () => {
          updateSystemMessage("Fermo driver ROS2...");
          try {
            const response = await fetch("/api/system/stop_driver", { method: "POST" });
            const payload = await response.json();
            if (payload.status === "ok") {
              updateSystemMessage(payload.message);
              setTimeout(fetchSystemStatus, 1000);
            } else {
              updateSystemMessage("Errore: " + payload.message, true);
            }
          } catch (err) {
            updateSystemMessage("Errore: " + err.message, true);
          }
        });
      }

      if (switchControllerBtn) {
        switchControllerBtn.addEventListener("click", async () => {
          updateSystemMessage("Switch controller...");
          try {
            const response = await fetch("/api/system/status");
            const payload = await response.json();
            const currentController = payload.data?.controller?.name;
            const useScaled = currentController !== "scaled_joint_trajectory_controller";
            
            const switchResponse = await fetch("/api/system/switch_controller", {
              method: "POST",
              headers: {"Content-Type": "application/json"},
              body: JSON.stringify({use_scaled: useScaled})
            });
            const switchPayload = await switchResponse.json();
            
            if (switchPayload.status === "ok") {
              updateSystemMessage(switchPayload.message);
              setTimeout(fetchSystemStatus, 1000);
            } else {
              updateSystemMessage("Errore: " + switchPayload.message, true);
            }
          } catch (err) {
            updateSystemMessage("Errore: " + err.message, true);
          }
        });
      }

      if (refreshSystemStatusBtn) {
        refreshSystemStatusBtn.addEventListener("click", () => fetchSystemStatus(true));
      }

      if (ros2DriverStatus) {
        fetchSystemStatus();
        setInterval(fetchSystemStatus, 3000);
      }

      function updateJoint(targetName, delta) {
        const input = form.querySelector('input[name="' + targetName + '"]');
        if (!input) return;
        const current = parseFloat(input.value) || 0;
        const next = current + delta;
        input.value = next.toFixed(3);
      }

      function handleArrowButtons(event) {
        const button = event.target.closest("button[data-target]");
        if (!button) return;
        event.preventDefault();
        if (!stepInput) return;
        const step = parseFloat(stepInput.value) || 0.1;
        const isIncrement = button.classList.contains("joint-increment");
        const delta = isIncrement ? step : -step;
        updateJoint(button.dataset.target, delta);
      }

      const jointButtons = document.querySelectorAll(".joint-increment, .joint-decrement");
      if (jointButtons && jointButtons.length > 0) {
        jointButtons.forEach((btn) => {
          btn.addEventListener("click", handleArrowButtons);
        });
      }

      if (jointInputs && jointInputs.length > 0) {
        jointInputs.forEach((input) => {
          input.addEventListener("focus", (event) => event.target.select());
        });
      }

      // --- Joystick logic -------------------------------------------------
      const joystick = document.getElementById("joystick");
      const handle = document.getElementById("joystick-handle");
      const joyX = document.getElementById("joy-x");
      const joyY = document.getElementById("joy-y");
      const stopJoystickBtn = document.getElementById("stop-joystick");

      const JOY_MAX_BASE = 0.05;       // max joint velocity base (rad/s) - controllabile con slider
      const JOY_CART_VEL_BASE = 0.05;  // max cartesian velocity base (m/s) - controllabile con slider
      const JOY_DEADZONE = 0.10;       // deadzone ridotta per maggiore sensibilità
      const MAX_SPEED_MULTIPLIER = 2.0; // Moltiplicatore massimo per velocità (200% = 0.10 rad/s max)
      
      // Velocità corrente (modificata dallo slider)
      let currentSpeedMultiplier = 0.5; // 50% di default - controllabile con slider (10%-200%)
      let joystickActive = false;
      let joyVector = { x: 0, y: 0 };
      
      // Slider velocità
      const speedSlider = document.getElementById("speed-slider");
      const speedInput = document.getElementById("speed-input");
      const frequencySlider = document.getElementById("frequency-slider");
      const frequencyInput = document.getElementById("frequency-input");
      
      // Sincronizzazione Speed Slider <-> Input
      if (speedSlider && speedInput) {
        speedSlider.addEventListener("input", () => {
          speedInput.value = speedSlider.value;
          updateSpeedMultiplier();
        });
        speedInput.addEventListener("input", () => {
          let val = parseInt(speedInput.value) || 1;
          val = Math.max(1, Math.min(10000, val)); // Limite ragionevole ma alto
          speedInput.value = val;
          speedSlider.value = Math.min(val, 1000); // Slider max 1000, input può essere più alto
          updateSpeedMultiplier();
        });
      }
      
      // Sincronizzazione Frequency Slider <-> Input
      if (frequencySlider && frequencyInput) {
        frequencySlider.addEventListener("input", () => {
          frequencyInput.value = frequencySlider.value;
          updateFrequency();
        });
        frequencyInput.addEventListener("input", () => {
          let val = parseInt(frequencyInput.value) || 10;
          val = Math.max(10, Math.min(1000, val)); // Limite ragionevole
          frequencyInput.value = val;
          frequencySlider.value = Math.min(val, 500); // Slider max 500, input può essere più alto
          updateFrequency();
        });
      }
      
      function updateSpeedMultiplier() {
        const sliderValue = parseInt(speedInput ? speedInput.value : (speedSlider ? speedSlider.value : 100));
        currentSpeedMultiplier = (sliderValue / 100.0) * MAX_SPEED_MULTIPLIER;
        if (speedInput) {
          speedInput.value = sliderValue;
        }
        if (speedSlider && sliderValue <= 1000) {
          speedSlider.value = sliderValue;
        }
      }
      
      if (speedSlider) {
        speedSlider.addEventListener("input", updateSpeedMultiplier);
      }
      updateSpeedMultiplier(); // Inizializza
      
      // Slider frequenza
      function updateFrequency() {
        currentFrequency = parseInt(frequencyInput ? frequencyInput.value : (frequencySlider ? frequencySlider.value : 125));
        if (frequencyInput) {
          frequencyInput.value = currentFrequency;
        }
        if (frequencySlider && currentFrequency <= 500) {
          frequencySlider.value = currentFrequency;
        }
        // Aggiorna frequenza nel bridge ROS2
        fetch("/api/bridge/set_frequency", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({frequency: currentFrequency})
        }).catch(err => console.error("Errore aggiornamento frequenza:", err));
      }
      
      if (frequencySlider) {
        frequencySlider.addEventListener("input", updateFrequency);
      }
      updateFrequency(); // Inizializza
      
      // Tracker Fluidità Movimento
      let commandCount = 0;
      let lastCommandTime = 0; // Usato sia per tracker fluidità che per dead man's switch
      let commandIntervals = [];
      const maxIntervals = 50;
      const fluidityBar = document.getElementById("fluidity-bar");
      const fluidityValue = document.getElementById("fluidity-value");
      const commandCountEl = document.getElementById("command-count");
      const lastUpdateEl = document.getElementById("last-update");
      
      function updateFluidityTracker() {
        commandCount++;
        const now = Date.now();
        
        if (lastCommandTime > 0) {
          const interval = now - lastCommandTime;
          commandIntervals.push(interval);
          if (commandIntervals.length > maxIntervals) {
            commandIntervals.shift();
          }
          
          // Calcola fluidità (inverso della varianza degli intervalli)
          if (commandIntervals.length > 5) {
            const avg = commandIntervals.reduce((a, b) => a + b, 0) / commandIntervals.length;
            const variance = commandIntervals.reduce((sum, val) => sum + Math.pow(val - avg, 2), 0) / commandIntervals.length;
            const stdDev = Math.sqrt(variance);
            const expectedInterval = 1000 / currentFrequency;
            const fluidity = Math.max(0, Math.min(100, 100 - (stdDev / expectedInterval) * 50));
            
            if (fluidityBar) {
              fluidityBar.style.width = fluidity + '%';
            }
            if (fluidityValue) {
              fluidityValue.textContent = fluidity.toFixed(0) + '%';
              fluidityValue.style.color = fluidity > 80 ? '#00aa00' : fluidity > 50 ? '#ff8800' : '#cc0000';
            }
          }
        }
        
        lastCommandTime = now;
        
        if (commandCountEl) {
          commandCountEl.textContent = commandCount;
        }
        if (lastUpdateEl) {
          lastUpdateEl.textContent = new Date().toLocaleTimeString();
        }
      }
      
      // Verifica processi doppi
      const checkProcessesBtn = document.getElementById("check-processes");
      const processStatus = document.getElementById("process-status");
      
      if (checkProcessesBtn) {
        checkProcessesBtn.addEventListener("click", async () => {
          processStatus.style.display = "block";
          processStatus.textContent = "Verifica processi...";
          try {
            const response = await fetch("/api/system/check_processes", {
              method: "POST",
              headers: {"Content-Type": "application/json"},
              body: JSON.stringify({kill_duplicates: true})
            });
            const payload = await response.json();
            if (payload.status === "ok") {
              const data = payload.data;
              let msg = "";
              if (data.duplicates_found) {
                msg = '<span class="material-icons md-18">warning</span> Processi doppi trovati e killati: ' + data.killed.join(", ");
                processStatus.style.background = "#fff3cd";
                processStatus.style.color = "#856404";
              } else {
                msg = '<span class="material-icons md-18">check_circle</span> Nessun processo doppio trovato';
                processStatus.style.background = "#d4edda";
                processStatus.style.color = "#155724";
              }
              processStatus.textContent = msg;
              setTimeout(() => fetchSystemStatus(), 1000);
            }
          } catch (err) {
            processStatus.textContent = "Errore verifica processi: " + err.message;
            processStatus.style.background = "#f8d7da";
            processStatus.style.color = "#721c24";
          }
        });
      }

      // Riavvia Web Interface
      const restartWebInterfaceBtn = document.getElementById("restart-web-interface");
      if (restartWebInterfaceBtn) {
        restartWebInterfaceBtn.addEventListener("click", async () => {
          if (!confirm('Sei sicuro di voler riavviare il web interface? La pagina si ricaricherà automaticamente.')) {
            return;
          }
          
          restartWebInterfaceBtn.disabled = true;
          restartWebInterfaceBtn.innerHTML = '<span class=' + '"material-icons md-18"' + '>refresh</span> Riavvio in corso...';
          
          try {
            const response = await fetch("/api/system/restart_web_interface", {
              method: "POST",
              headers: {"Content-Type": "application/json"}
            });
            const payload = await response.json();
            
            if (payload.status === "ok") {
              // Attendi 2 secondi poi ricarica la pagina
              setTimeout(() => {
                window.location.reload();
              }, 2000);
            } else {
              alert('Errore riavvio: ' + payload.message);
              restartWebInterfaceBtn.disabled = false;
              restartWebInterfaceBtn.innerHTML = '<span class="material-icons md-18">refresh</span> Riavvia Web Interface';
            }
          } catch (err) {
            alert('Errore: ' + err.message);
            restartWebInterfaceBtn.disabled = false;
            restartWebInterfaceBtn.innerHTML = '<span class="material-icons md-18">refresh</span> Riavvia Web Interface';
          }
        });
      }

      function clamp(value, min, max) {
        return Math.min(Math.max(value, min), max);
      }

      function setHandlePosition(xNorm, yNorm) {
        const radius = joystick.clientWidth / 2 - handle.clientWidth / 2;
        const x = radius * xNorm;
        const y = radius * yNorm;
        handle.style.transform = 'translate(calc(-50% + ' + x + 'px), calc(-50% + ' + y + 'px))';
        joyX.textContent = xNorm.toFixed(2);
        joyY.textContent = (-yNorm).toFixed(2);
      }

      function resetJoystick() {
        joyVector = { x: 0, y: 0 };
        setHandlePosition(0, 0);
        // Update speeds (bridge publishes continuously at 125Hz)
        updateSpeeds();
      }

      // --- Logging System Functions ---
      function initLogging() {
        // Verifica che le variabili siano inizializzate
        if (typeof lastLogId === 'undefined') {
          console.error('<span class="material-icons md-18">error</span> Variabili logging non inizializzate!');
          return;
        }
        
        const logsContainer = document.getElementById('logs-container');
        const clearLogsBtn = document.getElementById('clear-logs');
        const toggleLogsBtn = document.getElementById('toggle-logs');
        
        if (!logsContainer) return;

        // Pulisci log
        if (clearLogsBtn) {
          clearLogsBtn.addEventListener('click', async () => {
            try {
              const response = await fetch('/api/logs/clear', { method: 'POST' });
              const data = await response.json();
              if (data.status === 'ok') {
                logsContainer.innerHTML = '<div style="color: #888; font-style: italic;">Log puliti...</div>';
                lastLogId = -1;
                updateLogCounts();
              }
            } catch (err) {
              console.error('Errore pulizia log:', err);
            }
          });
        }

        // Pausa/Riprendi log
        if (toggleLogsBtn) {
          toggleLogsBtn.addEventListener('click', () => {
            logsPaused = !logsPaused;
            if (logsPaused) {
              toggleLogsBtn.innerHTML = '<span class=' + '"material-icons md-18"' + '>play_arrow</span> Riprendi';
            } else {
              toggleLogsBtn.innerHTML = '<span class=' + '"material-icons md-18"' + '>pause</span> Pausa';
            }
          });
        }

        // Aggiorna log ogni 500ms
        if (typeof logUpdateInterval !== 'undefined') {
          logUpdateInterval = setInterval(updateLogs, 500);
          updateLogs(); // Prima chiamata immediata
        } else {
          console.error('<span class="material-icons md-18">error</span> logUpdateInterval non definito!');
        }
      }

      async function updateLogs() {
        if (typeof logsPaused === 'undefined' || logsPaused) return;
        
        try {
          const response = await fetch('/api/logs?last_id=' + lastLogId);
          const data = await response.json();
          
          if (data.status === 'ok' && data.logs && data.logs.length > 0) {
            const logsContainer = document.getElementById('logs-container');
            if (!logsContainer) return;

            // Rimuovi messaggio "In attesa..."
            if (logsContainer.children.length === 1 && logsContainer.children[0].textContent.includes('In attesa')) {
              logsContainer.innerHTML = '';
            }

            data.logs.forEach(log => {
              const logLine = document.createElement('div');
              logLine.style.marginBottom = '2px';
              logLine.style.padding = '2px 4px';
              
              // Colori in base al livello
              let color = '#d4d4d4';
              let bgColor = 'transparent';
              if (log.level === 'ERROR') {
                color = '#f48771';
                bgColor = 'rgba(244, 135, 113, 0.1)';
              } else if (log.level === 'WARNING') {
                color = '#dcdcaa';
                bgColor = 'rgba(220, 220, 170, 0.1)';
              } else if (log.level === 'INFO') {
                color = '#4ec9b0';
              }
              
              logLine.style.color = color;
              if (bgColor !== 'transparent') {
                logLine.style.backgroundColor = bgColor;
              }
              
              // Formatta messaggio
              const message = log.message.replace(/<span class="material-icons md-18">warning<\\/span>/g, '[WARN]').replace(/<span class="material-icons md-18">check_circle<\\/span>/g, '[OK]').replace(/<span class="material-icons md-18">send<\\/span>/g, '[SEND]').replace(/<span class="material-icons md-18">hourglass_empty<\\/span>/g, '[WAIT]');
              logLine.innerHTML = '<span style="color: #888;">[' + log.timestamp + ']</span> <span style="color: #569cd6;">[' + log.level + ']</span> ' + message;
              
              logsContainer.appendChild(logLine);
            });

            // Mantieni solo ultimi 500 log visibili
            while (logsContainer.children.length > 500) {
              logsContainer.removeChild(logsContainer.firstChild);
            }

            // Scroll automatico in fondo
            logsContainer.scrollTop = logsContainer.scrollHeight;
            
            lastLogId = data.last_id;
            updateLogCounts();
          }
        } catch (err) {
          console.error('Errore aggiornamento log:', err);
        }
      }

      function updateLogCounts() {
        const logCountEl = document.getElementById('log-count');
        const logErrorsEl = document.getElementById('log-errors');
        const logWarningsEl = document.getElementById('log-warnings');
        
        if (!logCountEl) return;

        fetch('/api/logs')
          .then(r => r.json())
          .then(data => {
            if (data.status === 'ok' && data.logs) {
              const errors = data.logs.filter(l => l.level === 'ERROR').length;
              const warnings = data.logs.filter(l => l.level === 'WARNING').length;
              
              if (logCountEl) logCountEl.textContent = data.total || 0;
              if (logErrorsEl) logErrorsEl.textContent = errors;
              if (logWarningsEl) logWarningsEl.textContent = warnings;
            }
          })
          .catch(err => console.error('Errore conteggio log:', err));
      }

      // APPROCCIO DIRETTO: Invia comando immediatamente quando joystick si muove (come commit funzionante)
      // Non usare throttling complesso - invia direttamente come nel commit 31521b63
      // lastCommandTime è già dichiarato sopra per il tracker fluidità
      const COMMAND_TIMEOUT = 100; // 100ms timeout: se non arrivano comandi, ferma il robot (DEAD MAN'S SWITCH)
      
      function updateSpeeds() {
        // Calculate speeds from joystick positions
        const magnitude = Math.hypot(joyVector.x, joyVector.y);
        const magnitude2 = Math.hypot(joy2Vector.x, joy2Vector.y);
        
        let speeds;
        if (magnitude < JOY_DEADZONE && magnitude2 < JOY_DEADZONE) {
          speeds = [0, 0, 0, 0, 0, 0];
        } else {
          const cartesianModeEl = document.getElementById("cartesian-mode");
          const cartesianMode = cartesianModeEl ? cartesianModeEl.checked : false;
          const JOY_MAX = JOY_MAX_BASE * currentSpeedMultiplier;
          const JOY_CART_VEL = JOY_CART_VEL_BASE * currentSpeedMultiplier;
          
          if (cartesianMode) {
            speeds = [
              joyVector.y * JOY_CART_VEL,      // X
              joyVector.x * JOY_CART_VEL,      // Y
              -joy2Vector.y * JOY_CART_VEL,    // Z
              0,                                // Rx
              0,                                // Ry
              joy2Vector.x * JOY_CART_VEL * 0.5  // Rz (rotazione più lenta)
            ];
          } else {
            // Joint mode: tutti gli assi interpolati e funzionanti
            // Joystick 1: controlla base (joint 0, 1) - XY
            // Joystick 2: controlla elbow (joint 2) e wrist (joint 3, 4, 5) - Z e rotazioni
            speeds = [
              joyVector.y * JOY_MAX,                    // Joint 0 (shoulder_pan) - rotazione base X
              joyVector.x * JOY_MAX,                    // Joint 1 (shoulder_lift) - sollevamento Y
              -joy2Vector.y * JOY_MAX,                  // Joint 2 (elbow) - Z axis (su/giù)
              joy2Vector.x * JOY_MAX * 0.8,             // Joint 3 (wrist_1) - rotazione wrist X
              (joyVector.y + joy2Vector.y) * JOY_MAX * 0.3,  // Joint 4 (wrist_2) - combinazione per movimento fluido
              joy2Vector.x * JOY_MAX * 0.6              // Joint 5 (wrist_3) - rotazione finale Z
            ];
          }
        }
        
        // Aggiorna timestamp ultimo comando
        lastCommandTime = Date.now();
        
        // INVIA IMMEDIATAMENTE (come nel commit funzionante) - senza throttling
        const cartesianModeEl = document.getElementById("cartesian-mode");
        const cartesianMode = cartesianModeEl ? cartesianModeEl.checked : false;
        
        // Log per debug (solo se c'è movimento)
        const maxSpeed = Math.max(...speeds.map(Math.abs));
        if (maxSpeed > 0.001 && Math.random() < 0.1) { // Log solo 10% delle volte per ridurre spam
          console.log('[JOYSTICK] speeds=[' + speeds.map(s => s.toFixed(4)).join(', ') + '], max=' + maxSpeed.toFixed(4));
        }
        
        // Update speeds IMMEDIATAMENTE (bridge publishes continuously at 125Hz)
        fetch("/api/servo_loop_update", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ speeds, cartesian: cartesianMode }),
        })
        .then(response => {
          if (!response.ok) {
            console.error('[ERROR] Update error: ' + response.status + ' ' + response.statusText);
            return response.text().then(text => {
              console.error('[ERROR] Error response: ' + text);
              setStatus('Errore invio comando: ' + response.status + ' - ' + text, false);
            });
          } else {
            updateFluidityTracker(); // Aggiorna tracker fluidità
            return response.json();
          }
        })
        .then(data => {
          if (data && data.status === 'error') {
            console.error('[ERROR] Server error: ' + data.message);
            setStatus('Errore server: ' + data.message, false);
          }
        })
        .catch(err => {
          console.error('[ERROR] Update error:', err);
          setStatus('Errore: ' + err.message, false);
        });
      }
      
      // DEAD MAN'S SWITCH: Se non arrivano comandi per COMMAND_TIMEOUT ms, ferma il robot
      setInterval(() => {
        const now = Date.now();
        const timeSinceLastCommand = now - lastCommandTime;
        
        // Se è passato troppo tempo dall'ultimo comando, ferma il robot
        if (timeSinceLastCommand > COMMAND_TIMEOUT && lastCommandTime > 0) {
          // Forza invio velocità zero IMMEDIATAMENTE
          joyVector = { x: 0, y: 0 };
          joy2Vector = { x: 0, y: 0 };
          lastCommandTime = Date.now();
          // Invia zero direttamente
          fetch("/api/servo_loop_update", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ speeds: [0, 0, 0, 0, 0, 0], cartesian: false }),
          }).catch(() => {}); // Ignora errori nel timeout
          // Reset timestamp per evitare spam
          lastCommandTime = 0;
        }
      }, 50); // Controlla ogni 50ms

      async function stopJointMotion() {
        joyX.textContent = "0.00";
        joyY.textContent = "0.00";
        joyVector = { x: 0, y: 0 };
        joy2Vector = { x: 0, y: 0 };
        resetJoystick();
        resetJoystick2();
        try {
          await fetch("/api/stop", { method: "POST" });
          setStatus("Stop command sent", true);
        } catch (err) {
          setStatus("Stop error: " + err, false);
        }
      }

      function onJoystickStart(evt) {
        joystickActive = true;
        onJoystickMove(evt);
      }

      function onJoystickMove(evt) {
        if (!joystickActive) return;
        const rect = joystick.getBoundingClientRect();
        const point = evt.changedTouches ? evt.changedTouches[0] : evt;
        const x = point.clientX - rect.left - rect.width / 2;
        const y = point.clientY - rect.top - rect.height / 2;
        const radius = rect.width / 2 - handle.clientWidth / 2;
        const length = Math.hypot(x, y);
        const clampedLength = clamp(length, 0, radius);
        const angle = Math.atan2(y, x);
        const norm = clampedLength / radius;

        const xNorm = Math.cos(angle) * norm;
        const yNorm = Math.sin(angle) * norm;
        joyVector = { x: xNorm, y: yNorm };
        setHandlePosition(xNorm, yNorm);
        
        // Update speeds immediately (bridge publishes continuously at 125Hz)
        updateSpeeds();
      }

      function onJoystickEnd() {
        joystickActive = false;
        // IMPORTANTE: Quando rilasci il joystick, invia immediatamente velocità zero
        joyVector = { x: 0, y: 0 };
        resetJoystick();
        // Invia zero immediatamente (come nel commit funzionante)
        lastCommandTime = Date.now();
        updateSpeeds(); // Chiama updateSpeeds che invierà zero
      }

      joystick.addEventListener("mousedown", onJoystickStart);
      window.addEventListener("mousemove", (evt) => {
        if (!joystickActive) return;
        onJoystickMove(evt);
      });
      window.addEventListener("mouseup", onJoystickEnd);

      joystick.addEventListener("touchstart", (evt) => {
        evt.preventDefault();
        onJoystickStart(evt);
      }, { passive: false });
      joystick.addEventListener("touchmove", (evt) => {
        evt.preventDefault();
        onJoystickMove(evt);
      }, { passive: false });
      joystick.addEventListener("touchend", onJoystickEnd);
      joystick.addEventListener("touchcancel", onJoystickEnd);

      stopJoystickBtn.addEventListener("click", () => {
        stopJointMotion();
        resetJoystick();
        resetJoystick2();
      });

      // --- Second Joystick (Z + Rotation) ------------------------------------
      const joystick2 = document.getElementById("joystick2");
      const handle2 = document.getElementById("joystick2-handle");
      const joy2X = document.getElementById("joy2-x");
      const joy2Y = document.getElementById("joy2-y");

      let joystick2Active = false;
      let joy2Vector = { x: 0, y: 0 };

      function setHandle2Position(xNorm, yNorm) {
        const radius = joystick2.clientWidth / 2 - handle2.clientWidth / 2;
        const x = radius * xNorm;
        const y = radius * yNorm;
        handle2.style.transform = 'translate(calc(-50% + ' + x + 'px), calc(-50% + ' + y + 'px))';
        joy2X.textContent = xNorm.toFixed(2);
        joy2Y.textContent = (-yNorm).toFixed(2);
      }

      function resetJoystick2() {
        joy2Vector = { x: 0, y: 0 };
        setHandle2Position(0, 0);
        // Update speeds (bridge publishes continuously at 125Hz)
        updateSpeeds();
      }

      function onJoystick2Start(evt) {
        joystick2Active = true;
        onJoystick2Move(evt);
      }

      function onJoystick2Move(evt) {
        if (!joystick2Active) return;
        const rect = joystick2.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        let clientX, clientY;
        if (evt.touches) {
          clientX = evt.touches[0].clientX;
          clientY = evt.touches[0].clientY;
        } else {
          clientX = evt.clientX;
          clientY = evt.clientY;
        }
        const dx = clientX - cx;
        const dy = clientY - cy;
        const maxDist = rect.width / 2;
        const dist = Math.hypot(dx, dy);
        const limitedDist = Math.min(dist, maxDist);
        const angle = Math.atan2(dy, dx);
        const xNorm = (limitedDist / maxDist) * Math.cos(angle);
        const yNorm = (limitedDist / maxDist) * Math.sin(angle);
        joy2Vector = { x: clamp(xNorm, -1, 1), y: clamp(yNorm, -1, 1) };
        setHandle2Position(joy2Vector.x, joy2Vector.y);
        
        // Update speeds immediately (bridge publishes continuously at 125Hz)
        updateSpeeds();
      }

      function onJoystick2End() {
        joystick2Active = false;
        // IMPORTANTE: Quando rilasci il joystick, invia immediatamente velocità zero
        joy2Vector = { x: 0, y: 0 };
        resetJoystick2();
        // Invia zero immediatamente (come nel commit funzionante)
        lastCommandTime = Date.now();
        updateSpeeds(); // Chiama updateSpeeds che invierà zero
      }

      joystick2.addEventListener("mousedown", onJoystick2Start);
      window.addEventListener("mousemove", (evt) => {
        if (!joystick2Active) return;
        onJoystick2Move(evt);
      });
      window.addEventListener("mouseup", onJoystick2End);
      joystick2.addEventListener("touchstart", onJoystick2Start);
      joystick2.addEventListener("touchmove", onJoystick2Move);
      joystick2.addEventListener("touchend", onJoystick2End);
      joystick2.addEventListener("touchcancel", onJoystick2End);
      
      // --- Vision & MoveIt System ---
      let orbbecStatusInterval = null;
      let moveitStatusInterval = null;
      let detectionsInterval = null;
      
      // Orbbec Camera Controls
      const startOrbbecBtn = document.getElementById("start-orbbec");
      const stopOrbbecBtn = document.getElementById("stop-orbbec");
      const orbbecStatusValue = document.getElementById("orbbec-status-value");
      const orbbecTopicsCount = document.getElementById("orbbec-topics-count");
      const orbbecFps = document.getElementById("orbbec-fps");
      
      async function fetchOrbbecStatus() {
        try {
          const response = await fetch("/api/vision/orbbec_status");
          const data = await response.json();
          if (data.status === "ok") {
            const status = data.data;
            if (orbbecStatusValue) {
              if (status.active) {
                orbbecStatusValue.textContent = "Active";
                orbbecStatusValue.className = "mdc-chip mdc-chip--success";
              } else {
                orbbecStatusValue.textContent = "Inactive";
                orbbecStatusValue.className = "mdc-chip mdc-chip--error";
              }
            }
            if (orbbecTopicsCount) {
              orbbecTopicsCount.textContent = status.topics_count || 0;
            }
            if (orbbecFps) {
              orbbecFps.textContent = status.fps ? status.fps.toFixed(1) : "—";
            }
          }
        } catch (err) {
          console.error("Orbbec status error:", err);
        }
      }
      
      if (startOrbbecBtn) {
        startOrbbecBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/start_orbbec", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("Orbbec camera started", "success", 2000);
              setTimeout(fetchOrbbecStatus, 1000);
            } else {
              showToast("Error starting camera: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      if (stopOrbbecBtn) {
        stopOrbbecBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/stop_orbbec", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("Orbbec camera stopped", "success", 2000);
              setTimeout(fetchOrbbecStatus, 1000);
            } else {
              showToast("Error stopping camera: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      // MoveIt Controls
      const startMoveitBtn = document.getElementById("start-moveit");
      const stopMoveitBtn = document.getElementById("stop-moveit");
      const testMoveitBtn = document.getElementById("test-moveit");
      const planMoveBtn = document.getElementById("plan-move");
      const moveitStatusValue = document.getElementById("moveit-status-value");
      const moveitLastPlan = document.getElementById("moveit-last-plan");
      
      async function fetchMoveitStatus() {
        try {
          // Controlla status generale vision (include MoveIt)
          const statusResponse = await fetch("/api/vision/status");
          const statusData = await statusResponse.json();
          
          // Controlla anche status MoveIt specifico
          const response = await fetch("/api/vision/moveit_status");
          const data = await response.json();
          
          if (data.status === "ok") {
            const status = data.data;
            const moveitRunning = statusData.status === "ok" && statusData.data && statusData.data.moveit && statusData.data.moveit.running;
            
            if (moveitStatusValue) {
              if (moveitRunning) {
                moveitStatusValue.textContent = "Running";
                moveitStatusValue.className = "mdc-chip mdc-chip--success";
              } else if (status.available) {
                moveitStatusValue.textContent = "Available";
                moveitStatusValue.className = "mdc-chip";
              } else {
                moveitStatusValue.textContent = "Not Available";
                moveitStatusValue.className = "mdc-chip mdc-chip--error";
              }
            }
            if (moveitLastPlan) {
              moveitLastPlan.textContent = status.last_plan_time ? new Date(status.last_plan_time * 1000).toLocaleTimeString() : "—";
            }
          }
        } catch (err) {
          console.error("MoveIt status error:", err);
        }
      }
      
      if (startMoveitBtn) {
        startMoveitBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/start_moveit", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("MoveIt started", "success", 2000);
              setTimeout(fetchMoveitStatus, 2000);
            } else {
              showToast("Error starting MoveIt: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      if (stopMoveitBtn) {
        stopMoveitBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/stop_moveit", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("MoveIt stopped", "success", 2000);
              setTimeout(fetchMoveitStatus, 1000);
            } else {
              showToast("Error stopping MoveIt: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      if (testMoveitBtn) {
        testMoveitBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/test_moveit", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("MoveIt test successful", "success", 2000);
              setTimeout(fetchMoveitStatus, 1000);
            } else {
              showToast("MoveIt test failed: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      // Camera Stream
      const cameraStreamImg = document.getElementById("camera-stream");
      const cameraStreamStatus = document.getElementById("camera-stream-status");
      if (cameraStreamImg) {
        // Aggiorna stream ogni 100ms (10 FPS per ridurre carico)
        cameraStreamImg.onerror = () => {
          if (cameraStreamStatus) {
            cameraStreamStatus.style.display = "block";
            cameraStreamStatus.textContent = "Camera stream not available. Start camera first.";
          }
        };
        cameraStreamImg.onload = () => {
          if (cameraStreamStatus) {
            cameraStreamStatus.style.display = "none";
          }
        };
        // Refresh stream periodicamente
        setInterval(() => {
          if (cameraStreamImg) {
            const timestamp = new Date().getTime();
            cameraStreamImg.src = '/api/vision/camera_stream?t=' + timestamp;
          }
        }, 100); // 10 FPS
      }
      
      // MoveIt Execute Plan
      const executeMoveitBtn = document.getElementById("execute-moveit-plan");
      const moveitPlanStatus = document.getElementById("moveit-plan-status");
      if (executeMoveitBtn) {
        executeMoveitBtn.addEventListener("click", async () => {
          const x = parseFloat(document.getElementById("moveit-x")?.value || 0.3);
          const y = parseFloat(document.getElementById("moveit-y")?.value || 0.0);
          const z = parseFloat(document.getElementById("moveit-z")?.value || 0.3);
          const roll = parseFloat(document.getElementById("moveit-roll")?.value || 0.0);
          const pitch = parseFloat(document.getElementById("moveit-pitch")?.value || 0.0);
          const yaw = parseFloat(document.getElementById("moveit-yaw")?.value || 0.0);
          
          if (moveitPlanStatus) {
            moveitPlanStatus.style.display = "block";
            moveitPlanStatus.textContent = "Planning movement...";
            moveitPlanStatus.style.background = "rgba(255, 193, 7, 0.1)";
            moveitPlanStatus.style.color = "#856404";
          }
          
          executeMoveitBtn.disabled = true;
          
          try {
            const response = await fetch("/api/vision/plan_move", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({
                target_pose: { x, y, z, roll, pitch, yaw },
                execute: true
              })
            });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("Motion plan executed successfully", "success", 3000);
              if (moveitPlanStatus) {
                moveitPlanStatus.textContent = "✓ Plan executed: " + (data.message || "Success");
                moveitPlanStatus.style.background = "rgba(0, 200, 83, 0.1)";
                moveitPlanStatus.style.color = "#2e7d32";
              }
              setTimeout(fetchMoveitStatus, 1000);
            } else {
              showToast("Planning failed: " + data.message, "error", 4000);
              if (moveitPlanStatus) {
                moveitPlanStatus.textContent = "✗ Error: " + data.message;
                moveitPlanStatus.style.background = "rgba(211, 47, 47, 0.1)";
                moveitPlanStatus.style.color = "#c62828";
              }
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
            if (moveitPlanStatus) {
              moveitPlanStatus.textContent = "✗ Error: " + err.message;
              moveitPlanStatus.style.background = "rgba(211, 47, 47, 0.1)";
              moveitPlanStatus.style.color = "#c62828";
            }
          } finally {
            executeMoveitBtn.disabled = false;
          }
        });
      }
      
      if (planMoveBtn) {
        planMoveBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/plan_move", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("Motion plan created", "success", 2000);
              setTimeout(fetchMoveitStatus, 1000);
            } else {
              showToast("Planning failed: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      // Vision System Controls
      const startYoloBtn = document.getElementById("start-yolo");
      const stopYoloBtn = document.getElementById("stop-yolo");
      const startVisionBtn = document.getElementById("start-vision");
      const stopVisionBtn = document.getElementById("stop-vision");
      const detectionsContainer = document.getElementById("detections-container");
      
      if (startYoloBtn) {
        startYoloBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/start_yolo", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("YOLO detector started", "success", 2000);
              if (!detectionsInterval) {
                detectionsInterval = setInterval(fetchDetections, 2000);
              }
            } else {
              showToast("Error starting YOLO: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      if (stopYoloBtn) {
        stopYoloBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/stop_yolo", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("YOLO detector stopped", "success", 2000);
              if (detectionsInterval) {
                clearInterval(detectionsInterval);
                detectionsInterval = null;
              }
            } else {
              showToast("Error stopping YOLO: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      let selectedDetectionIndex = -1;
      
      function selectDetection(idx) {
        selectedDetectionIndex = idx;
        // Evidenzia detection selezionata
        document.querySelectorAll('[id^="detection-"]').forEach(el => {
          el.style.background = 'rgba(0, 0, 0, 0.06)';
        });
        const selected = document.getElementById('detection-' + idx);
        if (selected) {
          selected.style.background = 'rgba(25, 118, 210, 0.2)';
        }
      }
      
      // Rendi selectDetection globale
      window.selectDetection = selectDetection;
      
      async function fetchDetections() {
        try {
          const response = await fetch("/api/vision/detections");
          const data = await response.json();
          if (data.status === "ok" && data.detections && data.detections.length > 0) {
            if (detectionsContainer) {
              detectionsContainer.innerHTML = data.detections.map((det, idx) => {
                const pos = det.position || {};
                const className = (det.class_name || 'unknown').replace(/'/g, "&apos;").replace(/"/g, "&quot;");
                return '<div style="padding: 8px; margin-bottom: 4px; background: rgba(0, 0, 0, 0.06); border-radius: var(--mdc-shape-small); cursor: pointer;" onclick="selectDetection(' + idx + ')" id="detection-' + idx + '">' +
                  '<strong>' + className + '</strong> (' + (det.confidence * 100).toFixed(1) + '%)<br>' +
                  '<small>Position: [' + (pos.x || 0).toFixed(3) + ', ' + (pos.y || 0).toFixed(3) + ', ' + (pos.z || 0).toFixed(3) + '] m</small>' +
                  '</div>';
              }).join("");
            }
          } else if (detectionsContainer && (!data.detections || data.detections.length === 0)) {
            detectionsContainer.innerHTML = '<div style="color: rgba(0, 0, 0, 0.5); font-style: italic;">No detections yet. Start camera and vision system.</div>';
          }
        } catch (err) {
          console.error("Detections fetch error:", err);
        }
      }
      
      if (startVisionBtn) {
        startVisionBtn.addEventListener("click", async () => {
          try {
            // Chiama start_yolo invece di start (per compatibilità)
            const response = await fetch("/api/vision/start_yolo", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("YOLO detector started", "success", 2000);
              if (!detectionsInterval) {
                detectionsInterval = setInterval(fetchDetections, 2000);
              }
            } else {
              showToast("Error starting YOLO: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      if (stopVisionBtn) {
        stopVisionBtn.addEventListener("click", async () => {
          try {
            const response = await fetch("/api/vision/stop_yolo", { method: "POST" });
            const data = await response.json();
            if (data.status === "ok") {
              showToast("YOLO detector stopped", "success", 2000);
              if (detectionsInterval) {
                clearInterval(detectionsInterval);
                detectionsInterval = null;
              }
            } else {
              showToast("Error stopping YOLO: " + data.message, "error", 4000);
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
          }
        });
      }
      
      // Avvia polling status
      if (orbbecStatusValue) {
        fetchOrbbecStatus();
        orbbecStatusInterval = setInterval(fetchOrbbecStatus, 3000);
      }
      
      if (moveitStatusValue) {
        fetchMoveitStatus();
        moveitStatusInterval = setInterval(fetchMoveitStatus, 5000);
      }
      
      if (detectionsContainer) {
        detectionsInterval = setInterval(fetchDetections, 2000);
      }
      
      // Approach Object Button
      const approachBtn = document.getElementById("approach-object");
      const approachStatus = document.getElementById("approach-status");
      if (approachBtn) {
        approachBtn.addEventListener("click", async () => {
          approachBtn.disabled = true;
          if (approachStatus) {
            approachStatus.style.display = "block";
            approachStatus.textContent = "Calcolo posizione target...";
            approachStatus.style.background = "rgba(255, 193, 7, 0.1)";
            approachStatus.style.color = "#856404";
          }
          
          try {
            const response = await fetch("/api/vision/approach_object", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({
                distance: 0.20,  // 20cm
                selected_index: selectedDetectionIndex >= 0 ? selectedDetectionIndex : null
              })
            });
            
            const data = await response.json();
            if (data.status === "ok") {
              showToast("Robot si sta avvicinando al pezzo", "success", 3000);
              if (approachStatus) {
                approachStatus.textContent = "✓ " + (data.message || "Movimento in corso...");
                approachStatus.style.background = "rgba(0, 200, 83, 0.1)";
                approachStatus.style.color = "#2e7d32";
              }
            } else {
              showToast("Errore: " + data.message, "error", 4000);
              if (approachStatus) {
                approachStatus.textContent = "✗ Errore: " + data.message;
                approachStatus.style.background = "rgba(211, 47, 47, 0.1)";
                approachStatus.style.color = "#c62828";
              }
            }
          } catch (err) {
            showToast("Error: " + err.message, "error", 4000);
            if (approachStatus) {
              approachStatus.textContent = "✗ Errore: " + err.message;
              approachStatus.style.background = "rgba(211, 47, 47, 0.1)";
              approachStatus.style.color = "#c62828";
            }
          } finally {
            approachBtn.disabled = false;
          }
        });
      }
      
      // INIZIALIZZAZIONE AL CARICAMENTO DELLA PAGINA
      // Assicurati che tutto sia inizializzato quando il DOM è pronto
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function() {
          console.log('[INIT] DOM caricato, inizializzo wizard...');
          try {
            initWizardEventListeners();
            console.log('[INIT] Wizard inizializzato con successo');
          } catch (err) {
            console.error('[INIT ERROR] Errore inizializzazione wizard:', err);
            if (typeof showToast === 'function') {
              showToast('Errore inizializzazione: ' + err.message, 'error', 10000);
            }
          }
        });
      } else {
        // DOM già caricato, inizializza subito
        console.log('[INIT] DOM già caricato, inizializzo wizard immediatamente...');
        try {
          initWizardEventListeners();
          console.log('[INIT] Wizard inizializzato con successo');
        } catch (err) {
          console.error('[INIT ERROR] Errore inizializzazione wizard:', err);
          if (typeof showToast === 'function') {
            showToast('Errore inizializzazione: ' + err.message, 'error', 10000);
          }
        }
      }
    
    </script>
  </body>
</html>
"""


def load_config() -> ControllerConfig:
    """Carica configurazione robot. Non logga warning per UR_ROBOT_IP non impostata."""
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")  # Default senza warning
    port = int(os.environ.get("UR_ROBOT_PORT", 30002))
    return ControllerConfig(robot_ip=robot_ip, port=port)


_controller = None

def get_controller():
    """Ottiene il controller UR (singleton)."""
    global _controller
    if not _controller:
        config = load_config()
        _controller = RemoteURController(config.robot_ip, config.port)
    return _controller


def parse_joints(payload) -> List[float]:
    """
    Accept either a mapping {joint0: val, ...} or a flat sequence of 6 elements.
    """
    if payload is None:
        raise ValueError("Missing joint payload")

    if isinstance(payload, (list, tuple)):
        if len(payload) != 6:
            raise ValueError("Joint sequence must have 6 elements")
        return [float(value) for value in payload]

    joints = []
    for idx in range(6):
        key = f"joint{idx}"
        if key not in payload:
            raise ValueError(f"Missing {key}")
        joints.append(float(payload[key]))
    return joints


@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route("/api/movej", methods=["POST"])
def api_movej():
    payload = request.get_json(force=True)
    joints = parse_joints(payload)
    params = MoveParameters(
        acceleration=float(payload.get("acceleration", 1.2)),
        velocity=float(payload.get("velocity", 0.25)),
        blend_radius=float(payload.get("blend_radius", 0.0)),
        async_move=bool(payload.get("async_move", False)),
    )

    controller = get_controller()
    controller.movej(joints, params=params)
    return jsonify({"status": "ok", "message": "MoveJ command sent"})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    controller = get_controller()
    controller.stop()
    return jsonify({"status": "ok", "message": "Stop command sent"})


@app.route("/api/speedj", methods=["POST"])
def api_speedj():
    payload = request.get_json(force=True)
    controller = get_controller()
    speeds = parse_joints(payload.get("speeds"))
    controller.speedj(
        speeds,
        duration=float(payload.get("duration", 0.3)),
        acceleration=float(payload.get("acceleration", 1.0)),
    )
    return jsonify({"status": "ok", "message": "SpeedJ command sent"})


@app.route("/api/speedl", methods=["POST"])
def api_speedl():
    """Cartesian velocity control endpoint."""
    payload = request.get_json(force=True)
    controller = get_controller()
    speeds = parse_joints(payload.get("speeds"))  # [vx,vy,vz,wx,wy,wz]
    controller.speedl(
        speeds,
        duration=float(payload.get("duration", 0.3)),
        acceleration=float(payload.get("acceleration", 0.5)),
    )
    return jsonify({"status": "ok", "message": "SpeedL command sent"})


@app.route("/api/movel_relative", methods=["POST"])
def api_movel_relative():
    """Cartesian incremental movement endpoint."""
    payload = request.get_json(force=True)
    controller = get_controller()
    delta = parse_joints(payload.get("delta"))  # [dx,dy,dz,drx,dry,drz]
    controller.movel_relative(
        delta,
        acceleration=float(payload.get("acceleration", 0.3)),
        velocity=float(payload.get("velocity", 0.03)),
        blend=float(payload.get("blend", 0.01)),
    )
    return jsonify({"status": "ok", "message": "MoveL relative command sent"})


@app.route("/api/system/ros2_bridge_status", methods=["GET"])
def api_ros2_bridge_status():
    """Verifica stato ROS2 bridge."""
    bridge = get_ros2_bridge()
    if bridge:
        try:
            is_initialized = bridge.ensure_ros()
            is_running = getattr(bridge, '_running', False)
            thread_alive = getattr(bridge, '_publish_thread', None) and bridge._publish_thread.is_alive()
            last_error = getattr(bridge, '_last_error', None)
            
            return jsonify({
                "status": "ok",
                "data": {
                    "initialized": is_initialized,
                    "running": is_running,
                    "thread_alive": thread_alive,
                    "last_error": last_error
                },
                "message": "ROS2 bridge ready - publishing at 125Hz" if is_initialized and is_running else "ROS2 bridge not ready"
            })
        except Exception as e:
            app.logger.error(f"Error checking ROS2 bridge status: {e}")
            return jsonify({
                "status": "error",
                "data": {
                    "initialized": False,
                    "running": False,
                    "thread_alive": False,
                    "last_error": str(e)
                },
                "message": f"Error checking bridge: {e}"
            })
    return jsonify({
        "status": "ok",
        "data": {
            "initialized": False,
            "running": False,
            "thread_alive": False,
            "last_error": "Bridge not available"
        },
        "message": "Using socket fallback"
    })


@app.route("/api/servo_loop_start", methods=["POST"])
def api_servo_loop_start():
    """Inizializza ROS2 bridge (publishing starts automatically at 125Hz)."""
    if ROS2_AVAILABLE:
        bridge = get_ros2_bridge()
        if bridge and bridge.ensure_ros():
            return jsonify({"status": "ok", "message": "ROS2 bridge ready - publishing at 125Hz"})
    return jsonify({"status": "ok", "message": "Using socket fallback"})


# Traccia ultimo comando ricevuto per timeout automatico (DEAD MAN'S SWITCH)
_last_command_time = None
_last_timeout_warning_time = None
_COMMAND_TIMEOUT_SEC = 0.3  # 300ms timeout backend (doppio livello di sicurezza)
_TIMEOUT_WARNING_INTERVAL = 10.0  # Log warning solo ogni 10 secondi per evitare spam

def _check_command_timeout():
    """Verifica se è passato troppo tempo dall'ultimo comando. Se sì, ferma il robot."""
    global _last_command_time, _last_timeout_warning_time
    import time

    if _last_command_time is None:
        return False

    time_since_last = time.time() - _last_command_time
    if time_since_last > _COMMAND_TIMEOUT_SEC:
        # Timeout: ferma il robot
        try:
            if ROS2_AVAILABLE:
                bridge = get_ros2_bridge()
                if bridge and bridge.ensure_ros():
                    bridge.publish_speedj([0, 0, 0, 0, 0, 0])
                    # Log warning solo ogni 10 secondi per evitare spam nei log
                    current_time = time.time()
                    if _last_timeout_warning_time is None or (current_time - _last_timeout_warning_time) >= _TIMEOUT_WARNING_INTERVAL:
                        app.logger.warning(f"[TIMEOUT] Nessun comando per {time_since_last:.3f}s - robot fermato automaticamente")
                        _last_timeout_warning_time = current_time
        except Exception as e:
            app.logger.error(f"[ERROR] Errore timeout stop: {e}")
        return True
    return False

def _timeout_monitor_thread():
    """Thread che monitora il timeout dei comandi e ferma il robot se necessario."""
    import time
    import threading

    while True:
        try:
            time.sleep(0.05)  # Controlla ogni 50ms
            _check_command_timeout()
        except Exception as e:
            app.logger.error(f"[ERROR] Errore monitor timeout: {e}")
            time.sleep(1)  # In caso di errore, aspetta 1 secondo prima di riprovare

@app.route("/api/servo_loop_update", methods=["POST"])
def api_servo_loop_update():
    """Aggiorna velocità via socket diretto (più affidabile di ROS2)."""
    global _last_command_time
    
    try:
        payload = request.get_json(force=True)
        speeds = parse_joints(payload.get("speeds"))
        cartesian_mode = payload.get("cartesian", False)
        
        # Aggiorna timestamp ultimo comando
        import time
        _last_command_time = time.time()
        
        # Verifica timeout: se non arrivano comandi da troppo tempo, forza velocità zero
        if _check_command_timeout():
            speeds = [0, 0, 0, 0, 0, 0]
            app.logger.warning(f"[TIMEOUT] Nessun comando per {_COMMAND_TIMEOUT_SEC}s - fermo robot")
        
        # ROS2 (SOLUZIONE PRINCIPALE - secondo documentazione ufficiale)
        # Richiede driver UR ROS2 in esecuzione e robot configurato con External Control URCap
        if ROS2_AVAILABLE:
            bridge = get_ros2_bridge()
            if bridge:
                if bridge.ensure_ros():
                    # Se il bridge ROS2 è attivo, usa quello (gestisce già la sicurezza)
                    max_speed = max(abs(s) for s in speeds) if speeds else 0.0
                    
                    # Se il bridge ROS2 è attivo, usa quello (gestisce già la sicurezza)
                    if bridge.publish_speedj(speeds):
                        max_speed = max(abs(s) for s in speeds)
                        # Log solo se velocità significativa (riduce spam log)
                        if max_speed > 0.01:
                            app.logger.debug(f"[OK] Comando ROS2: speeds={[f'{s:.4f}' for s in speeds]}, max={max_speed:.4f}")
                        return jsonify({"status": "ok", "message": f"ROS2 speedj (max speed: {max_speed:.4f} rad/s)"})
                    else:
                        # Log solo una volta, non ad ogni richiesta
                        if not hasattr(api_servo_loop_update, '_publish_failed_warned'):
                            app.logger.debug("[ERROR] Bridge ROS2: publish_speedj fallito")
                            api_servo_loop_update._publish_failed_warned = True
                else:
                    app.logger.debug("Bridge ROS2: ensure_ros() fallito")
            else:
                app.logger.debug("Bridge ROS2: get_ros2_bridge() restituito None")
        
        # FALLBACK ROS2: Se ROS2 non disponibile nel processo Flask, usa ros2 topic pub via subprocess
        # Questo funziona anche se rclpy non può essere importato nel processo Flask
        try:
            import subprocess
            # Verifica se il topic esiste (driver ROS2 attivo) - solo una volta
            if not hasattr(api_servo_loop_update, '_ros2_topic_checked'):
                check_result = subprocess.run(
                    ['bash', '-c', 'source /opt/ros/humble/setup.bash 2>/dev/null && timeout 1 ros2 topic list 2>/dev/null | grep -q forward_velocity_controller/commands'],
                    capture_output=True,
                    timeout=2
                )
                api_servo_loop_update._ros2_topic_available = (check_result.returncode == 0)
                api_servo_loop_update._ros2_topic_checked = True
            
            if getattr(api_servo_loop_update, '_ros2_topic_available', False):
                # Topic esiste - pubblica via ros2 topic pub
                # Formato: std_msgs/msg/Float64MultiArray con campo data come array YAML
                speeds_str = '[' + ','.join(str(s) for s in speeds) + ']'
                # Usa formato YAML: data: [val1, val2, ...]
                yaml_msg = f"data: {speeds_str}"
                cmd = f'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{yaml_msg}" 2>/dev/null'
                pub_result = subprocess.run(
                    ['bash', '-c', cmd],
                    capture_output=True,
                    timeout=1
                )
                if pub_result.returncode == 0:
                    max_speed = max(abs(s) for s in speeds) if speeds else 0.0
                    if max_speed > 0.01:
                        app.logger.debug(f"[OK] ROS2 (subprocess): speeds={[f'{s:.4f}' for s in speeds]}, max={max_speed:.4f}")
                    return jsonify({"status": "ok", "message": f"ROS2 speedj via subprocess (max speed: {max_speed:.4f} rad/s)"})
        except Exception as subprocess_err:
            # Se subprocess fallisce, continua con fallback socket
            pass
        
        # Verifica stato robot solo se ROS2 non è disponibile (fallback socket)
        # Rendiamo i controlli più flessibili per evitare falsi negativi
        try:
            config = load_config()
            dashboard = DashboardClient(config.robot_ip)
            program_state = dashboard.get_program_state()
            robot_mode = dashboard.get_robot_mode()
            
            # Controlli flessibili: accettiamo anche stati parziali o "unknown"
            # Solo se abbiamo informazioni certe che il robot NON è pronto, blocchiamo
            if program_state and program_state != "unknown" and "PLAYING" not in program_state and "STOPPED" not in program_state:
                # Se è in STOPPED, potrebbe essere normale in remote control
                if "STOPPED" not in program_state:
                    return jsonify({
                        "status": "error", 
                        "message": f"Robot non in PLAYING (stato: {program_state}). Avvia il programma sul teach pendant."
                    }), 400
            
            if robot_mode and robot_mode != "unknown" and "RUNNING" not in robot_mode:
                return jsonify({
                    "status": "error", 
                    "message": f"Robot non in RUNNING (stato: {robot_mode})"
                }), 400
        except Exception as check_err:
            # Se non riesce a verificare, continua comunque (potrebbe essere un problema temporaneo)
            # Log per debug ma non bloccare
            app.logger.warning(f"Errore verifica stato robot: {check_err}")
            pass
        
        # FALLBACK: Socket URScript (se ROS2 non disponibile o non configurato)
        # IMPORTANTE: Questo è il metodo che funziona quando ROS2 non è disponibile nel processo Flask
        controller = get_controller()
        try:
            max_speed = max(abs(s) for s in speeds) if speeds else 0.0
            if max_speed > 0.001:
                # Log solo se velocità significativa (riduce spam)
                app.logger.debug(f"[FALLBACK] Socket control: speeds={[f'{s:.4f}' for s in speeds]}, max={max_speed:.4f}")
            
            if cartesian_mode:
                controller.speedl(speeds, duration=0.008, acceleration=0.3)  # 125Hz = 0.008s
                return jsonify({"status": "ok", "message": f"Socket control (speedl cartesian) - max speed: {max_speed:.4f} rad/s"})
            else:
                # Usa speedj per controllo fluido real-time (125Hz)
                # speedj è il comando URScript standard per controllo velocità joint
                controller.speedj(speeds, duration=0.008, acceleration=1.0)
                return jsonify({"status": "ok", "message": f"Socket control (speedj) - max speed: {max_speed:.4f} rad/s"})
        except (socket.timeout, OSError, ConnectionError) as sock_err:
            # Log errore solo una volta
            if not hasattr(api_servo_loop_update, '_socket_error_logged'):
                app.logger.warning(f"Errore connessione socket robot: {sock_err}")
                api_servo_loop_update._socket_error_logged = True
            return jsonify({
                "status": "error", 
                "message": f"Errore connessione robot: {sock_err}. Verifica che il programma sia PLAYING sul teach pendant."
            }), 500
        
    except Exception as e:
        error_msg = str(e)
        if "PLAYING" in error_msg or "RUNNING" in error_msg:
            return jsonify({"status": "error", "message": error_msg}), 400
        return jsonify({"status": "error", "message": f"Errore: {error_msg}"}), 500


@app.route("/api/servo_loop_stop", methods=["POST"])
def api_servo_loop_stop():
    """Ferma movimento (sets velocities to zero - bridge continues publishing at 125Hz)."""
    try:
        # ROS2 - set speeds to zero (bridge continues publishing)
        if ROS2_AVAILABLE:
            bridge = get_ros2_bridge()
            if bridge:
                bridge.publish_stop()
                return jsonify({"status": "ok", "message": "ROS2 stop (velocities set to zero)"})
        
        # FALLBACK: Socket
        controller = get_controller()
        controller.stop()
        return jsonify({"status": "ok", "message": "Socket stop"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/api/servoj", methods=["POST"])
def api_servoj():
    """ServoJ endpoint per controllo fluido real-time (legacy)."""
    payload = request.get_json(force=True)
    controller = get_controller()
    speeds = parse_joints(payload.get("speeds"))
    
    controller.servoj_velocity(
        speeds,
        t=float(payload.get("duration", 0.008)),
        lookahead_time=float(payload.get("lookahead_time", 0.1)),
        gain=float(payload.get("gain", 300.0)),
    )
    return jsonify({"status": "ok", "message": "ServoJ command sent"})


@app.route("/api/status", methods=["GET"])
def api_status():
    data: Dict[str, object] = {
        "ros2_available": ROS2_AVAILABLE,
        "env": {
            "UR_ROBOT_IP": os.environ.get("UR_ROBOT_IP"),
            "WEB_HOST": os.environ.get("WEB_HOST"),
            "WEB_PORT": os.environ.get("WEB_PORT"),
        },
    }

    if ROS2_AVAILABLE:
        bridge = get_ros2_bridge()
        if bridge:
            data["ros2_bridge"] = bridge.get_status()

    return jsonify({"status": "ok", "data": data})


@app.route("/api/robot_status", methods=["GET"])
def api_robot_status():
    """Legge lo stato del robot (joints, TCP, robot mode, etc.)"""
    try:
        config = load_config()
        robot_status = {}
        
        # 1. Leggi stato Dashboard (robot mode, safety mode, etc.)
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((config.robot_ip, 29999))
            
            # Leggi welcome
            sock.recv(1024)
            
            # Comandi Dashboard
            dashboard_commands = {
                "robotmode": "robotmode",
                "safetymode": "safetymode",
                "programState": "programState",
                "remote_control": "is in remote control",
            }
            
            dashboard_status = {}
            for key, cmd in dashboard_commands.items():
                try:
                    sock.sendall((cmd + "\n").encode('utf-8'))
                    time.sleep(0.1)
                    response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
                    dashboard_status[key] = response
                except:
                    dashboard_status[key] = "unknown"
            
            sock.close()
            robot_status["dashboard"] = dashboard_status
        except Exception as e:
            robot_status["dashboard"] = {"error": str(e)}
        
        # 2. Leggi joints e TCP via RTDE
        # [IMPORTANT] RTDE può essere usato da UN SOLO processo alla volta!
        # Se il driver UR ROS2 è in esecuzione, NON usare RTDE qui (causa overflow)
        # In quel caso, usa solo Dashboard Server per stato base
        try:
            # Verifica se driver ROS2 è attivo (controlla porta 50002 o topic ROS2)
            ros2_driver_active = False
            try:
                # Prova a vedere se ci sono topic ROS2 attivi
                import subprocess
                result = subprocess.run(
                    ['timeout', '1', 'ros2', 'topic', 'list'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode == 0 and 'joint_states' in result.stdout:
                    ros2_driver_active = True
            except:
                pass
            
            if ros2_driver_active:
                # Driver ROS2 attivo → NON usare RTDE (causa overflow)
                # Usa solo Dashboard per stato base
                robot_status["rtde"] = {
                    "info": "RTDE disabilitato - driver ROS2 attivo (evita overflow)",
                    "joints": None,
                    "tcp_pose": None
                }
            else:
                # Driver ROS2 NON attivo → puoi usare RTDE
                import rtde.rtde as rtde_lib
                rtde_client = rtde_lib.RTDE(config.robot_ip, 30004)
                rtde_client.connect()
                
                # Setup output - non passare types (usa default della libreria)
                rtde_client.send_output_setup(['actual_q', 'actual_TCP_pose'], frequency=10)
                rtde_client.send_start()
                
                # Ricevi dati
                state = rtde_client.receive()
                
                if state:
                    joints = None
                    tcp_pose = None
                    
                    if hasattr(state, 'actual_q') and state.actual_q is not None:
                        try:
                            joints = [round(float(j), 4) for j in state.actual_q]
                        except:
                            joints = None
                    
                    # Prova sia actual_tcp_pose che actual_TCP_pose
                    tcp_attr = None
                    if hasattr(state, 'actual_tcp_pose'):
                        tcp_attr = 'actual_tcp_pose'
                    elif hasattr(state, 'actual_TCP_pose'):
                        tcp_attr = 'actual_TCP_pose'
                    
                    if tcp_attr and getattr(state, tcp_attr) is not None:
                        try:
                            tcp_pose = [round(float(p), 4) for p in getattr(state, tcp_attr)]
                        except:
                            tcp_pose = None
                    
                    robot_status["rtde"] = {
                        "joints": joints,
                        "tcp_pose": tcp_pose,
                    }
                else:
                    robot_status["rtde"] = {"error": "No data received"}
                
                rtde_client.send_pause()
                rtde_client.disconnect()
        except ImportError:
            robot_status["rtde"] = {"error": "ur_rtde not installed"}
        except Exception as e:
            robot_status["rtde"] = {"error": str(e)}
            # Log errore dettagliato per debug
            print(f"RTDE Error: {e}")
            print(traceback.format_exc())
        
        return jsonify({"status": "ok", "data": robot_status})
        
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/api/config", methods=["GET"])
def api_config():
    config = load_config()
    return jsonify({"status": "ok", "config": asdict(config)})


@app.route("/api/system/status", methods=["GET"])
def api_system_status():
    """Verifica stato sistema: driver ROS2, controller, robot, porta 50002."""
    import subprocess
    import socket
    
    status = {
        "ros2_driver": {"running": False, "pid": None},
        "controller": {"active": None, "name": None},
        "robot_mode": None,
        "robot_safety_mode": None,
        "remote_control": None,
        "program_state": None,
        "port_50002": {"listening": False},
    }
    
    # 1. Verifica driver ROS2
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'ur_ros2_control_node'],
            capture_output=True,
            text=True,
            timeout=1
        )
        if result.returncode == 0 and result.stdout.strip():
            status["ros2_driver"]["running"] = True
            status["ros2_driver"]["pid"] = result.stdout.strip().split('\n')[0]
    except:
        pass
    
    # 2. Verifica controller attivo (se ROS2 disponibile)
    if status["ros2_driver"]["running"]:
        controller_found = False
        
        # Metodo 1: Usa rclpy se disponibile
        if ROS2_AVAILABLE:
            try:
                import rclpy
                from controller_manager_msgs.srv import ListControllers
                
                if not rclpy.ok():
                    rclpy.init()
                
                node = rclpy.create_node('system_status_checker')
                client = node.create_client(ListControllers, '/controller_manager/list_controllers')
                
                if client.wait_for_service(timeout_sec=2.0):
                    req = ListControllers.Request()
                    future = client.call_async(req)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                    
                    if future.done():
                        response = future.result()
                        for controller in response.controller:
                            if 'forward_velocity_controller' in controller.name and controller.state == 'active':
                                status["controller"]["active"] = True
                                status["controller"]["name"] = "forward_velocity_controller"
                                controller_found = True
                                break
                            elif 'scaled_joint_trajectory_controller' in controller.name and controller.state == 'active':
                                status["controller"]["active"] = True
                                status["controller"]["name"] = "scaled_joint_trajectory_controller"
                                controller_found = True
                                break
                    node.destroy_node()
                    rclpy.shutdown()
            except Exception as e:
                # Se rclpy fallisce, usa metodo alternativo
                pass
        
        # Metodo 2: Fallback - usa comando ros2 service call esterno
        if not controller_found:
            try:
                cmd = "source /opt/ros/humble/setup.bash 2>/dev/null && timeout 3 ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>&1"
                result = subprocess.run(
                    ['bash', '-c', cmd],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                if result.returncode == 0:
                    output = result.stdout
                    # Cerca forward_velocity_controller attivo - vari formati possibili
                    # Il formato ROS2 può essere: state='active' o state="active" o state=active
                    # Cerca anche senza spazi: state='active'
                    import re
                    
                    # Pattern più flessibile: cerca name='forward_velocity_controller' seguito da state='active'
                    fvc_pattern = r"name\s*[=:]\s*['\"]?forward_velocity_controller['\"]?[^}]*state\s*[=:]\s*['\"]?active['\"]?"
                    sjt_pattern = r"name\s*[=:]\s*['\"]?scaled_joint_trajectory_controller['\"]?[^}]*state\s*[=:]\s*['\"]?active['\"]?"
                    
                    if re.search(fvc_pattern, output, re.IGNORECASE):
                        status["controller"]["active"] = True
                        status["controller"]["name"] = "forward_velocity_controller"
                        controller_found = True
                    elif re.search(sjt_pattern, output, re.IGNORECASE):
                        status["controller"]["active"] = True
                        status["controller"]["name"] = "scaled_joint_trajectory_controller"
                        controller_found = True
                    # Fallback: cerca semplicemente le stringhe
                    elif 'forward_velocity_controller' in output and ('active' in output.lower()):
                        # Verifica che sia effettivamente attivo (non inactive)
                        if 'inactive' not in output.lower() or output.lower().find('active') < output.lower().find('inactive'):
                            status["controller"]["active"] = True
                            status["controller"]["name"] = "forward_velocity_controller"
                            controller_found = True
                    elif 'scaled_joint_trajectory_controller' in output and ('active' in output.lower()):
                        if 'inactive' not in output.lower() or output.lower().find('active') < output.lower().find('inactive'):
                            status["controller"]["active"] = True
                            status["controller"]["name"] = "scaled_joint_trajectory_controller"
                            controller_found = True
            except Exception as e:
                # Se anche questo fallisce, lascia controller come None
                status["controller"]["error"] = f"Fallback failed: {str(e)}"
                pass
    
    # 3. Verifica porta 50002
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        result = sock.connect_ex(('127.0.0.1', 50002))
        sock.close()
        status["port_50002"]["listening"] = (result == 0)
    except:
        pass
    
    # 4. Verifica robot mode e stato (Dashboard) - con retry e gestione errori migliorata
    try:
        config = load_config()
        dashboard = DashboardClient(config.robot_ip)
        
        # Prova a connettere con timeout breve
        try:
            dashboard.connect()
        except Exception as conn_err:
            # Se non riesce a connettersi, usa valori di default
            status["robot_mode"] = "unknown"
            status["robot_safety_mode"] = "unknown"
            status["program_state"] = "unknown"
            status["remote_control"] = None
        else:
            try:
                robot_mode = dashboard.get_robot_mode()
                if robot_mode:
                    # Pulisci il risultato
                    robot_mode_clean = robot_mode.replace("Robotmode: ", "").replace("robotmode: ", "").strip()
                    status["robot_mode"] = robot_mode_clean if robot_mode_clean else "unknown"
                else:
                    status["robot_mode"] = "unknown"
            except Exception as e:
                status["robot_mode"] = "unknown"
            
            try:
                safety_mode = dashboard.get_safety_mode()
                if safety_mode:
                    safety_mode_clean = safety_mode.replace("Safetymode: ", "").replace("safetymode: ", "").strip()
                    status["robot_safety_mode"] = safety_mode_clean if safety_mode_clean else "unknown"
                else:
                    status["robot_safety_mode"] = "unknown"
            except Exception as e:
                status["robot_safety_mode"] = "unknown"
            
            try:
                program_state = dashboard.get_program_state()
                if program_state:
                    program_state_clean = program_state.replace("ProgramState: ", "").replace("programstate: ", "").strip()
                    status["program_state"] = program_state_clean if program_state_clean else "unknown"
                else:
                    status["program_state"] = "unknown"
            except Exception as e:
                status["program_state"] = "unknown"
            
            try:
                remote_control = dashboard.is_in_remote_control()
                # Converti in boolean se necessario
                if isinstance(remote_control, str):
                    status["remote_control"] = remote_control.lower() in ["true", "1", "yes"]
                elif isinstance(remote_control, bool):
                    status["remote_control"] = remote_control
                else:
                    status["remote_control"] = None
            except Exception as e:
                status["remote_control"] = None
            finally:
                try:
                    dashboard.close()
                except:
                    pass
    except Exception as e:
        # Se tutto fallisce, usa valori di default
        status["robot_mode"] = "unknown"
        status["robot_safety_mode"] = "unknown"
        status["program_state"] = "unknown"
        status["remote_control"] = None
    
    return jsonify({"status": "ok", "data": status})


@app.route("/api/system/check_robot_connection", methods=["POST"])
def api_check_robot_connection():
    """Verifica che il robot sia raggiungibile prima di avviare il driver."""
    import subprocess
    import socket
    
    config = load_config()
    robot_ip = config.robot_ip
    
    result = {
        "reachable": False,
        "ping_ok": False,
        "port_30002": False,
        "port_29999": False,
        "message": ""
    }
    
    # Test ping
    try:
        ping_result = subprocess.run(
            ['ping', '-c', '2', '-W', '2', robot_ip],
            capture_output=True,
            text=True,
            timeout=5
        )
        result["ping_ok"] = ping_result.returncode == 0
    except:
        result["ping_ok"] = False
    
    # Test porta 30002 (Primary Interface)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        port_result = sock.connect_ex((robot_ip, 30002))
        sock.close()
        result["port_30002"] = port_result == 0
    except:
        result["port_30002"] = False
    
    # Test porta 29999 (Dashboard)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        port_result = sock.connect_ex((robot_ip, 29999))
        sock.close()
        result["port_29999"] = port_result == 0
    except:
        result["port_29999"] = False
    
    # Considera raggiungibile se almeno ping o una porta funziona
    result["reachable"] = result["ping_ok"] or result["port_30002"] or result["port_29999"]
    
    if not result["reachable"]:
        result["message"] = f"Robot {robot_ip} non raggiungibile. Verifica che sia acceso e connesso."
    elif not result["port_30002"] and not result["port_29999"]:
        result["message"] = f"Robot {robot_ip} raggiungibile ma porte non aperte. Verifica configurazione robot."
    else:
        result["message"] = f"Robot {robot_ip} raggiungibile e pronto."
    
    return jsonify({"status": "ok", "data": result})


@app.route("/api/system/check_processes", methods=["POST"])
def api_check_processes():
    """Verifica e kill processi doppi."""
    import subprocess
    
    processes = {
        "ros2_driver": [],
        "web_interface": [],
        "duplicates_found": False,
        "killed": []
    }
    
    # Verifica processi driver ROS2
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'ur_ros2_control_node'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            pids = result.stdout.strip().split('\n')
            processes["ros2_driver"] = [pid for pid in pids if pid]
            if len(processes["ros2_driver"]) > 1:
                processes["duplicates_found"] = True
    except:
        pass
    
    # Verifica processi web interface
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'web_interface'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            pids = result.stdout.strip().split('\n')
            processes["web_interface"] = [pid for pid in pids if pid]
            if len(processes["web_interface"]) > 1:
                processes["duplicates_found"] = True
    except:
        pass
    
    # Kill processi doppi se richiesto
    payload = request.get_json(force=True) if request.is_json else {}
    if payload.get("kill_duplicates", False):
        # Kill driver ROS2 doppi (lascia solo il primo se c'è)
        if len(processes["ros2_driver"]) > 1:
            for pid in processes["ros2_driver"][1:]:
                try:
                    subprocess.run(['kill', '-9', pid], timeout=2, check=False)
                    processes["killed"].append(f"ros2_driver:{pid}")
                except:
                    pass
        
        # Kill web interface doppi (IMPORTANTE: kill tutti tranne questo processo stesso)
        current_pid = os.getpid()
        for pid in processes["web_interface"]:
            try:
                pid_int = int(pid)
                if pid_int != current_pid:  # NON killare questo processo!
                    subprocess.run(['kill', '-9', pid], timeout=2, check=False)
                    processes["killed"].append(f"web_interface:{pid}")
            except (ValueError, subprocess.TimeoutExpired):
                pass
        
        # Attendi che i processi vengano killati
        if processes["killed"]:
            import time
            time.sleep(1)
    
    return jsonify({"status": "ok", "data": processes})


@app.route("/api/system/start_driver", methods=["POST"])
def api_start_driver():
    """Avvia driver ROS2 in background. Verifica e kill processi doppi prima."""
    import subprocess
    import os
    
    # PRIMA: Usa script dedicato per killare tutti i processi RTDE e liberare porta 50002
    try:
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        kill_rtde_script = os.path.join(script_dir, 'kill_rtde_processes.sh')
        
        # Se lo script esiste, usalo
        if os.path.exists(kill_rtde_script):
            app.logger.info("[INFO] Esecuzione script kill_rtde_processes.sh...")
            kill_result = subprocess.run(
                ['bash', kill_rtde_script],
                capture_output=True,
                text=True,
                timeout=10
            )
            if kill_result.returncode == 0:
                app.logger.info(f"[OK] Script kill RTDE completato:\n{kill_result.stdout}")
            else:
                app.logger.warning(f"[WARN] Script kill RTDE ha avuto problemi:\n{kill_result.stderr}")
        else:
            # Fallback: kill manuale se lo script non esiste
            app.logger.warning("[WARN] Script kill_rtde_processes.sh non trovato, uso metodo manuale")
            
            # Kill driver ROS2 esistente
            result = subprocess.run(
                ['pgrep', '-f', 'ur_ros2_control_node'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                pids = [p for p in pids if p]
                if len(pids) > 0:
                    for pid in pids:
                        try:
                            subprocess.run(['kill', '-9', pid], timeout=2)
                        except:
                            pass
                    time.sleep(2)
        
        # Libera porta 50002 - trova e kill processo che la usa
        try:
            # Prova con lsof
            lsof_result = subprocess.run(
                ['lsof', '-ti', ':50002'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if lsof_result.returncode == 0 and lsof_result.stdout.strip():
                port_pids = lsof_result.stdout.strip().split('\n')
                for port_pid in port_pids:
                    if port_pid:
                        try:
                            subprocess.run(['kill', '-9', port_pid], timeout=2)
                            app.logger.info(f'🔧 Killato processo {port_pid} che usava porta 50002')
                        except:
                            pass
        except:
            # Se lsof non disponibile, prova con fuser
            try:
                fuser_result = subprocess.run(
                    ['fuser', '-k', '50002/tcp'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                app.logger.info(f'🔧 Porta 50002 liberata con fuser')
            except:
                pass
        
        # PULIZIA AGGIUNTIVA: Kill tutti i processi ROS2 che potrebbero interferire
        try:
            # Kill processi ros2 launch residui
            subprocess.run(['pkill', '-9', '-f', 'ros2.*launch.*ur_robot_driver'], timeout=2, check=False)
            # Kill processi spawner
            subprocess.run(['pkill', '-9', '-f', 'spawner.*controller'], timeout=2, check=False)
            # Kill processi controller_manager
            subprocess.run(['pkill', '-9', '-f', 'controller_manager'], timeout=2, check=False)
            app.logger.info("🔧 Pulizia processi ROS2 completata")
        except:
            pass
        
        # Attesa più lunga per pulizia completa e rilascio risorse
        time.sleep(8)  # Aumentato a 8 secondi per maggiore stabilità
    except:
        pass
    
    # Avvia driver ROS2 in background usando uno script separato
    try:
        config = load_config()
        import tempfile
        
        # Crea uno script temporaneo per avviare il driver
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        kill_rtde_script = os.path.join(script_dir, 'kill_rtde_processes.sh')
        
        script_content = f"""#!/bin/bash
# Non usare set -e per permettere gestione errori personalizzata
set -o pipefail  # Cattura errori nei pipe

# Aumenta limiti di risorse per evitare problemi
ulimit -c unlimited  # Core dumps illimitati per debug
ulimit -n 4096        # File descriptors aumentati
ulimit -s 8192        # Stack size aumentato

export HOME={os.path.expanduser('~')}
cd {os.path.expanduser('~/MekoAiAccelerator')}

# PULIZIA AGGIUNTIVA: Kill tutti i processi ROS2/RTDE prima dello script
echo "[INFO] Pulizia preliminare processi..." >> /tmp/ros2_driver.log
pkill -9 -f 'ur_ros2_control_node' 2>/dev/null || true
pkill -9 -f 'ros2.*launch.*ur_robot_driver' 2>/dev/null || true
pkill -9 -f 'spawner.*controller' 2>/dev/null || true
pkill -9 -f 'controller_manager' 2>/dev/null || true
sleep 2  # Attendi che i processi vengano killati

# PRIMA: Esegui script per killare altri processi RTDE
KILL_RTDE_SCRIPT="{kill_rtde_script}"
if [ -f "$KILL_RTDE_SCRIPT" ]; then
    echo "[INFO] Esecuzione kill_rtde_processes.sh..." >> /tmp/ros2_driver.log
    bash "$KILL_RTDE_SCRIPT" >> /tmp/ros2_driver.log 2>&1
    sleep 3  # Attesa dopo kill script
else
    echo "[WARN] Script kill_rtde_processes.sh non trovato: $KILL_RTDE_SCRIPT" >> /tmp/ros2_driver.log
fi

# Verifica che la porta 50002 sia libera
echo "[INFO] Verifica porta 50002..." >> /tmp/ros2_driver.log
if command -v lsof >/dev/null 2>&1; then
    PORT_PIDS=$(lsof -ti :50002 2>/dev/null || true)
    if [ -n "$PORT_PIDS" ]; then
        echo "[WARN] Porta 50002 ancora occupata, kill processi..." >> /tmp/ros2_driver.log
        echo "$PORT_PIDS" | xargs kill -9 2>/dev/null || true
        sleep 2
    fi
fi

# Source ROS2 con verifica errori
echo "[INFO] Source ROS2 environment..." >> /tmp/ros2_driver.log
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS2 Humble non trovato in /opt/ros/humble/" >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi
source /opt/ros/humble/setup.bash

if [ ! -f ~/ros2_ws/install/setup.bash ]; then
    echo "[WARN] Workspace ROS2 non trovato, continuo comunque..." >> /tmp/ros2_driver.log
else
    source ~/ros2_ws/install/setup.bash
fi

# SOLUZIONE RTDE OVERFLOW: Riduci update_rate da 500Hz a 30Hz (come suggerito dall'utente)
# Il file di configurazione viene caricato automaticamente dal launch file
UPDATE_RATE_FILE="$HOME/ros2_ws/install/ur_robot_driver/share/ur_robot_driver/config/ur5e_update_rate.yaml"
if [ -f "$UPDATE_RATE_FILE" ]; then
    echo "[INFO] Modifica update_rate da 500Hz a 30Hz per evitare RTDE overflow..." >> /tmp/ros2_driver.log
    cp "$UPDATE_RATE_FILE" "$UPDATE_RATE_FILE.backup" 2>/dev/null || true
    cat > "$UPDATE_RATE_FILE" << 'EOF'
controller_manager:
  ros__parameters:
    update_rate: 30  # Hz - Ridotto da 500Hz a 30Hz per evitare RTDE overflow
EOF
    echo "[OK] update_rate modificato a 30Hz" >> /tmp/ros2_driver.log
fi

# Avvia driver ROS2 in background
echo "=== AVVIO DRIVER ROS2 ===" >> /tmp/ros2_driver.log
echo "Data: $(date)" >> /tmp/ros2_driver.log
echo "Robot IP: {config.robot_ip}" >> /tmp/ros2_driver.log

# Verifica che il launch file esista
LAUNCH_FILE="/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    # Prova anche nel workspace
    LAUNCH_FILE="$HOME/ros2_ws/install/ur_robot_driver/share/ur_robot_driver/launch/ur_control.launch.py"
    if [ ! -f "$LAUNCH_FILE" ]; then
        echo "ERROR: Launch file ur_control.launch.py non trovato" >> /tmp/ros2_driver.log
        echo "ERROR"
        exit 1
    fi
fi

# Verifica che il robot sia raggiungibile PRIMA di avviare (con più tentativi)
echo "[INFO] Verifica connessione robot {config.robot_ip}..." >> /tmp/ros2_driver.log
ROBOT_REACHABLE=false
for i in 1 2 3 4 5; do
    if ping -c 2 -W 3 {config.robot_ip} > /dev/null 2>&1; then
        ROBOT_REACHABLE=true
        break
    fi
    echo "[WARN] Tentativo $i/5: robot non raggiungibile, riprovo..." >> /tmp/ros2_driver.log
    sleep 1
done

if [ "$ROBOT_REACHABLE" = "false" ]; then
    echo "ERROR: Robot {config.robot_ip} non raggiungibile dopo 5 tentativi" >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi
echo "[OK] Robot raggiungibile" >> /tmp/ros2_driver.log

# Avvia driver ROS2 con forward_velocity_controller invece di scaled_joint_trajectory_controller
# (scaled_joint_trajectory_controller causa segmentation fault)
# IMPORTANTE: usa nohup e disown per evitare che il processo venga killato quando lo script termina
echo "[INFO] Avvio ros2 launch..." >> /tmp/ros2_driver.log
nohup ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:={config.robot_ip} launch_rviz:=false initial_joint_controller:=forward_velocity_controller >> /tmp/ros2_driver.log 2>&1 &
LAUNCH_PID=$!
echo "PID launch: $LAUNCH_PID" >> /tmp/ros2_driver.log

# Verifica immediatamente che il processo sia partito (con più attesa)
echo "[INFO] Attendo avvio processo (5s)..." >> /tmp/ros2_driver.log
sleep 5  # Aumentato a 5 secondi per dare più tempo al processo di avviarsi

if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo "ERROR: Processo launch morto immediatamente (PID: $LAUNCH_PID)" >> /tmp/ros2_driver.log
    echo "[ERROR] Verifica errori nel log:" >> /tmp/ros2_driver.log
    # Cerca errori specifici nel log
    if grep -i "error\|abort\|fault\|died\|failed\|segmentation\|killed" /tmp/ros2_driver.log | tail -30 >> /tmp/ros2_driver.log 2>/dev/null; then
        echo "" >> /tmp/ros2_driver.log
    fi
    echo "[ERROR] Ultimi 150 righe del log:" >> /tmp/ros2_driver.log
    tail -150 /tmp/ros2_driver.log >> /tmp/ros2_driver.log
    echo "[ERROR] Diagnostica crash:" >> /tmp/ros2_driver.log
    echo "  - Verifica robot raggiungibile: ping -c 2 {config.robot_ip}" >> /tmp/ros2_driver.log
    echo "  - Verifica EtherNet/IP disabilitato sul robot" >> /tmp/ros2_driver.log
    echo "  - Verifica Remote Control abilitato sul robot" >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi

# Disown dopo verifica che sia vivo
disown $LAUNCH_PID 2>/dev/null || true  # Disown per evitare che venga killato quando lo script termina

# Attendi che il processo si avvii completamente (aumentato a 20s per maggiore stabilità)
echo "[INFO] Attendo inizializzazione driver (20s)..." >> /tmp/ros2_driver.log
sleep 20  # Aumentato a 20 secondi per dare più tempo all'inizializzazione

# Verifica di nuovo che il processo launch sia ancora vivo
if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo "ERROR: Processo launch morto durante inizializzazione (PID: $LAUNCH_PID)" >> /tmp/ros2_driver.log
    echo "[ERROR] Cercando errori nel log..." >> /tmp/ros2_driver.log
    if grep -i "error\|abort\|fault\|died\|failed\|segmentation\|killed" /tmp/ros2_driver.log | tail -30 >> /tmp/ros2_driver.log 2>/dev/null; then
        echo "" >> /tmp/ros2_driver.log
    fi
    echo "[ERROR] Ultimi 150 righe del log:" >> /tmp/ros2_driver.log
    tail -150 /tmp/ros2_driver.log >> /tmp/ros2_driver.log
    echo "[ERROR] Diagnostica crash:" >> /tmp/ros2_driver.log
    echo "  - Verifica robot raggiungibile: ping -c 2 {config.robot_ip}" >> /tmp/ros2_driver.log
    echo "  - Verifica EtherNet/IP disabilitato sul robot (Installation → Fieldbus)" >> /tmp/ros2_driver.log
    echo "  - Verifica Remote Control abilitato (Settings → System → Remote Control)" >> /tmp/ros2_driver.log
    echo "  - Verifica programma remote_control.urp in PLAYING sul Teach Pendant" >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi
echo "[OK] Processo launch ancora vivo (PID: $LAUNCH_PID)" >> /tmp/ros2_driver.log

# Cerca il processo ur_ros2_control_node (il processo principale del driver)
# Aspetta con più tentativi per dare tempo al processo di avviarsi
FOUND_PID=""
for i in 1 2 3 4 5 6 7 8; do
    FOUND_PID=$(pgrep -f 'ur_ros2_control_node' | head -1)
    if [ -n "$FOUND_PID" ]; then
        echo "[OK] ur_ros2_control_node trovato (PID: $FOUND_PID)" >> /tmp/ros2_driver.log
        break
    fi
    echo "[INFO] Tentativo $i/8: ur_ros2_control_node non ancora avviato, attendo..." >> /tmp/ros2_driver.log
    sleep 3
done

if [ -z "$FOUND_PID" ]; then
    echo "[ERROR] ur_ros2_control_node non trovato dopo 24 secondi" >> /tmp/ros2_driver.log
    echo "[ERROR] Verifica log per errori:" >> /tmp/ros2_driver.log
    if grep -i "error\|abort\|fault\|died\|failed\|segmentation\|killed" /tmp/ros2_driver.log | tail -30 >> /tmp/ros2_driver.log 2>/dev/null; then
        echo "" >> /tmp/ros2_driver.log
    fi
    echo "[ERROR] Possibili cause:" >> /tmp/ros2_driver.log
    echo "  1. EtherNet/IP abilitato sul robot (DISABILITALO: Installation → Fieldbus)" >> /tmp/ros2_driver.log
    echo "  2. Robot non raggiungibile o non acceso" >> /tmp/ros2_driver.log
    echo "  3. Remote Control non abilitato (Settings → System → Remote Control)" >> /tmp/ros2_driver.log
    echo "  4. Problema con configurazione ROS2" >> /tmp/ros2_driver.log
    echo "ERROR: ur_ros2_control_node non trovato" >> /tmp/ros2_driver.log
    echo "Ultimi 50 righe del log:" >> /tmp/ros2_driver.log
    tail -50 /tmp/ros2_driver.log >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi

# Verifica che il processo ur_ros2_control_node sia ancora vivo dopo 5 secondi
# (diamo più tempo perché l'inizializzazione può richiedere tempo)
echo "[INFO] Verifica stabilità ur_ros2_control_node (5s)..." >> /tmp/ros2_driver.log
sleep 5
if ! ps -p $FOUND_PID > /dev/null 2>&1; then
    echo "ERROR: ur_ros2_control_node è crashato durante l'inizializzazione (PID: $FOUND_PID)" >> /tmp/ros2_driver.log
    echo "[ERROR] Cercando errori nel log..." >> /tmp/ros2_driver.log
    if grep -i "error\|abort\|fault\|died\|failed\|segmentation\|killed" /tmp/ros2_driver.log | tail -30 >> /tmp/ros2_driver.log 2>/dev/null; then
        echo "" >> /tmp/ros2_driver.log
    fi
    echo "[ERROR] Ultimi 150 righe del log:" >> /tmp/ros2_driver.log
    tail -150 /tmp/ros2_driver.log >> /tmp/ros2_driver.log
    echo "[ERROR] Diagnostica crash:" >> /tmp/ros2_driver.log
    echo "  - Verifica robot raggiungibile: ping -c 2 {config.robot_ip}" >> /tmp/ros2_driver.log
    echo "  - Verifica EtherNet/IP DISABILITATO sul robot (Installation → Fieldbus)" >> /tmp/ros2_driver.log
    echo "  - Verifica Remote Control abilitato (Settings → System → Remote Control)" >> /tmp/ros2_driver.log
    echo "  - Verifica programma remote_control.urp in PLAYING sul Teach Pendant" >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi
echo "[OK] ur_ros2_control_node stabile (PID: $FOUND_PID)" >> /tmp/ros2_driver.log

# Se arriviamo qui, il processo è ancora vivo dopo 8 secondi - probabilmente OK
# Verifica solo errori fatali che indicano un crash definitivo
if grep -q "process has died.*exit code -[0-9]" /tmp/ros2_driver.log 2>/dev/null; then
    # Se c'è un "process has died" con exit code negativo, verifica che il processo sia ancora vivo
    # Se il processo è vivo, potrebbe essere un messaggio vecchio
    if ! ps -p $FOUND_PID > /dev/null 2>&1; then
        echo "ERROR: Processo morto rilevato nel log" >> /tmp/ros2_driver.log
        echo "ERROR"
        exit 1
    fi
fi

# Se tutto ok, restituisci il PID del processo launch (non quello del nodo figlio)
echo "[OK] Driver avviato correttamente: launch PID=$LAUNCH_PID, node PID=$FOUND_PID" >> /tmp/ros2_driver.log
echo $LAUNCH_PID
exit 0
"""
        
        # Scrivi script temporaneo
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
            f.write(script_content)
            script_path = f.name
        
        # Rendi eseguibile
        os.chmod(script_path, 0o755)
        
        try:
            # Esegui script con timeout più lungo per permettere avvio completo
            # Lo script ha: 8s pulizia + 2s porta + 5s robot check + 5s avvio processo + 20s inizializzazione = ~40s minimo
            result = subprocess.run(
                ['bash', script_path],
                capture_output=True,
                text=True,
                timeout=90,  # Timeout aumentato a 90 secondi per permettere avvio completo con tutti i controlli
                env=os.environ.copy()
            )
            
            # Rimuovi script temporaneo
            try:
                os.unlink(script_path)
            except:
                pass
            
            # Debug: mostra output completo
            app.logger.info(f"Script output: stdout='{result.stdout}', stderr='{result.stderr}', returncode={result.returncode}")
            
            # Leggi il log per verificare errori
            log_content = ""
            try:
                with open('/tmp/ros2_driver.log', 'r') as log_file:
                    log_content = log_file.read()
            except:
                pass
            
            if result.returncode != 0:
                error_msg = result.stderr or result.stdout or "Errore sconosciuto"
                
                # Verifica errori specifici nel log
                if "process has died" in log_content or "Aborted" in log_content or "Segmentation fault" in log_content or "Processo launch morto" in log_content:
                    # Analizza il tipo di crash
                    if "Processo launch morto" in log_content:
                        # Estrai solo le parti rilevanti del log (ultime 50 righe)
                        log_lines = log_content.split('\n')
                        relevant_log = '\n'.join(log_lines[-50:])
                        
                        error_msg = f"""Driver crashato durante l'avvio!

CAUSA: Il processo launch è morto immediatamente dopo l'avvio

POSSIBILI CAUSE:
1. Robot non raggiungibile o non acceso
2. Problema di configurazione ROS2
3. Conflitto con altri processi
4. Problema con il launch file

SOLUZIONI RAPIDE:
1. Verifica robot raggiungibile: ping 192.168.10.194
2. Verifica che il robot sia acceso e in Remote Control
3. Clicca "Riprova" - il sistema pulirà automaticamente e riproverà
4. Se persiste, riavvia il robot e riprova

Log rilevante:
{relevant_log}"""
                    elif "Segmentation fault" in log_content or "Aborted" in log_content:
                        # Estrai solo le parti rilevanti del log (ultime 30 righe)
                        log_lines = log_content.split('\n')
                        relevant_log = '\n'.join(log_lines[-30:])
                        
                        error_msg = f"""Driver crashato durante l'avvio!

CAUSA: Segmentation fault o Aborted nel processo ur_ros2_control_node

SOLUZIONI RAPIDE:
1. Verifica robot raggiungibile: ping 192.168.10.194
2. Clicca "Riprova" - il sistema pulirà automaticamente e riproverà
3. Se persiste, riavvia il robot e riprova

Log rilevante:
{relevant_log}"""
                    elif "process has died" in log_content:
                        log_lines = log_content.split('\n')
                        relevant_log = '\n'.join(log_lines[-20:])
                        error_msg = f"""Driver terminato durante l'avvio.

Clicca "Riprova" per riprovare automaticamente.

Log: {relevant_log}"""
                    else:
                        log_lines = log_content.split('\n')
                        relevant_log = '\n'.join(log_lines[-20:])
                        error_msg = f"Driver crashato. Log: {relevant_log}"
                else:
                    error_msg += f"\n\nLog driver:\n{log_content[-2000:]}"
                
                return jsonify({"status": "error", "message": f"Errore avvio driver: {error_msg}"})
            
            pid = result.stdout.strip()
            app.logger.info(f"PID letto dallo script: '{pid}'")
            
            if not pid or not pid.isdigit():
                # Leggi log per vedere cosa è successo
                try:
                    with open('/tmp/ros2_driver.log', 'r') as log_file:
                        log_content = log_file.read()[-2000:]
                except:
                    log_content = "Impossibile leggere log"
                
                # Verifica se il processo è comunque partito (magari il PID non è stato stampato)
                check_pgrep = subprocess.run(
                    ['pgrep', '-f', 'ur_ros2_control_node'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if check_pgrep.returncode == 0:
                    actual_pid = check_pgrep.stdout.strip().split('\n')[0]
                    app.logger.info(f"Processo trovato con pgrep: PID={actual_pid}")
                    pid = actual_pid  # Usa il PID trovato con pgrep
                else:
                    return jsonify({"status": "error", "message": f"Driver non avviato. PID non valido: '{pid}'. Output script: '{result.stdout}'. Stderr: '{result.stderr}'. Log completo (ultimi 2000 caratteri):\n{log_content[-2000:]}"})
            
            # Aspetta un po' per verificare che il processo sia ancora attivo
            import time
            time.sleep(3)
            
            # Verifica che il processo sia ancora attivo
            check_result = subprocess.run(
                ['pgrep', '-f', 'ur_ros2_control_node'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if check_result.returncode == 0 and check_result.stdout.strip():
                # Verifica anche che la porta 50002 si apra (aspetta fino a 30 secondi)
                port_open = False
                max_wait = 30  # Aspetta fino a 30 secondi
                for i in range(max_wait):
                    try:
                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.settimeout(0.5)
                        result_port = sock.connect_ex(('127.0.0.1', 50002))
                        sock.close()
                        if result_port == 0:
                            port_open = True
                            break
                    except:
                        pass
                    
                    # Controlla anche se il processo è ancora vivo
                    check_alive = subprocess.run(
                        ['pgrep', '-f', 'ur_ros2_control_node'],
                        capture_output=True,
                        text=True,
                        timeout=1
                    )
                    if check_alive.returncode != 0:
                        # Processo morto! Leggi log per vedere perché
                        try:
                            with open('/tmp/ros2_driver.log', 'r') as log_file:
                                log_content = log_file.read()[-2000:]
                        except:
                            log_content = "Impossibile leggere log"
                        return jsonify({"status": "error", "message": f"Driver ROS2 crashato dopo l'avvio! Log: {log_content[-500:]}"})
                    
                    time.sleep(1)
                
                if port_open:
                    return jsonify({"status": "ok", "message": f"Driver ROS2 avviato (PID: {pid}), porta 50002 aperta e pronta", "pid": pid})
                else:
                    # Leggi log per vedere cosa è successo
                    try:
                        with open('/tmp/ros2_driver.log', 'r') as log_file:
                            log_content = log_file.read()[-2000:]
                    except:
                        log_content = "Impossibile leggere log"
                    return jsonify({"status": "error", "message": f"Driver ROS2 avviato (PID: {pid}) ma porta 50002 non si è aperta dopo {max_wait} secondi. Verifica log: tail -50 /tmp/ros2_driver.log. Ultimi log: {log_content[-500:]}"})
            else:
                # Processo non trovato - mostra log completo
                import time
                time.sleep(2)  # Aspetta che il log venga scritto
                
                log_content = ""
                log_file_path = '/tmp/ros2_driver.log'
                
                # Prova a leggere il log più volte
                for attempt in range(3):
                    try:
                        if os.path.exists(log_file_path):
                            with open(log_file_path, 'r') as log_file:
                                log_content = log_file.read()
                            if log_content:
                                break
                        time.sleep(1)
                    except Exception as e:
                        app.logger.error(f"Errore lettura log tentativo {attempt+1}: {e}")
                        time.sleep(1)
                
                if not log_content:
                    # Se il log è vuoto, verifica se il file esiste
                    if os.path.exists(log_file_path):
                        log_content = f"File log esiste ma è vuoto. Dimensione: {os.path.getsize(log_file_path)} bytes"
                    else:
                        log_content = f"File log non esiste: {log_file_path}"
                
                # Verifica se c'è errore RTDE overflow
                rtde_overflow = "Pipeline producer overflowed" in log_content or "pipeline overflowed" in log_content.lower()
                
                # Verifica anche l'output dello script
                script_output = f"Script stdout: '{result.stdout}', stderr: '{result.stderr}', returncode: {result.returncode}"
                
                if rtde_overflow:
                    error_msg = f"""[ERROR] ERRORE RTDE OVERFLOW - Driver crashato immediatamente!

[PROBLEMA] "Pipeline producer overflowed!" - EtherNet/IP è attivo sul robot!

[SOLUZIONE]
1. Sul Teach Pendant: Installation → Fieldbus
2. DISABILITA EtherNet/IP
3. DISABILITA PROFINET
4. Riavvia robot
5. Riprova ad avviare il driver

[IMPORTANTE] EtherNet/IP e ROS2 driver NON possono essere attivi contemporaneamente!

Log completo:
{log_content[-2000:] if len(log_content) > 2000 else log_content}"""
                else:
                    error_msg = f"Driver non avviato correttamente. Processo non trovato dopo avvio.\n\n{script_output}\n\nLog driver:\n{log_content[-3000:] if len(log_content) > 3000 else log_content}"
                
                app.logger.error(f"Driver non avviato: {error_msg}")
                return jsonify({"status": "error", "message": error_msg})
                
        except subprocess.TimeoutExpired:
            try:
                os.unlink(script_path)
            except:
                pass
            return jsonify({"status": "error", "message": "Timeout avvio driver (troppo tempo)"})
            
    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        return jsonify({"status": "error", "message": f"Eccezione: {str(e)}\n{error_trace}"})


@app.route("/api/system/stop_driver", methods=["POST"])
def api_stop_driver():
    """Ferma driver ROS2."""
    import subprocess
    
    try:
        result = subprocess.run(
            ['pkill', '-f', 'ur_ros2_control_node'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return jsonify({"status": "ok", "message": "Driver ROS2 fermato"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/bridge/set_frequency", methods=["POST"])
def api_set_frequency():
    """Cambia frequenza pubblicazione del bridge ROS2."""
    if not ROS2_AVAILABLE:
        return jsonify({"status": "error", "message": "ROS2 non disponibile"})
    
    try:
        payload = request.get_json(force=True) if request.is_json else {}
        frequency = float(payload.get("frequency", 125.0))
        
        if frequency < 10.0 or frequency > 200.0:
            return jsonify({"status": "error", "message": "Frequenza deve essere tra 10 e 200 Hz"})
        
        bridge = get_ros2_bridge()
        if bridge:
            bridge.set_publish_rate(frequency)
            app.logger.info(f"Frequenza bridge impostata a {frequency} Hz")
            return jsonify({"status": "ok", "message": f"Frequenza impostata a {frequency} Hz"})
        else:
            return jsonify({"status": "error", "message": "Bridge ROS2 non disponibile"})
    except Exception as e:
        app.logger.error(f"Errore impostazione frequenza: {e}")
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/logs", methods=["GET"])
def api_logs():
    """Restituisce i log per visualizzazione in tempo reale."""
    try:
        payload = request.args
        last_id = int(payload.get("last_id", -1))
        
        with _log_lock:
            logs = list(_log_buffer)
        
        # Filtra log dopo last_id
        if last_id >= 0:
            logs = logs[last_id + 1:]
        
        return jsonify({
            "status": "ok",
            "logs": logs,
            "total": len(_log_buffer),
            "last_id": len(_log_buffer) - 1
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/logs/clear", methods=["POST"])
def api_logs_clear():
    """Pulisce i log."""
    try:
        with _log_lock:
            _log_buffer.clear()
        app.logger.info("Log puliti")
        return jsonify({"status": "ok", "message": "Log puliti"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/vision/orbbec_status", methods=["GET"])
def api_orbbec_status():
    """Restituisce lo stato della camera Orbbec."""
    import subprocess
    try:
        # Verifica se ci sono topics Orbbec attivi
        result = subprocess.run(
            ['bash', '-c', r'source /opt/ros/humble/setup.bash 2>/dev/null && timeout 2 ros2 topic list 2>/dev/null | grep -i "camera\|orbbec" | wc -l'],
            capture_output=True,
            text=True,
            timeout=5
        )
        topics_count = int(result.stdout.strip()) if result.stdout.strip().isdigit() else 0
        
        # Verifica se il processo camera è attivo
        result = subprocess.run(
            ['pgrep', '-f', 'orbbec_camera'],
            capture_output=True,
            text=True,
            timeout=3
        )
        active = result.returncode == 0
        
        # Prova a leggere FPS (se disponibile)
        fps = None
        if active:
            try:
                result = subprocess.run(
                    ['bash', '-c', 'source /opt/ros/humble/setup.bash 2>/dev/null && timeout 1 ros2 topic hz /camera/color/image_raw 2>/dev/null | head -1'],
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                if 'average rate' in result.stdout:
                    import re
                    match = re.search(r'(\d+\.?\d*)', result.stdout)
                    if match:
                        fps = float(match.group(1))
            except:
                pass
        
        return jsonify({
            "status": "ok",
            "data": {
                "active": active,
                "topics_count": topics_count,
                "fps": fps
            }
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


def _check_process_running(process_name):
    """Verifica se un processo è attivo."""
    import subprocess
    try:
        result = subprocess.run(
            ['pgrep', '-f', process_name],
            capture_output=True,
            text=True,
            timeout=2
        )
        return result.returncode == 0
    except:
        return False

def _start_process(name, command, log_file):
    """Avvia un processo in background e lo traccia."""
    global _managed_processes
    import subprocess
    import os
    
    with _processes_lock:
        # Verifica se già attivo
        if _managed_processes.get(name) is not None:
            proc = _managed_processes[name]
            if proc is not None and proc.poll() is None:
                return {"status": "already_running", "pid": proc.pid}
        
        # Kill processo esistente se presente
        if name == 'orbbec_camera':
            subprocess.run(['pkill', '-f', 'orbbec_camera'], timeout=3, check=False)
        elif name == 'yolo_detector':
            subprocess.run(['pkill', '-f', 'vision_yolo_detector'], timeout=3, check=False)
        elif name == 'moveit':
            subprocess.run(['pkill', '-f', 'moveit.launch.py'], timeout=3, check=False)
        
        time.sleep(1)
        
        # Avvia nuovo processo
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        log_path = os.path.join('/tmp', log_file)
        
        # Comando completo con source ROS2
        full_cmd = f'source /opt/ros/humble/setup.bash 2>/dev/null && '
        if os.path.exists(os.path.expanduser('~/ros2_ws/install/setup.bash')):
            full_cmd += f'source ~/ros2_ws/install/setup.bash 2>/dev/null && '
        full_cmd += f'{command} > {log_path} 2>&1'
        
        proc = subprocess.Popen(
            ['bash', '-c', full_cmd],
            shell=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        _managed_processes[name] = proc
        
        # Attendi un momento per verificare avvio
        time.sleep(2)
        
        if proc.poll() is not None:
            # Processo già terminato (errore)
            _managed_processes[name] = None
            return {"status": "error", "message": f"Process {name} terminated immediately. Check {log_path}"}
        
        return {"status": "ok", "pid": proc.pid, "log": log_path}

def _stop_process(name, process_pattern):
    """Ferma un processo gestito."""
    global _managed_processes
    import subprocess
    
    with _processes_lock:
        # Kill processo gestito
        if _managed_processes.get(name) is not None:
            proc = _managed_processes[name]
            if proc is not None:
                try:
                    proc.terminate()
                    time.sleep(1)
                    if proc.poll() is None:
                        proc.kill()
                except:
                    pass
                _managed_processes[name] = None
        
        # Kill anche per pattern (per sicurezza)
        subprocess.run(['pkill', '-f', process_pattern], timeout=5, check=False)
        time.sleep(1)
        
        return {"status": "ok"}

@app.route("/api/vision/start_orbbec", methods=["POST"])
def api_start_orbbec():
    """Avvia la camera Orbbec."""
    try:
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        result = _start_process(
            'orbbec_camera',
            'ros2 launch orbbec_camera gemini_330_series.launch.py',
            'orbbec_camera.log'
        )
        
        if result["status"] == "ok":
            return jsonify({
                "status": "ok",
                "message": f"Orbbec camera starting (PID: {result['pid']})",
                "data": {"pid": result["pid"], "log": result["log"]}
            })
        elif result["status"] == "already_running":
            return jsonify({
                "status": "ok",
                "message": f"Orbbec camera already running (PID: {result['pid']})"
            })
        else:
            return jsonify({
                "status": "error",
                "message": result.get("message", "Failed to start Orbbec camera")
            })
    except Exception as e:
        app.logger.error(f"Start Orbbec error: {e}")
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/vision/stop_orbbec", methods=["POST"])
def api_stop_orbbec():
    """Ferma la camera Orbbec."""
    try:
        result = _stop_process('orbbec_camera', 'orbbec_camera')
        return jsonify({"status": "ok", "message": "Orbbec camera stopped"})
    except Exception as e:
        app.logger.error(f"Stop Orbbec error: {e}")
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/vision/moveit_status", methods=["GET"])
def api_moveit_status():
    """Restituisce lo stato di MoveIt2."""
    import subprocess
    try:
        # Verifica se MoveIt è installato
        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 pkg list 2>/dev/null | grep -i moveit | wc -l'],
            capture_output=True,
            text=True,
            timeout=5
        )
        packages_count = int(result.stdout.strip()) if result.stdout.strip().isdigit() else 0
        available = packages_count > 0
        
        # Verifica se il planning service è disponibile
        planning_available = False
        if available:
            try:
                result = subprocess.run(
                    ['bash', '-c', r'source /opt/ros/humble/setup.bash 2>/dev/null && timeout 2 ros2 service list 2>/dev/null | grep -i "plan\|moveit" | wc -l'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                planning_available = int(result.stdout.strip()) > 0 if result.stdout.strip().isdigit() else False
            except:
                pass
        
        return jsonify({
            "status": "ok",
            "data": {
                "available": available,
                "packages_count": packages_count,
                "planning_available": planning_available,
                "last_plan_time": None  # TODO: implementare tracking
            }
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/vision/test_moveit", methods=["POST"])
def api_test_moveit():
    """Test MoveIt2 disponibilità."""
    try:
        # Verifica import Python
        try:
            from moveit_msgs.msg import Constraints
            from moveit_msgs.action import MoveGroup
            python_available = True
        except ImportError:
            python_available = False
        
        return jsonify({
            "status": "ok" if python_available else "error",
            "message": "MoveIt2 Python interface available" if python_available else "MoveIt2 Python interface not available",
            "data": {"python_available": python_available}
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/vision/camera_stream", methods=["GET"])
def api_camera_stream():
    """Stream video camera Orbbec come MJPEG."""
    # Placeholder JPEG (1x1 pixel nero)
    placeholder_jpeg = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x11\x08\x00\x01\x00\x01\x01\x01\x11\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xaa\xff\xd9'
    
    if not ROS2_AVAILABLE:
        return Response(placeholder_jpeg, mimetype='image/jpeg')
    
    # Inizializza subscriber se non già fatto
    if _camera_subscriber_node is None:
        _init_camera_subscriber()
    
    # Restituisci ultimo frame se disponibile
    with _camera_frame_lock:
        if _camera_frame:
            return Response(_camera_frame, mimetype='image/jpeg')
    
    # Nessun frame disponibile - restituisci placeholder
    return Response(placeholder_jpeg, mimetype='image/jpeg')


@app.route("/api/vision/plan_move", methods=["POST"])
def api_plan_move():
    """Pianifica ed esegue un movimento con MoveIt2."""
    if not ROS2_AVAILABLE:
        return jsonify({
            "status": "error",
            "message": "ROS2 not available"
        })
    
    try:
        payload = request.get_json(force=True) if request.is_json else {}
        target_pose = payload.get("target_pose", {})
        execute = payload.get("execute", False)
        
        x = target_pose.get("x", 0.3)
        y = target_pose.get("y", 0.0)
        z = target_pose.get("z", 0.3)
        roll = target_pose.get("roll", 0.0)
        pitch = target_pose.get("pitch", 0.0)
        yaw = target_pose.get("yaw", 0.0)
        
        # Prova a importare MoveIt
        try:
            from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
            from moveit_msgs.action import MoveGroup
            from geometry_msgs.msg import Pose, Point, Quaternion
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.node import Node
            
            # Verifica se MoveIt è disponibile
            import subprocess
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 service list 2>/dev/null | grep -i moveit | head -1'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode != 0 or not result.stdout.strip():
                return jsonify({
                    "status": "error",
                    "message": "MoveIt services not available. Make sure MoveIt is running."
                })
            
            # Implementa pianificazione ed esecuzione con MoveIt
            import math
            from builtin_interfaces.msg import Time
            import time as time_module
            
            # Converti roll, pitch, yaw in quaternion
            def euler_to_quaternion(roll, pitch, yaw):
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)
                
                qw = cr * cp * cy + sr * sp * sy
                qx = sr * cp * cy - cr * sp * sy
                qy = cr * sp * cy + sr * cp * sy
                qz = cr * cp * sy - sr * sp * cy
                return Quaternion(x=qx, y=qy, z=qz, w=qw)
            
            # Crea target pose
            target_pose_msg = Pose()
            target_pose_msg.position = Point(x=float(x), y=float(y), z=float(z))
            target_pose_msg.orientation = euler_to_quaternion(roll, pitch, yaw)
            
            # Crea goal per MoveIt
            goal_msg = MoveGroup.Goal()
            now = Time()
            now.sec = int(time_module.time())
            now.nanosec = int((time_module.time() - now.sec) * 1e9)
            goal_msg.request.workspace_parameters.header.frame_id = "base_link"
            goal_msg.request.workspace_parameters.header.stamp = now
            
            # Planning group
            goal_msg.request.group_name = "ur_manipulator"
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 5.0
            
            # Target pose
            goal_msg.request.goal_constraints = [Constraints()]
            goal_msg.request.goal_constraints[0].name = "goal"
            
            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = "tool0"
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # Bounding box per posizione (tolleranza)
            from shape_msgs.msg import SolidPrimitive
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.05, 0.05, 0.05]  # 5cm tolleranza
            pos_constraint.constraint_region.primitives = [box]
            pos_constraint.constraint_region.primitive_poses = [target_pose_msg]
            pos_constraint.weight = 1.0
            
            goal_msg.request.goal_constraints[0].position_constraints = [pos_constraint]
            
            # Orientation constraint
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = "base_link"
            orient_constraint.link_name = "tool0"
            orient_constraint.orientation = target_pose_msg.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0
            
            goal_msg.request.goal_constraints[0].orientation_constraints = [orient_constraint]
            
            # Planning options
            goal_msg.request.planning_options.plan_only = not execute
            
            # Crea action client e invia goal
            if not rclpy.ok():
                rclpy.init()
            
            node = Node('moveit_plan_client')
            action_client = ActionClient(node, MoveGroup, 'move_action')
            
            # Attendi server
            if not action_client.wait_for_server(timeout_sec=5.0):
                node.destroy_node()
                return jsonify({
                    "status": "error",
                    "message": "MoveIt action server not available. Make sure MoveIt is running: ros2 launch ur_moveit_config moveit.launch.py"
                })
            
            # Invia goal
            app.logger.info(f"[MOVEIT] Invio goal: pose=({x:.3f}, {y:.3f}, {z:.3f}), execute={execute}")
            send_goal_future = action_client.send_goal_async(goal_msg)
            
            # Attendi risultato (timeout 10s)
            rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
            
            if not send_goal_future.done():
                node.destroy_node()
                return jsonify({
                    "status": "error",
                    "message": "MoveIt planning timeout. Try again or check MoveIt configuration."
                })
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                node.destroy_node()
                return jsonify({
                    "status": "error",
                    "message": "MoveIt goal rejected. Check target pose validity and robot configuration."
                })
            
            # Attendi risultato esecuzione
            if execute:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
                
                if not result_future.done():
                    node.destroy_node()
                    return jsonify({
                        "status": "error",
                        "message": "MoveIt execution timeout. Movement may still be in progress."
                    })
                
                result = result_future.result().result
                node.destroy_node()
                
                if result.error_code.val == MoveGroup.Result.SUCCESS:
                    return jsonify({
                        "status": "ok",
                        "message": f"Movement executed successfully to pose: x={x:.3f}, y={y:.3f}, z={z:.3f}",
                        "data": {
                            "target_pose": target_pose,
                            "executed": True,
                            "error_code": result.error_code.val
                        }
                    })
                else:
                    error_names = {
                        1: "SUCCESS",
                        99999: "FAILURE",
                        -1: "PLANNING_FAILED",
                        -2: "INVALID_MOTION_PLAN",
                        -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                        -4: "CONTROL_FAILED",
                        -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                        -6: "TIMED_OUT",
                        -7: "PREEMPTED"
                    }
                    error_name = error_names.get(result.error_code.val, f"UNKNOWN_{result.error_code.val}")
                    return jsonify({
                        "status": "error",
                        "message": f"MoveIt execution failed: {error_name}",
                        "data": {
                            "target_pose": target_pose,
                            "executed": False,
                            "error_code": result.error_code.val
                        }
                    })
            else:
                # Solo planning
                node.destroy_node()
                return jsonify({
                    "status": "ok",
                    "message": f"Plan created successfully for pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}",
                    "data": {
                        "target_pose": target_pose,
                        "plan_created": True
                    }
                })
                
        except ImportError as e:
            return jsonify({
                "status": "error",
                "message": f"MoveIt Python interface not available: {e}. Install: sudo apt install ros-humble-moveit"
            })
        except Exception as e:
            app.logger.error(f"MoveIt planning error: {e}")
            return jsonify({
                "status": "error",
                "message": f"MoveIt planning error: {str(e)}"
            })
            
    except Exception as e:
        app.logger.error(f"Plan move error: {e}")
        return jsonify({
            "status": "error",
            "message": str(e)
        })


@app.route("/api/vision/detections", methods=["GET"])
def api_detections():
    """Restituisce le ultime detections dalla vision system."""
    # Inizializza subscriber se non già fatto
    if _detections_subscriber_node is None:
        _init_detections_subscriber()
    
    # Restituisci ultime detections
    with _detections_lock:
        if _latest_detections:
            # Formatta per frontend
            formatted_detections = []
            for det in _latest_detections:
                formatted_detections.append({
                    'class_name': det.get('class', 'unknown'),
                    'confidence': det.get('confidence', 0.0),
                    'position': det.get('position_3d', {}),
                    'bbox_2d': det.get('bbox_2d', {}),
                    'center_2d': det.get('center_2d', {})
                })
            return jsonify({
                "status": "ok",
                "detections": formatted_detections
            })
        else:
            return jsonify({
                "status": "ok",
                "detections": []
            })


@app.route("/api/vision/start_yolo", methods=["POST"])
def api_start_yolo():
    """Avvia YOLO detector."""
    if not ROS2_AVAILABLE:
        return jsonify({
            "status": "error",
            "message": "ROS2 not available"
        })
    
    try:
        # Verifica che la camera sia attiva
        if not _check_process_running('orbbec_camera'):
            return jsonify({
                "status": "error",
                "message": "Orbbec camera not running. Start camera first using 'Start Camera' button."
            })
        
        # Avvia YOLO detector
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        script_path = os.path.join(script_dir, "vision_yolo_detector.py")
        
        if not os.path.exists(script_path):
            return jsonify({
                "status": "error",
                "message": f"YOLO detector script not found: {script_path}"
            })
        
        result = _start_process(
            'yolo_detector',
            f'python3 {script_path}',
            'yolo_detector.log'
        )
        
        if result["status"] == "ok":
            # Inizializza subscriber detections
            _init_detections_subscriber()
            
            return jsonify({
                "status": "ok",
                "message": f"YOLO detector started (PID: {result['pid']})",
                "data": {"pid": result["pid"], "log": result["log"]}
            })
        elif result["status"] == "already_running":
            _init_detections_subscriber()
            return jsonify({
                "status": "ok",
                "message": f"YOLO detector already running (PID: {result['pid']})"
            })
        else:
            return jsonify({
                "status": "error",
                "message": result.get("message", "Failed to start YOLO detector")
            })
    except Exception as e:
        app.logger.error(f"Start YOLO error: {e}")
        return jsonify({
            "status": "error",
            "message": f"Error starting YOLO detector: {str(e)}"
        })

@app.route("/api/vision/stop_yolo", methods=["POST"])
def api_stop_yolo():
    """Ferma YOLO detector."""
    try:
        result = _stop_process('yolo_detector', 'vision_yolo_detector')
        return jsonify({"status": "ok", "message": "YOLO detector stopped"})
    except Exception as e:
        app.logger.error(f"Stop YOLO error: {e}")
        return jsonify({"status": "error", "message": str(e)})

@app.route("/api/vision/start", methods=["POST"])
def api_vision_start():
    """Avvia il sistema vision completo (YOLO + detections)."""
    # Wrapper per compatibilità - chiama start_yolo
    return api_start_yolo()


@app.route("/api/vision/stop", methods=["POST"])
def api_vision_stop():
    """Ferma il sistema vision completo."""
    # Wrapper per compatibilità - chiama stop_yolo
    return api_stop_yolo()

@app.route("/api/vision/start_moveit", methods=["POST"])
def api_start_moveit():
    """Avvia MoveIt."""
    if not ROS2_AVAILABLE:
        return jsonify({
            "status": "error",
            "message": "ROS2 not available"
        })
    
    try:
        result = _start_process(
            'moveit',
            'ros2 launch ur_moveit_config moveit.launch.py',
            'moveit.log'
        )
        
        if result["status"] == "ok":
            return jsonify({
                "status": "ok",
                "message": f"MoveIt started (PID: {result['pid']})",
                "data": {"pid": result["pid"], "log": result["log"]}
            })
        elif result["status"] == "already_running":
            return jsonify({
                "status": "ok",
                "message": f"MoveIt already running (PID: {result['pid']})"
            })
        else:
            return jsonify({
                "status": "error",
                "message": result.get("message", "Failed to start MoveIt")
            })
    except Exception as e:
        app.logger.error(f"Start MoveIt error: {e}")
        return jsonify({
            "status": "error",
            "message": f"Error starting MoveIt: {str(e)}"
        })

@app.route("/api/vision/stop_moveit", methods=["POST"])
def api_stop_moveit():
    """Ferma MoveIt."""
    try:
        result = _stop_process('moveit', 'moveit.launch.py')
        return jsonify({"status": "ok", "message": "MoveIt stopped"})
    except Exception as e:
        app.logger.error(f"Stop MoveIt error: {e}")
        return jsonify({"status": "error", "message": str(e)})

@app.route("/api/vision/status", methods=["GET"])
def api_vision_status():
    """Restituisce lo stato di tutti i componenti vision."""
    try:
        import subprocess
        
        status = {
            "orbbec_camera": {
                "running": _check_process_running('orbbec_camera'),
                "pid": None
            },
            "yolo_detector": {
                "running": _check_process_running('vision_yolo_detector'),
                "pid": None
            },
            "moveit": {
                "running": _check_process_running('moveit.launch.py'),
                "pid": None
            }
        }
        
        # Ottieni PID se attivi
        for name in ['orbbec_camera', 'vision_yolo_detector', 'moveit.launch.py']:
            try:
                result = subprocess.run(
                    ['pgrep', '-f', name],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode == 0:
                    pid = result.stdout.strip().split('\n')[0]
                    if name == 'orbbec_camera':
                        status['orbbec_camera']['pid'] = int(pid)
                    elif name == 'vision_yolo_detector':
                        status['yolo_detector']['pid'] = int(pid)
                    elif name == 'moveit.launch.py':
                        status['moveit']['pid'] = int(pid)
            except:
                pass
        
        return jsonify({
            "status": "ok",
            "data": status
        })
    except Exception as e:
        app.logger.error(f"Vision status error: {e}")
        return jsonify({
            "status": "error",
            "message": str(e)
        })


@app.route("/api/system/health_check", methods=["GET"])
def api_health_check():
    """Health check per verificare che il sistema sia funzionante."""
    import subprocess
    import os
    import time
    
    health_status = {
        "web_interface": True,
        "ros2_bridge": False,
        "ros2_driver": False,
        "port_listening": False,
        "timestamp": time.time()
    }
    
    try:
        # 1. Verifica che questo processo sia attivo
        current_pid = os.getpid()
        try:
            os.kill(current_pid, 0)  # Verifica che il processo esista
            health_status["web_interface"] = True
        except:
            health_status["web_interface"] = False
        
        # 2. Verifica ROS2 bridge
        if ROS2_AVAILABLE:
            bridge = get_ros2_bridge()
            if bridge and bridge.ensure_ros():
                health_status["ros2_bridge"] = True
        
        # 3. Verifica ROS2 driver
        try:
            result = subprocess.run(
                ['pgrep', '-f', 'ur_ros2_control_node'],
                capture_output=True,
                text=True,
                timeout=2
            )
            health_status["ros2_driver"] = (result.returncode == 0 and result.stdout.strip() != "")
        except:
            pass
        
        # 4. Verifica che la porta sia in ascolto
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            result = sock.connect_ex(('127.0.0.1', int(os.environ.get("WEB_PORT", 8080))))
            sock.close()
            health_status["port_listening"] = (result == 0)
        except:
            pass
        
        # Calcola health score
        health_score = sum([
            health_status["web_interface"],
            health_status["ros2_bridge"],
            health_status["ros2_driver"],
            health_status["port_listening"]
        ]) / 4.0
        
        return jsonify({
            "status": "ok" if health_score >= 0.5 else "degraded",
            "data": health_status,
            "health_score": health_score
        })
    except Exception as e:
        app.logger.error(f"[HEALTH] Health check error: {e}")
        return jsonify({
            "status": "error",
            "message": str(e),
            "data": health_status
        })


@app.route("/api/system/auto_restart", methods=["POST"])
def api_auto_restart():
    """Auto-restart del web interface quando viene chiamato all'accesso della pagina."""
    import subprocess
    import os
    import time
    
    try:
        app.logger.info("[AUTO-RESTART] Richiesta auto-restart web interface...")
        
        # 1. Verifica e kill processi doppi
        try:
            result = subprocess.run(
                ['pgrep', '-f', 'web_interface'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                current_pid = str(os.getpid())
                for pid in pids:
                    if pid and pid != current_pid:
                        try:
                            subprocess.run(['kill', '-9', pid], timeout=2, check=False)
                            app.logger.info(f"[AUTO-RESTART] Killed duplicate process {pid}")
                        except:
                            pass
                time.sleep(1)
        except:
            pass
        
        # 2. Verifica che il processo corrente sia ancora attivo
        current_pid = os.getpid()
        
        # 3. Crea script di riavvio robusto
        script_content = f"""#!/bin/bash
# Script auto-restart web interface
set -e

cd ~/MekoAiAccelerator || exit 1

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash 2>/dev/null || true
fi

# Export variabili
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${{LD_LIBRARY_PATH:-}}
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${{PYTHONPATH:-}}
export UR_ROBOT_IP=${{UR_ROBOT_IP:-192.168.10.194}}
export WEB_HOST=${{WEB_HOST:-0.0.0.0}}
export WEB_PORT=${{WEB_PORT:-8080}}

# Kill processo corrente se ancora attivo
sleep 1
kill -9 {current_pid} 2>/dev/null || true

# Attendi che il processo venga killato
sleep 2

# Libera porta se occupata
fuser -k ${{WEB_PORT}}/tcp 2>/dev/null || true
sleep 1

# Avvia nuovo processo
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface_auto_restart.log 2>&1 &

echo "Web interface auto-restarted (PID: $!)"
"""
        
        script_path = "/tmp/auto_restart_web_interface.sh"
        with open(script_path, 'w') as f:
            f.write(script_content)
        os.chmod(script_path, 0o755)
        
        # Esegui lo script in background
        subprocess.Popen(
            ['bash', script_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # Crea nuovo process group
        )
        
        app.logger.info(f"[AUTO-RESTART] Auto-restart script avviato (PID corrente: {current_pid})")
        return jsonify({
            "status": "ok",
            "message": f"Auto-restart avviato. Il processo verrà riavviato automaticamente."
        })
    except Exception as e:
        app.logger.error(f"[AUTO-RESTART] Errore: {e}")
        import traceback
        app.logger.error(traceback.format_exc())
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/system/restart_web_interface", methods=["POST"])
def api_restart_web_interface():
    """Riavvia il web interface killando il processo corrente."""
    import subprocess
    import os
    
    try:
        app.logger.info("[INFO] Richiesta riavvio web interface...")
        
        # Usa lo stesso meccanismo di auto-restart
        return api_auto_restart()
    except Exception as e:
        app.logger.error(f"[ERROR] Errore riavvio web interface: {e}")
        import traceback
        app.logger.error(traceback.format_exc())
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/system/switch_controller", methods=["POST"])
def api_switch_controller():
    """Switch controller usando switch_controller.py robusto (evita timeout)."""
    import subprocess
    import logging
    
    logger = logging.getLogger(__name__)
    
    try:
        payload = request.get_json(force=True) if request.is_json else {}
        use_scaled = payload.get("use_scaled", False)
        
        # Determina quale controller attivare
        if use_scaled:
            activate = 'scaled_joint_trajectory_controller'
            deactivate = 'forward_velocity_controller'
        else:
            activate = 'forward_velocity_controller'
            deactivate = 'scaled_joint_trajectory_controller'
        
        app.logger.info("[INFO] Richiesta switch controller...")
        app.logger.info(f"[INFO] Attivazione {activate}...")
        app.logger.info(f"   Deattivazione: {deactivate}")
        
        # Verifica che ROS2 sia disponibile
        if not ROS2_AVAILABLE:
            app.logger.warning("[WARN] ROS2 non disponibile - tentativo comunque...")
            # Potrebbe funzionare se lo script fa source ROS2 internamente
        
        # Usa switch_controller.py robusto invece di chiamate ROS2 dirette
        script_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "switch_controller.py")
        
        if not os.path.exists(script_path):
            error_msg = f"switch_controller.py non trovato: {script_path}"
            app.logger.error(f"[ERROR] {error_msg}")
            return jsonify({
                "status": "error",
                "message": error_msg
            }), 500
        
        # Verifica che ROS2 sia disponibile prima di eseguire
        ros2_available = ROS2_AVAILABLE
        if not ros2_available:
            app.logger.warning("[WARN] ROS2 non disponibile - tentativo comunque...")
        
        # Esegui script con timeout aumentato (controller può impiegare più tempo a configurarsi)
        # Usa bash per source ROS2 environment
        env = os.environ.copy()
        # Aggiungi ROS2 al PATH se disponibile
        if os.path.exists('/opt/ros/humble/setup.bash'):
            # Lo script switch_controller.py gestisce già il source ROS2 internamente
            pass
        
        app.logger.info(f"[INFO] Esecuzione: python3 {script_path} {activate} {deactivate}")
        app.logger.info(f"[INFO] Working directory: {os.path.dirname(script_path)}")
        app.logger.info(f"[INFO] Environment ROS2: ROS_DISTRO={env.get('ROS_DISTRO', 'NOT SET')}")
        
        try:
            result = subprocess.run(
                ["python3", script_path, activate, deactivate],
                capture_output=True,
                text=True,
                timeout=30,  # Aumentato da 15 a 30 secondi
                cwd=os.path.dirname(script_path),
                env=env
            )
        except subprocess.TimeoutExpired as timeout_err:
            app.logger.error(f"[ERROR] Script timeout dopo 30s")
            app.logger.error(f"[ERROR] Questo significa che lo script è ancora in esecuzione o bloccato")
            raise
        
        # Log output per debug COMPLETO
        app.logger.info(f"[INFO] Return code: {result.returncode}")
        if result.stdout:
            app.logger.info(f"[INFO] stdout completo ({len(result.stdout)} chars):")
            for line in result.stdout.split('\n'):
                if line.strip():
                    app.logger.info(f"   {line}")
        if result.stderr:
            app.logger.warning(f"[WARN] stderr completo ({len(result.stderr)} chars):")
            for line in result.stderr.split('\n'):
                if line.strip():
                    app.logger.warning(f"   {line}")
        
        if result.returncode == 0:
            return jsonify({
                "status": "ok",
                "message": f"Controller {activate} attivato",
                "output": result.stdout.strip()
            })
        else:
            # Se già attivo, considera successo
            output_text = result.stdout.strip() + " " + result.stderr.strip()
            if "già attivo" in output_text or "already active" in output_text.lower() or "OK:" in result.stdout:
                return jsonify({
                    "status": "ok",
                    "message": f"Controller {activate} già attivo o attivato",
                    "output": result.stdout.strip()
                })
            
            # Estrai messaggio di errore più utile
            error_msg = result.stderr.strip() or result.stdout.strip() or "Errore sconosciuto"
            # Se contiene "ERROR:", usa quello
            if "ERROR:" in error_msg:
                error_lines = error_msg.split('\n')
                for line in error_lines:
                    if "ERROR:" in line:
                        error_msg = line.replace("ERROR:", "").strip()
                        break
            
            logger.error(f"[ERROR] Switch controller fallito: {error_msg}")
            return jsonify({
                "status": "error",
                "message": f"Errore attivazione controller: {error_msg}",
                "output": result.stdout.strip(),
                "error": result.stderr.strip(),
                "returncode": result.returncode
            }), 500
            
    except subprocess.TimeoutExpired as e:
        logger.error(f"[ERROR] Timeout switch controller dopo 30s: {e}")
        return jsonify({
            "status": "error",
            "message": "Timeout switch controller (30s) - il controller potrebbe richiedere più tempo. Verifica che il driver ROS2 sia attivo e che il robot sia in Remote Control."
        }), 500
    except Exception as e:
        logger.error(f"[ERROR] Errore switch controller: {e}")
        import traceback
        error_trace = traceback.format_exc()
        logger.error(f"[ERROR] Traceback: {error_trace}")
        return jsonify({
            "status": "error",
            "message": f"Errore switch controller: {str(e)}",
            "traceback": error_trace
        }), 500


@app.route("/api/system/switch_controller_debug", methods=["POST"])
def api_switch_controller_debug():
    """Endpoint di diagnostica per switch controller - mostra output completo."""
    import subprocess
    
    try:
        payload = request.get_json(force=True) if request.is_json else {}
        use_scaled = payload.get("use_scaled", False)
        
        activate = 'forward_velocity_controller' if not use_scaled else 'scaled_joint_trajectory_controller'
        deactivate = 'scaled_joint_trajectory_controller' if not use_scaled else 'forward_velocity_controller'
        
        script_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "switch_controller.py")
        
        if not os.path.exists(script_path):
            return jsonify({
                "status": "error",
                "message": f"switch_controller.py non trovato: {script_path}"
            }), 500
        
        env = os.environ.copy()
        
        app.logger.info(f"[DEBUG] Esecuzione diagnostica: python3 {script_path} {activate} {deactivate}")
        
        # Esegui script
        result = subprocess.run(
            ["python3", script_path, activate, deactivate],
            capture_output=True,
            text=True,
            timeout=30,
            cwd=os.path.dirname(script_path),
            env=env
        )
        
        return jsonify({
            "status": "ok" if result.returncode == 0 else "error",
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "stdout_lines": result.stdout.split('\n'),
            "stderr_lines": result.stderr.split('\n'),
            "message": "OK" if result.returncode == 0 else f"Errore (code: {result.returncode})"
        })
        
    except subprocess.TimeoutExpired:
        return jsonify({
            "status": "error",
            "message": "Timeout dopo 30s",
        }), 500
    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        app.logger.error(f"[ERROR] Errore diagnostica: {e}")
        app.logger.error(f"[ERROR] Traceback: {error_trace}")
        return jsonify({
            "status": "error",
            "message": f"Errore: {str(e)}",
            "traceback": error_trace
        }), 500


def check_and_restart_ros2_driver():
    """Verifica se ROS2 driver è attivo, altrimenti lo riavvia."""
    import subprocess
    import time
    
    # 1. Verifica se ROS2 è disponibile (rclpy importabile)
    try:
        import rclpy
        ros2_available = True
    except ImportError:
        print("[WARN] ROS2 (rclpy) non disponibile - verificare installazione ROS2")
        return False
    
    # 2. Verifica se il driver ROS2 è in esecuzione
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'ur_ros2_control_node'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0 and result.stdout.strip():
            print("[OK] ROS2 driver già in esecuzione")
            return True
    except:
        pass
    
    # 3. Driver non attivo - NON riavviarlo automaticamente qui
    # L'utente deve usare il pulsante "Start Driver" nel wizard
    print("[INFO] ROS2 driver non attivo")
    print("[INFO] Usa il pulsante 'Start Driver' nello step A del wizard per avviarlo")
    return False


def main() -> None:
    """Entry point for running the Flask development server."""
    import subprocess
    
    print("=" * 80)
    print("AVVIO WEB INTERFACE")
    print("=" * 80)
    
    # 1. Verifica e riavvia ROS2 driver se necessario
    print("\n[1/3] Verifica ROS2 driver...")
    driver_ok = check_and_restart_ros2_driver()
    if not driver_ok:
        print("[WARN] ROS2 driver non disponibile - alcune funzionalità potrebbero non funzionare")
    
    # 2. Initialize ROS2 bridge (starts 125Hz publishing loop)
    print("\n[2/3] Inizializzazione ROS2 bridge...")
    if ROS2_AVAILABLE:
        try:
            bridge = get_ros2_bridge()
            if bridge and bridge.ensure_ros():
                print("[OK] ROS2 bridge initialized - publishing at 125Hz")
            else:
                print("[WARN] ROS2 bridge initialization failed - verificare ambiente ROS2")
            
            # Inizializza camera subscriber
            _init_camera_subscriber()
            # Inizializza detections subscriber
            _init_detections_subscriber()
        except Exception as e:
            print(f"[WARN] Failed to initialize ROS2 bridge: {e}")
    else:
        print("[WARN] ROS2 non disponibile (rclpy non trovato)")
        print("   Suggerimento: avvia web interface con: bash avvia_web_interface.sh")
    
    # 2.5. Avvia thread monitor timeout comandi (DEAD MAN'S SWITCH)
    print("\n[2.5/4] Avvio monitor timeout comandi...")
    timeout_thread = threading.Thread(target=_timeout_monitor_thread, daemon=True)
    timeout_thread.start()
    print("[OK] Monitor timeout comandi attivo (300ms timeout)")
    
    # 3. Verifica e libera porta se occupata + kill processi doppi
    print("\n[3/4] Verifica porta e processi...")
    host = os.environ.get("WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("WEB_PORT", 8080))
    debug = bool(int(os.environ.get("WEB_DEBUG", "0")))
    
    # Kill processi web_interface esistenti (tranne questo se già esiste)
    current_pid = os.getpid()
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'web_interface'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid and pid != str(current_pid):
                    try:
                        subprocess.run(['kill', '-9', pid], timeout=2, check=False)
                        print(f"[INFO] Processo web_interface {pid} terminato")
                    except:
                        pass
            time.sleep(1)
    except:
        pass
    
    # Verifica se la porta è occupata e kill processo
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        result = sock.connect_ex((host, port))
        sock.close()
        
        if result == 0:
            # Porta occupata - trova e kill processo
            print(f"[WARN] Porta {port} occupata, tentativo di liberarla...")
            try:
                result = subprocess.run(
                    ['lsof', '-ti', f':{port}'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode == 0 and result.stdout.strip():
                    pids = result.stdout.strip().split('\n')
                    for pid in pids:
                        if pid and pid != str(current_pid):
                            try:
                                subprocess.run(['kill', '-9', pid], timeout=2, check=False)
                                print(f"[INFO] Processo {pid} sulla porta {port} terminato")
                            except:
                                pass
                    time.sleep(1)
                    print("[OK] Porta liberata")
            except:
                # Fallback: usa fuser se lsof non disponibile
                try:
                    subprocess.run(['fuser', '-k', f'{port}/tcp'], timeout=2, check=False)
                    time.sleep(1)
                    print("[OK] Porta liberata (fuser)")
                except:
                    pass
    except:
        pass
    
    # 4. Salva PID per monitoraggio
    print("\n[4/4] Salvataggio PID per monitoraggio...")
    try:
        pid_file = "/tmp/web_interface.pid"
        with open(pid_file, 'w') as f:
            f.write(str(current_pid))
        print(f"[OK] PID salvato: {current_pid}")
    except:
        pass
    
    print(f"[INFO] Web interface disponibile su http://{host}:{port}")
    print("=" * 80)
    app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    main()


