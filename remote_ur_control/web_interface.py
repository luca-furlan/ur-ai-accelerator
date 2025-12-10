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
from collections import deque
from datetime import datetime

# Aggiungi path per ros2_bridge
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import Flask, jsonify, render_template_string, request
import socket

from .remote_ur_controller import MoveParameters, RemoteURController, DashboardClient

# ROS2 bridge (SOLUZIONE PRINCIPALE)
try:
    from ros2_bridge_fixed import ROS2Bridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    ROS2Bridge = None

app = Flask(__name__)

# ROS2 bridge singleton
_ros2_bridge = None

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
                if '⚠️' in text_stripped or 'error' in text_lower:
                    level = 'ERROR'
                elif '✅' in text_stripped or 'success' in text_lower:
                    level = 'INFO'
                elif '⏳' in text_stripped or 'waiting' in text_lower:
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
    <style>
      :root {
        color-scheme: light;
        --primary: #0066cc;
        --primary-dark: #004499;
        --accent: #00aa00;
        --accent-dark: #008800;
        --danger: #cc0000;
        --danger-dark: #990000;
        --warning: #ff8800;
        --info: #0066cc;
        --success: #00aa00;
        --border: rgba(0, 0, 0, 0.25);
        --bg: #f8f9fa;
        --bg-light: #ffffff;
        --text: #212529;
        --text-muted: #6c757d;
      }
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif;
        margin: 32px;
        max-width: 1200px;
        background: var(--bg);
        color: var(--text);
        line-height: 1.6;
      }
      h1 {
        color: var(--primary-dark);
        margin-bottom: 4px;
        font-size: 28px;
        font-weight: 700;
      }
      p.lead {
        margin-top: 0;
        color: var(--text-muted);
        font-size: 15px;
      }
      fieldset {
        border: 1px solid var(--border);
        border-radius: 8px;
        padding: 16px 20px;
        margin-top: 20px;
        background: var(--bg);
      }
      legend {
        padding: 0 8px;
        font-weight: bold;
        color: var(--primary);
      }
      label {
        display: block;
        font-weight: 600;
        margin-bottom: 4px;
      }
      input[type="number"] {
        width: 120px;
        padding: 6px;
        border-radius: 4px;
        border: 1px solid var(--border);
        font-size: 15px;
      }
      input[type="range"] {
        -webkit-appearance: none;
        appearance: none;
        width: 100%;
        height: 8px;
        border-radius: 4px;
        background: var(--bg);
        outline: none;
        opacity: 0.9;
        transition: opacity 0.2s;
      }
      input[type="range"]:hover {
        opacity: 1;
      }
      input[type="range"]::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background: var(--primary);
        cursor: pointer;
        box-shadow: 0 2px 4px rgba(0,0,0,0.2);
      }
      input[type="range"]::-moz-range-thumb {
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background: var(--primary);
        cursor: pointer;
        border: none;
        box-shadow: 0 2px 4px rgba(0,0,0,0.2);
      }
      .joint-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
        gap: 16px;
        margin-top: 12px;
      }
      .joint-card {
        background: white;
        border-radius: 8px;
        border: 1px solid var(--border);
        padding: 12px 14px;
        box-shadow: 0 1px 3px rgba(0,0,0,0.05);
      }
      .joint-card h3 {
        margin: 0 0 8px 0;
        font-size: 16px;
        color: var(--primary);
      }
      .joint-controls {
        display: flex;
        align-items: center;
        gap: 8px;
      }
      .joint-controls button {
        width: 42px;
        height: 42px;
        font-size: 20px;
        border-radius: 6px;
        border: 1px solid var(--border);
        background: white;
        cursor: pointer;
        transition: transform 0.1s ease, background 0.2s ease;
      }
      .joint-controls button:active {
        transform: translateY(1px);
      }
      .joint-controls button:hover {
        background: #e6f4ff;
        border-color: var(--primary);
      }
      .parameters {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
        gap: 12px;
      }
      .actions {
        margin-top: 24px;
        display: flex;
        gap: 12px;
        flex-wrap: wrap;
      }
      .primary-btn,
      .secondary-btn {
        padding: 12px 22px;
        border-radius: 6px;
        border: 1px solid transparent;
        font-size: 16px;
        cursor: pointer;
        font-weight: 600;
      }
      .primary-btn {
        background: var(--success);
        color: white;
        border: 2px solid var(--success);
        font-weight: 700;
        box-shadow: 0 2px 4px rgba(0,170,0,0.2);
      }
      .primary-btn:hover {
        background: var(--accent-dark);
        border-color: var(--accent-dark);
        box-shadow: 0 3px 6px rgba(0,170,0,0.3);
        transform: translateY(-1px);
      }
      .primary-btn:active {
        transform: translateY(0);
      }
      .secondary-btn {
        background: white;
        color: var(--danger);
        border: 2px solid var(--danger);
        font-weight: 700;
        box-shadow: 0 2px 4px rgba(204,0,0,0.2);
      }
      .secondary-btn:hover {
        background: #fff5f5;
        border-color: var(--danger-dark);
        box-shadow: 0 3px 6px rgba(204,0,0,0.3);
        transform: translateY(-1px);
      }
      .secondary-btn:active {
        transform: translateY(0);
      }
      .status-bar {
        margin-top: 24px;
        padding: 14px 18px;
        border-radius: 8px;
        border: 2px solid var(--border);
        background: var(--bg-light);
        font-weight: 600;
        font-size: 16px;
        box-shadow: 0 2px 4px rgba(0,0,0,0.05);
      }
      .status-bar span.ready { 
        color: var(--success);
        font-weight: 700;
      }
      .status-bar span.error { 
        color: var(--danger);
        font-weight: 700;
      }
      .status-bar span.warning {
        color: var(--warning);
        font-weight: 700;
      }
      .monitor-panel {
        margin-top: 24px;
        padding: 16px 20px;
        border: 1px solid var(--border);
        border-radius: 10px;
        background: white;
        box-shadow: 0 1px 3px rgba(0,0,0,0.06);
      }
      .monitor-header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 12px;
        gap: 12px;
        flex-wrap: wrap;
      }
      .monitor-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
        gap: 12px;
      }
      .monitor-card {
        border: 2px solid var(--border);
        border-radius: 8px;
        padding: 14px 16px;
        background: var(--bg-light);
        box-shadow: 0 1px 3px rgba(0,0,0,0.08);
      }
      .monitor-card.wide {
        grid-column: 1 / -1;
      }
      .monitor-label {
        font-size: 12px;
        text-transform: uppercase;
        letter-spacing: 0.08em;
        color: var(--text-muted);
        font-weight: 600;
        margin-bottom: 4px;
      }
      .monitor-value {
        margin-top: 8px;
        font-size: 20px;
        font-weight: 700;
        color: var(--text);
      }
      .badge {
        display: inline-flex;
        align-items: center;
        padding: 4px 10px;
        border-radius: 999px;
        font-size: 13px;
        font-weight: 600;
      }
      .badge-ok {
        background: #d4edda;
        color: #155724;
        border: 1px solid #c3e6cb;
      }
      .badge-error {
        background: #f8d7da;
        color: #721c24;
        border: 1px solid #f5c6cb;
      }
      .badge-warning {
        background: #fff3cd;
        color: #856404;
        border: 1px solid #ffeaa7;
      }
      .monitor-warning {
        margin-top: 12px;
        padding: 10px 14px;
        font-size: 14px;
        color: var(--danger);
        background: #fff5f5;
        border: 1px solid #f5c6cb;
        border-radius: 6px;
        min-height: 18px;
        font-weight: 600;
      }
      .monitor-info {
        margin-top: 12px;
        padding: 10px 14px;
        font-size: 14px;
        color: var(--info);
        background: #e7f3ff;
        border: 1px solid #b3d9ff;
        border-radius: 6px;
        min-height: 18px;
        font-weight: 500;
      }
      .monitor-topics {
        margin-top: 16px;
      }
      .topic-list {
        display: flex;
        flex-direction: column;
        border: 1px solid var(--border);
        border-radius: 8px;
        overflow: hidden;
      }
      .topic-row {
        display: flex;
        justify-content: space-between;
        padding: 8px 12px;
        border-bottom: 1px solid var(--border);
        font-size: 14px;
      }
      .topic-row:last-child {
        border-bottom: none;
      }
      .topic-row.ok {
        background: rgba(56,118,29,0.05);
      }
      .topic-row.error {
        background: rgba(153,0,0,0.04);
      }
      .monitor-json details {
        margin-top: 16px;
      }
      .monitor-json pre {
        background: #111;
        color: #0f0;
        padding: 12px;
        border-radius: 6px;
        overflow-x: auto;
        max-height: 260px;
      }
      .status-timestamp {
        font-size: 13px;
        color: #666;
      }
      .layout {
        display: grid;
        grid-template-columns: 1fr;
        gap: 12px;
      }
      @media (min-width: 768px) {
        .layout {
          grid-template-columns: repeat(2, 1fr);
        }
      }
      @media (orientation: landscape) and (max-height: 600px) {
        .layout {
          grid-template-columns: repeat(2, 1fr);
          gap: 8px;
        }
        .joystick-panel {
          padding: 12px;
        }
        #joystick, #joystick2 {
          width: 180px;
          height: 180px;
        }
      }
      .joystick-panel {
        border: 1px solid var(--border);
        border-radius: 12px;
        padding: 20px;
        background: white;
        box-shadow: inset 0 0 0 1px rgba(11, 83, 148, 0.05);
        display: flex;
        flex-direction: column;
        gap: 16px;
        align-items: center;
        justify-content: center;
      }
      .joystick-panel h2 {
        margin: 0;
        font-size: 20px;
        color: var(--primary);
      }
      #joystick, #joystick2 {
        position: relative;
        width: 220px;
        height: 220px;
        border-radius: 50%;
        background: radial-gradient(circle at center, #f5f8fd 0%, #d9e4f7 70%);
        border: 2px solid rgba(11, 83, 148, 0.25);
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
        background: rgba(11, 83, 148, 0.12);
        border: 1px solid rgba(11, 83, 148, 0.2);
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
        border: 1px solid rgba(11, 83, 148, 0.35);
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.25);
        cursor: grab;
      }
      #joystick-handle:active {
        cursor: grabbing;
      }
      .joystick-readout {
        font-family: "Courier New", monospace;
        font-size: 16px;
        color: var(--text);
        text-align: center;
        font-weight: 600;
        background: var(--bg-light);
        padding: 8px 12px;
        border-radius: 6px;
        border: 1px solid var(--border);
      }
      @media (max-width: 940px) {
        .layout {
          grid-template-columns: 1fr;
        }
        .joystick-panel {
          order: 1;
        }
      }
      @media (max-width: 480px) {
        body { margin: 16px; }
        input[type="number"] { width: 100%; }
      }
      .step-status {
        font-size: 14px;
        font-weight: 600;
        padding: 4px 12px;
        border-radius: 4px;
        background: #e9ecef;
        color: #495057;
      }
      .step-status.success {
        background: #d4edda;
        color: #155724;
      }
      .step-status.error {
        background: #f8d7da;
        color: #721c24;
      }
      .step-status.waiting {
        background: #fff3cd;
        color: #856404;
      }
      .wizard-step.active {
        opacity: 1 !important;
        border-color: #0066cc !important;
        background: #f0f7ff !important;
      }
      .wizard-step.completed {
        opacity: 1 !important;
        border-color: #00aa00 !important;
        background: #f0fff4 !important;
      }
      .wizard-step.error {
        opacity: 1 !important;
        border-color: #cc0000 !important;
        background: #fff5f5 !important;
      }
      .step-status {
        font-size: 11px !important;
        padding: 2px 6px !important;
      }
      @keyframes pulse {
        0%, 100% { box-shadow: 0 0 0 0 rgba(0, 170, 0, 0.4); }
        50% { box-shadow: 0 0 0 10px rgba(0, 170, 0, 0); }
      }
      #joystick-section {
        transition: all 0.3s ease;
      }
    </style>
  </head>
  <body>
    <h1>🤖 Controllo Robot UR5e</h1>
    <p class="lead" style="font-size: 16px; font-weight: 500;">
      Segui i passaggi qui sotto per configurare il robot. Quando tutto è pronto, usa i joystick per controllarlo.
    </p>

    <!-- Wizard Barra Orizzontale in Alto -->
    <section id="wizard-bar" style="background: linear-gradient(135deg, #fff9e6 0%, #fff3cd 100%); border: 3px solid #ffaa00; border-radius: 8px; padding: 16px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(255,170,0,0.2);">
      <div style="display: flex; align-items: center; justify-content: space-between; margin-bottom: 12px;">
        <h2 style="font-size: 20px; margin: 0;">🚀 Setup Robot</h2>
        <span style="font-size: 14px; color: #856404;">Completa i passaggi A → B → C → D → E</span>
      </div>
      
      <!-- Wizard Barra Orizzontale -->
      <div id="wizard-container" style="display: flex; gap: 8px; flex-wrap: wrap;">
        <!-- Step A: Avvia Driver -->
        <div class="wizard-step" id="step-a" style="flex: 1; min-width: 180px; padding: 12px; border: 2px solid #ddd; border-radius: 6px; background: #f9f9f9; position: relative;">
          <div style="display: flex; align-items: center; gap: 8px; margin-bottom: 8px;">
            <span style="font-size: 20px; font-weight: bold; color: #0066cc;">A</span>
            <h3 style="margin: 0; flex: 1; font-size: 14px;">Avvia Driver</h3>
            <span id="step-a-status" class="step-status" style="font-size: 12px;">⏳</span>
          </div>
          <button type="button" class="primary-btn" id="wizard-start-driver" style="width: 100%; font-size: 12px; padding: 6px;">
            ▶️ Avvia
          </button>
          <button type="button" class="secondary-btn" id="wizard-retry-a" style="width: 100%; font-size: 11px; padding: 4px; margin-top: 4px; display: none;">
            🔄 Riprova
          </button>
          <div id="step-a-message" style="margin-top: 6px; padding: 6px; background: #fff; border-radius: 4px; font-size: 11px; color: #666; min-height: 30px;"></div>
        </div>

        <!-- Step B: Verifica Driver -->
        <div class="wizard-step" id="step-b" style="flex: 1; min-width: 180px; padding: 12px; border: 2px solid #ddd; border-radius: 6px; background: #f9f9f9; opacity: 0.5; position: relative;">
          <div style="display: flex; align-items: center; gap: 8px; margin-bottom: 8px;">
            <span style="font-size: 20px; font-weight: bold; color: #0066cc;">B</span>
            <h3 style="margin: 0; flex: 1; font-size: 14px;">Verifica Driver</h3>
            <span id="step-b-status" class="step-status" style="font-size: 12px;">⏸️</span>
          </div>
          <button type="button" class="secondary-btn" id="wizard-retry-b" style="width: 100%; font-size: 11px; padding: 4px; display: none;">
            🔄 Riprova
          </button>
          <div id="step-b-message" style="margin-top: 6px; padding: 6px; background: #fff; border-radius: 4px; font-size: 11px; color: #666; min-height: 30px;"></div>
        </div>

        <!-- Step C: Attiva Controller -->
        <div class="wizard-step" id="step-c" style="flex: 1; min-width: 180px; padding: 12px; border: 2px solid #ddd; border-radius: 6px; background: #f9f9f9; opacity: 0.5; position: relative;">
          <div style="display: flex; align-items: center; gap: 8px; margin-bottom: 8px;">
            <span style="font-size: 20px; font-weight: bold; color: #0066cc;">C</span>
            <h3 style="margin: 0; flex: 1; font-size: 14px;">Attiva Controller</h3>
            <span id="step-c-status" class="step-status" style="font-size: 12px;">⏸️</span>
          </div>
          <button type="button" class="secondary-btn" id="wizard-retry-c" style="width: 100%; font-size: 11px; padding: 4px; display: none;">
            🔄 Riprova
          </button>
          <div id="step-c-message" style="margin-top: 6px; padding: 6px; background: #fff; border-radius: 4px; font-size: 11px; color: #666; min-height: 30px;"></div>
        </div>

        <!-- Step D: Attiva External Control sul Teach Pendant -->
        <div class="wizard-step" id="step-d" style="flex: 1; min-width: 200px; padding: 12px; border: 2px solid #ddd; border-radius: 6px; background: #f9f9f9; opacity: 0.5; position: relative;">
          <div style="display: flex; align-items: center; gap: 8px; margin-bottom: 8px;">
            <span style="font-size: 20px; font-weight: bold; color: #0066cc;">D</span>
            <h3 style="margin: 0; flex: 1; font-size: 14px;">Teach Pendant</h3>
            <span id="step-d-status" class="step-status" style="font-size: 12px;">⏸️</span>
          </div>
          <details style="margin: 8px 0;">
            <summary style="cursor: pointer; font-size: 11px; color: #856404; font-weight: 600;">📋 Istruzioni</summary>
            <div style="margin-top: 6px; padding: 8px; background: #fff3cd; border-radius: 4px; font-size: 11px; color: #856404;">
              <ol style="margin: 0; padding-left: 18px;">
                <li>Vai su <strong>Program</strong></li>
                <li>Apri programma con <strong>External Control</strong></li>
                <li>IP: <strong>192.168.10.191</strong>, Porta: <strong>50002</strong></li>
                <li><strong>IMPORTANTE:</strong> Attiva <strong>Remote Control</strong> sul Teach Pendant</li>
                <li><strong>SALVA</strong> e premi <strong>PLAY</strong></li>
              </ol>
            </div>
          </details>
          <button type="button" class="primary-btn" id="wizard-check-teach-pendant" style="width: 100%; font-size: 12px; padding: 6px; display: none;">
            ✅ Fatto
          </button>
          <button type="button" class="secondary-btn" id="wizard-retry-d" style="width: 100%; font-size: 11px; padding: 4px; margin-top: 4px; display: none;">
            🔄 Riprova
          </button>
          <div id="step-d-message" style="margin-top: 6px; padding: 6px; background: #fff; border-radius: 4px; font-size: 11px; color: #666; min-height: 30px;"></div>
        </div>

        <!-- Step E: Verifica Connessione -->
        <div class="wizard-step" id="step-e" style="flex: 1; min-width: 180px; padding: 12px; border: 2px solid #ddd; border-radius: 6px; background: #f9f9f9; opacity: 0.5; position: relative;">
          <div style="display: flex; align-items: center; gap: 8px; margin-bottom: 8px;">
            <span style="font-size: 20px; font-weight: bold; color: #0066cc;">E</span>
            <h3 style="margin: 0; flex: 1; font-size: 14px;">Verifica</h3>
            <span id="step-e-status" class="step-status" style="font-size: 12px;">⏸️</span>
          </div>
          <button type="button" class="secondary-btn" id="wizard-retry-e" style="width: 100%; font-size: 11px; padding: 4px; display: none;">
            🔄 Riprova
          </button>
          <div id="step-e-message" style="margin-top: 6px; padding: 6px; background: #fff; border-radius: 4px; font-size: 11px; color: #666; min-height: 30px;"></div>
        </div>
      </div>

    </section>

    <!-- Sezione Joystick - Solo quando robot è pronto -->
    <section id="joystick-section" class="monitor-panel" style="display: none; margin-top: 20px; border: 3px solid #00aa00; background: linear-gradient(135deg, #f0fff4 0%, #e8f5e9 100%);">
      <div class="monitor-header">
        <h2 style="color: #00aa00; font-size: 24px;">🎮 Controllo Robot - Pronto!</h2>
        <p style="margin: 8px 0 0 0; color: #666; font-size: 14px; font-weight: 500;">
          ✅ Robot connesso e pronto. Usa i joystick qui sotto per muovere il robot. Il robot si muoverà solo quando muovi i joystick.
        </p>
      </div>

    <div class="layout">
      <section class="joystick-panel">
        <h2>Joystick XY</h2>
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
        <h2>Joystick Z / Rotation</h2>
        <div id="joystick2">
          <div id="joystick2-base"></div>
          <div id="joystick2-handle"></div>
        </div>
        <div class="joystick-readout">
          Z: <span id="joy2-x">0.00</span> &nbsp;
          Rz: <span id="joy2-y">0.00</span>
        </div>
      </section>

      <div style="grid-column: 1 / -1; text-align: center; padding: 20px;">
        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px;">
          <div>
            <label for="speed-slider" style="display: block; margin-bottom: 8px; font-weight: 600;">
              Velocità Joystick: <input type="number" id="speed-input" value="100" min="1" step="1" style="width: 80px; padding: 4px; border: 1px solid #ddd; border-radius: 4px; font-size: 14px; margin-left: 8px;">%
            </label>
            <input type="range" id="speed-slider" min="1" max="1000" value="100" step="1" 
                   style="width: 100%; max-width: 400px; height: 8px; cursor: pointer;">
            <div style="display: flex; justify-content: space-between; font-size: 12px; color: var(--text-muted); margin-top: 4px;">
              <span>Lento (1%)</span>
              <span>Veloce (1000%)</span>
            </div>
          </div>
          <div>
            <label for="frequency-slider" style="display: block; margin-bottom: 8px; font-weight: 600;">
              Frequenza: <input type="number" id="frequency-input" value="125" min="10" max="500" step="1" style="width: 80px; padding: 4px; border: 1px solid #ddd; border-radius: 4px; font-size: 14px; margin-left: 8px;"> Hz
            </label>
            <input type="range" id="frequency-slider" min="10" max="500" value="125" step="5" 
                   style="width: 100%; max-width: 400px; height: 8px; cursor: pointer;">
            <div style="display: flex; justify-content: space-between; font-size: 12px; color: var(--text-muted); margin-top: 4px;">
              <span>Lento (10Hz)</span>
              <span>Veloce (500Hz)</span>
            </div>
          </div>
          <div style="grid-column: 1 / -1; margin-top: 12px; padding: 12px; background: #f0f7ff; border-radius: 6px; border: 1px solid #0066cc;">
            <div style="display: flex; align-items: center; gap: 12px;">
              <div style="flex: 1;">
                <div style="font-size: 12px; color: #666; margin-bottom: 4px;">Fluidità Movimento</div>
                <div style="display: flex; align-items: center; gap: 8px;">
                  <div id="fluidity-indicator" style="width: 200px; height: 20px; background: #e0e0e0; border-radius: 10px; overflow: hidden; position: relative;">
                    <div id="fluidity-bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #00aa00 0%, #00ff00 100%); transition: width 0.3s;"></div>
                  </div>
                  <span id="fluidity-value" style="font-size: 14px; font-weight: 600; color: #0066cc;">—</span>
                </div>
              </div>
              <div style="font-size: 11px; color: #666;">
                <div>Comandi inviati: <span id="command-count">0</span></div>
                <div>Ultimo aggiornamento: <span id="last-update">—</span></div>
              </div>
            </div>
          </div>
        </div>
        <div style="margin-top: 10px;">
          <button type="button" class="secondary-btn" id="stop-joystick">Emergency Stop</button>
          <button type="button" class="primary-btn" id="check-processes" style="margin-left: 10px;">🔍 Verifica Processi</button>
          <button type="button" class="danger-btn" id="restart-web-interface" style="margin-left: 10px; background: #cc0000; color: white;">🔄 Riavvia Web Interface</button>
        </div>
        <div id="process-status" style="margin-top: 10px; padding: 8px; background: #f0f0f0; border-radius: 4px; font-size: 12px; display: none;">
        </div>
      </div>

      <!-- Sezione Logging -->
      <div id="logging-section" style="margin-top: 20px; border: 1px solid var(--border); border-radius: 8px; padding: 16px; background: var(--bg-light);">
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px;">
          <h3 style="margin: 0; color: var(--primary-dark); font-size: 18px;">📋 Log & Errori</h3>
          <div>
            <button type="button" class="secondary-btn" id="clear-logs" style="font-size: 12px; padding: 4px 8px;">🗑️ Pulisci</button>
            <button type="button" class="secondary-btn" id="toggle-logs" style="font-size: 12px; padding: 4px 8px; margin-left: 5px;">⏸️ Pausa</button>
          </div>
        </div>
        <div id="logs-container" style="background: #1e1e1e; color: #d4d4d4; font-family: 'Courier New', monospace; font-size: 12px; padding: 12px; border-radius: 4px; max-height: 400px; overflow-y: auto; min-height: 200px;">
          <div style="color: #888; font-style: italic;">In attesa di log...</div>
        </div>
        <div style="margin-top: 8px; font-size: 11px; color: var(--text-muted);">
          <span id="log-count">0</span> log totali | 
          <span id="log-errors">0</span> errori | 
          <span id="log-warnings">0</span> warning
        </div>
      </div>

      <!-- Sezioni Avanzate (nascoste inizialmente) -->
      <details id="advanced-controls" style="margin-top: 20px; display: none;">
        <summary style="cursor: pointer; padding: 12px; background: #f0f0f0; border-radius: 6px; font-weight: 600; color: #666;">
          ⚙️ Controlli Avanzati (clicca per espandere)
        </summary>
        
      <form id="move-form">
      <fieldset>
        <legend>Joint targets (radians)</legend>
        <div class="parameters">
          <label>Step size (rad)
            <input type="number" step="0.01" name="step" id="step-size" value="0.10">
          </label>
          <label style="margin-left: 2em;">
            <input type="checkbox" id="cartesian-mode">
            Cartesian mode (Tool X/Y/Z control)
          </label>
        </div>
        <div class="joint-grid">
          {% for idx in range(6) %}
            <div class="joint-card">
              <h3>Joint {{ idx + 1 }}</h3>
              <div class="joint-controls">
                <button type="button" class="joint-decrement" data-target="joint{{ idx }}">&larr;</button>
                <input type="number" step="0.01" name="joint{{ idx }}" value="0.0">
                <button type="button" class="joint-increment" data-target="joint{{ idx }}">&rarr;</button>
              </div>
            </div>
          {% endfor %}
        </div>
      </fieldset>

      <fieldset>
        <legend>Motion parameters</legend>
        <div class="parameters">
          <label>Acceleration
            <input type="number" step="0.1" name="acceleration" value="1.2">
          </label>
          <label>Velocity
            <input type="number" step="0.05" name="velocity" value="0.25">
          </label>
          <label>Blend radius
            <input type="number" step="0.01" name="blend_radius" value="0.0">
          </label>
          <label>
            <input type="checkbox" name="async_move">
            Non blocking (no wait)
          </label>
        </div>
      </fieldset>

      <div class="actions">
        <button type="submit" class="primary-btn">MoveJ</button>
        <button type="button" class="secondary-btn" onclick="sendStop()">Stop</button>
      </div>
    </form>
      </details>
    </div>

    <div id="status" class="status-bar">
      Stato: <span class="ready">Ready</span>
    </div>

    <script>
      const status = document.getElementById("status");
      const form = document.getElementById("move-form");
      const stepInput = document.getElementById("step-size");
      const jointInputs = Array.from(form.querySelectorAll("input[name^='joint']"));
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
        status.innerHTML = `Stato: <span class="${ok ? "ready" : "error"}">${text}</span>`;
      }

      function describeAge(ageSeconds, isoString) {
        if (ageSeconds == null) return "—";
        const rounded = ageSeconds > 60 ? `${(ageSeconds / 60).toFixed(1)} min` : `${ageSeconds.toFixed(2)} s`;
        return isoString ? `${rounded} fa (${isoString})` : `${rounded} fa`;
      }

      function renderTopics(publishers) {
        if (!publishers || Object.keys(publishers).length === 0) {
          return "<div class='topic-row error'><span>Nessun publisher</span><span>offline</span></div>";
        }
        return Object.entries(publishers).map(([name, ok]) => {
          const cls = ok ? "ok" : "error";
          const label = ok ? "online" : "missing";
          return `<div class="topic-row ${cls}"><span>${name}</span><span>${label}</span></div>`;
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
          rosBridgeState.className = `monitor-value badge ${rosReady ? "badge-ok" : "badge-error"}`;
        }

        const loopRunning = bridge && bridge.publish_loop_running;
        const publishRate = bridge && typeof bridge.publish_rate_hz === "number"
          ? bridge.publish_rate_hz.toFixed(0)
          : "0";
        if (rosLoopState) {
          rosLoopState.textContent = loopRunning ? `${publishRate} Hz` : "fermo";
          rosLoopState.className = `monitor-value badge ${loopRunning ? "badge-ok" : "badge-error"}`;
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
            env.ROS_DISTRO ? `ROS ${env.ROS_DISTRO}` : "ROS? n/d",
            env.LD_LIBRARY_PATH ? "LD_LIB ✓" : "LD_LIB ✗",
            env.PYTHONPATH ? "PYTHONPATH ✓" : "PYTHONPATH ✗",
            env.HOSTNAME ? `Host: ${env.HOSTNAME}` : null,
          ].filter(Boolean).join(" · ") || "n/d";
        }

        if (rosTopicList) {
          rosTopicList.innerHTML = renderTopics(bridge ? bridge.publishers : null);
        }
        if (rosWarning) {
          rosWarning.textContent = bridge && bridge.last_error ? `⚠️ Errore: ${bridge.last_error}` : "";
          rosWarning.style.display = bridge && bridge.last_error ? "block" : "none";
        }
        
        // Mostra info utili per debug
        if (rosInfo) {
          const infoMessages = [];
          if (bridge) {
            const speeds = bridge.target_speeds || [0,0,0,0,0,0];
            const hasNonZeroSpeed = speeds.some(s => Math.abs(s) > 0.001);
            if (hasNonZeroSpeed) {
              infoMessages.push(`🎮 Velocità target: [${speeds.map(s => s.toFixed(3)).join(", ")}]`);
            }
            if (bridge.publish_loop_running && bridge.last_publish_age_s !== null) {
              const age = bridge.last_publish_age_s;
              if (age > 0.1) {
                infoMessages.push(`⚠️ Ultimo publish ${age.toFixed(2)}s fa - potrebbe essere un problema`);
              } else {
                infoMessages.push(`✅ Pubblicazione attiva (${bridge.publish_rate_hz}Hz)`);
              }
            }
            if (!bridge.ros_initialized) {
              infoMessages.push(`❌ ROS2 non inizializzato - controlla la configurazione`);
            }
            if (!bridge.publish_loop_running) {
              infoMessages.push(`❌ Loop di pubblicazione fermo - riavvia il servizio`);
            }
          }
          rosInfo.textContent = infoMessages.length > 0 ? infoMessages.join(" | ") : "";
          rosInfo.style.display = infoMessages.length > 0 ? "block" : "none";
        }

        if (rosStatusJson) {
          rosStatusJson.textContent = JSON.stringify(payload, null, 2);
        }
        if (statusTimestamp) {
          statusTimestamp.textContent = `Agg. ${new Date().toLocaleTimeString()}`;
        }
      }

      async function fetchSystemStatus(showToast = false) {
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
        refreshStatusBtn.addEventListener("click", () => fetchSystemStatus(true));
      }
      if (hasMonitor) {
        fetchSystemStatus();
        setInterval(fetchSystemStatus, 3000);
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
          robotMode.className = `monitor-value badge ${mode === "RUNNING" ? "badge-ok" : "badge-error"}`;
        }

        if (safetyMode) {
          const mode = dashboard.safetymode || "unknown";
          safetyMode.textContent = mode;
          safetyMode.className = `monitor-value badge ${mode === "NORMAL" ? "badge-ok" : "badge-warning"}`;
        }

        if (programState) {
          const state = dashboard.programState || "unknown";
          programState.textContent = state;
          programState.className = `monitor-value badge ${state === "PLAYING" ? "badge-ok" : "badge-error"}`;
        }

        if (remoteControl) {
          const rc = dashboard.remote_control || "unknown";
          remoteControl.textContent = rc;
          remoteControl.className = `monitor-value badge ${rc === "true" ? "badge-ok" : "badge-error"}`;
        }

        // RTDE data
        if (jointPositions && rtde.joints) {
          jointPositions.textContent = `[${rtde.joints.map(j => j.toFixed(4)).join(", ")}]`;
        }

        if (tcpPose && rtde.tcp_pose) {
          tcpPose.textContent = `[${rtde.tcp_pose.map(p => p.toFixed(4)).join(", ")}]`;
        }

        // Warnings
        if (robotStatusWarning) {
          const warnings = [];
          const robotModeClean = (dashboard.robotmode || "").replace(/^Robotmode:\s*/i, "").trim();
          if (robotModeClean && robotModeClean !== "RUNNING") {
            warnings.push(`⚠️ Robot non in RUNNING: ${robotModeClean}`);
          }
          if (dashboard.programState && !dashboard.programState.includes("PLAYING")) {
            warnings.push(`⚠️ Programma non in PLAYING: ${dashboard.programState}`);
          }
          if (dashboard.remote_control && dashboard.remote_control !== "true") {
            warnings.push(`⚠️ Remote control non attivo`);
          }
          if (rtde.error) {
            warnings.push(`⚠️ RTDE: ${rtde.error}`);
          }
          if (dashboard.error) {
            warnings.push(`⚠️ Dashboard: ${dashboard.error}`);
          }
          robotStatusWarning.textContent = warnings.join(" | ");
          robotStatusWarning.style.display = warnings.length > 0 ? "block" : "none";
        }

        // Info
        if (robotStatusInfo) {
          const infos = [];
          if (rtde.joints && rtde.tcp_pose) {
            infos.push(`✅ Dati RTDE disponibili`);
          }
          if (dashboard.robotmode === "RUNNING" && dashboard.programState === "PLAYING") {
            infos.push(`✅ Robot pronto per controllo`);
          }
          robotStatusInfo.textContent = infos.join(" | ");
          robotStatusInfo.style.display = infos.length > 0 ? "block" : "none";
        }

        if (robotStatusJson) {
          robotStatusJson.textContent = JSON.stringify(payload.data, null, 2);
        }
        if (robotStatusTimestamp) {
          robotStatusTimestamp.textContent = `Agg. ${new Date().toLocaleTimeString()}`;
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
          systemMessages.innerHTML = `<div style="color: ${isError ? '#cc0000' : '#0066cc'};">${msg}</div>`;
        }
      }

      async function fetchSystemStatus(showToast = false) {
        try {
          const response = await fetch("/api/system/status");
          const payload = await response.json();
          if (payload.status === "ok") {
            const data = payload.data;
            
            if (ros2DriverStatus) {
              ros2DriverStatus.textContent = data.ros2_driver.running ? "✅ Attivo" : "❌ Fermo";
              ros2DriverStatus.className = "monitor-value badge " + (data.ros2_driver.running ? "badge-success" : "badge-error");
            }
            
            if (controllerStatus) {
              if (data.controller.active) {
                controllerStatus.textContent = `✅ ${data.controller.name}`;
                controllerStatus.className = "monitor-value badge badge-success";
              } else {
                controllerStatus.textContent = data.controller.error || "❌ Nessuno";
                controllerStatus.className = "monitor-value badge badge-error";
              }
            }
            
            if (robotModeStatus) {
              const mode = data.robot_mode || "—";
              robotModeStatus.textContent = mode;
              robotModeStatus.className = "monitor-value badge " + (mode === "RUNNING" ? "badge-success" : "badge-error");
            }
            
            if (portStatus) {
              portStatus.textContent = data.port_50002.listening ? "✅ Aperta" : "❌ Chiusa";
              portStatus.className = "monitor-value badge " + (data.port_50002.listening ? "badge-success" : "badge-error");
            }
            
            // Aggiorna status rapido
            const quickStatus = document.getElementById("quick-status");
            const ros2DriverQuick = document.getElementById("ros2-driver-status-quick");
            const controllerQuick = document.getElementById("controller-status-quick");
            const robotModeQuick = document.getElementById("robot-mode-status-quick");
            const portQuick = document.getElementById("port-status-quick");
            
            if (quickStatus && ros2DriverQuick && controllerQuick && robotModeQuick && portQuick) {
              quickStatus.style.display = "block";
              ros2DriverQuick.textContent = data.ros2_driver.running ? "✅ Attivo" : "❌ Fermo";
              controllerQuick.textContent = data.controller.active ? data.controller.name : "❌ Nessuno";
              robotModeQuick.textContent = data.robot_mode || "—";
              portQuick.textContent = data.port_50002.listening ? "✅ Aperta" : "❌ Chiusa";
            }
            
            // Aggiorna anche wizard se necessario
            if (data.robot_mode === "RUNNING" && data.port_50002.listening) {
              const stepBStatus = document.getElementById("step-b-status");
              if (stepBStatus && stepBStatus.textContent.includes("Bloccato")) {
                updateWizardStep('a', 'success', 'Driver ROS2 attivo.');
                updateWizardStep('b', 'success', 'Driver verificato.');
              }
            }
            
            if (systemStatusTimestamp) {
              systemStatusTimestamp.textContent = `Agg. ${new Date().toLocaleTimeString()}`;
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
      const wizardStartDriverBtn = document.getElementById("wizard-start-driver");
      const wizardCheckTeachPendantBtn = document.getElementById("wizard-check-teach-pendant");
      let wizardCurrentStep = 'a';
      let wizardCheckInterval = null;

      function updateWizardStep(step, status, message) {
        const stepEl = document.getElementById(`step-${step}`);
        const statusEl = document.getElementById(`step-${step}-status`);
        const messageEl = document.getElementById(`step-${step}-message`);
        const retryBtn = document.getElementById(`wizard-retry-${step}`);
        
        if (stepEl) {
          stepEl.classList.remove('active', 'completed', 'error');
          stepEl.style.opacity = '1';
          if (status === 'success') {
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
          statusEl.className = 'step-status';
          if (status === 'success') {
            statusEl.textContent = '✅';
            statusEl.classList.add('success');
          } else if (status === 'error') {
            statusEl.textContent = '❌';
            statusEl.classList.add('error');
          } else if (status === 'waiting') {
            statusEl.textContent = '⏳';
            statusEl.classList.add('waiting');
          } else if (status === 'active') {
            statusEl.textContent = '🔄';
            statusEl.classList.add('waiting');
          } else {
            statusEl.textContent = '⏸️';
          }
        }
        
        if (messageEl && message) {
          messageEl.innerHTML = message;
        }
        
        // Mostra pulsante retry se errore
        if (retryBtn) {
          retryBtn.style.display = (status === 'error') ? 'block' : 'none';
        }
      }

      async function wizardStepA() {
        updateWizardStep('a', 'active', '🔄 Verifica e pulizia processi esistenti...');
        
        // Prima verifica e kill processi esistenti (IMPORTANTE per evitare crash)
        try {
          const checkResponse = await fetch("/api/system/check_processes", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({kill_duplicates: true})
          });
          const checkPayload = await checkResponse.json();
          if (checkPayload.status === "ok") {
            if (checkPayload.data.duplicates_found) {
              updateWizardStep('a', 'active', `🔄 Processi duplicati trovati e terminati:<br>- Driver ROS2: ${checkPayload.data.ros2_driver.length}<br>- Web Interface: ${checkPayload.data.web_interface.length}<br>Attendo pulizia...`);
              await new Promise(resolve => setTimeout(resolve, 3000)); // Attendi 3 secondi per pulizia completa
            } else {
              updateWizardStep('a', 'active', '✅ Nessun processo duplicato trovato. Procedo con avvio...');
              await new Promise(resolve => setTimeout(resolve, 500));
            }
          }
        } catch (err) {
          console.warn("Errore verifica processi:", err);
          updateWizardStep('a', 'active', '⚠️ Impossibile verificare processi. Procedo comunque...');
        }
        
        updateWizardStep('a', 'active', '🔄 Avvio driver ROS2 in corso...');
        try {
          const response = await fetch("/api/system/start_driver", { method: "POST" });
          const payload = await response.json();
          if (payload.status === "ok") {
            updateWizardStep('a', 'waiting', '✅ Driver avviato! Attendo stabilizzazione (3 secondi)...');
            setTimeout(() => wizardStepB(), 3000);
          } else {
            updateWizardStep('a', 'error', `❌ Errore: ${payload.message}<br><small>Clicca "Riprova" dopo aver verificato i processi.</small>`);
          }
        } catch (err) {
          updateWizardStep('a', 'error', `❌ Errore di connessione: ${err.message}<br><small>Verifica che la web interface sia attiva e riprova.</small>`);
        }
      }

      async function wizardStepB() {
        updateWizardStep('b', 'active', '🔄 Verifica driver attivo e porta 50002...');
        let attempts = 0;
        const maxAttempts = 10;
        
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
              } else if (attempts >= maxAttempts) {
                updateWizardStep('b', 'error', 'Driver non attivo o porta 50002 non aperta dopo ' + maxAttempts + ' tentativi.');
                return true;
              }
            }
          } catch (err) {
            console.error("Wizard step B error", err);
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

      async function wizardStepC() {
        updateWizardStep('c', 'active', '🔄 Verifica controller...');
        
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
              if (controller.active && controller.name === 'forward_velocity_controller') {
                updateWizardStep('c', 'success', '✅ Controller forward_velocity_controller già attivo.');
                setTimeout(() => wizardStepD(), 1000);
                return true;
              }
              
              // Se non è attivo, prova ad attivarlo
              if (!controller.active || controller.name !== 'forward_velocity_controller') {
                updateWizardStep('c', 'active', `🔄 Attivazione forward_velocity_controller... (tentativo ${attempts + 1}/${maxAttempts})`);
                
                const switchResponse = await fetch("/api/system/switch_controller", {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({ use_scaled: false })
                });
                
                const switchData = await switchResponse.json();
                
                if (switchData.status === "ok") {
                  // Attendi un momento e verifica che sia stato attivato
                  await new Promise(resolve => setTimeout(resolve, 1500));
                  
                  // Verifica di nuovo
                  const verifyResponse = await fetch("/api/system/status");
                  const verifyData = await verifyResponse.json();
                  
                  if (verifyData.status === "ok" && verifyData.data.controller) {
                    const newController = verifyData.data.controller;
                    if (newController.active && newController.name === 'forward_velocity_controller') {
                      updateWizardStep('c', 'success', '✅ Controller forward_velocity_controller attivato.');
                      setTimeout(() => wizardStepD(), 1000);
                      return true;
                    }
                  }
                } else {
                  updateWizardStep('c', 'error', `❌ Errore attivazione: ${switchData.message}`);
                  return false;
                }
              }
            }
            
            attempts++;
            if (attempts >= maxAttempts) {
              updateWizardStep('c', 'error', `❌ Controller non attivato dopo ${maxAttempts} tentativi. Verifica manualmente.`);
              return true; // Ferma il loop
            }
            
            return false; // Continua a provare
          } catch (err) {
            console.error("Wizard step C error", err);
            attempts++;
            if (attempts >= maxAttempts) {
              updateWizardStep('c', 'error', `❌ Errore: ${err.message}`);
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

      function wizardStepD() {
        updateWizardStep('d', 'waiting', 'Attendi che attivi External Control sul Teach Pendant, poi clicca il pulsante qui sotto.');
        const btn = document.getElementById("wizard-check-teach-pendant");
        if (btn) {
          btn.style.display = "block";
        }
      }

      async function wizardStepE() {
        updateWizardStep('e', 'active', '🔄 Verifica connessione robot...');
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
              const controllerReady = data.controller.active;
              
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
              const debugInfo = `Driver: ${driverReady ? '✅' : '❌'}, Controller: ${controllerReady ? '✅' : '❌'}, Mode: ${data.robot_mode || 'N/A'}, Safety: ${data.robot_safety_mode || 'N/A'}, Remote: ${data.remote_control || 'N/A'}, Program: ${data.program_state || 'N/A'}`;
              
              if (isReady) {
                let successMsg = '✅ <strong style="font-size: 16px; color: #00aa00;">Robot connesso e pronto!</strong><br>';
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
                  joystickSection.scrollIntoView({ behavior: 'smooth', block: 'start' });
                  // Evidenzia la sezione
                  joystickSection.style.animation = 'pulse 2s ease-in-out';
                  setTimeout(() => {
                    joystickSection.style.animation = '';
                  }, 2000);
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
                if (!driverReady) errorMsg += '<br>❌ Driver ROS2 non attivo o porta 50002 chiusa.';
                if (!controllerReady) errorMsg += '<br>❌ Controller non attivo.';
                if (data.robot_mode && data.robot_mode !== "RUNNING" && data.robot_mode !== "unknown") {
                  errorMsg += '<br>⚠️ Modalità robot: ' + data.robot_mode + ' (atteso: RUNNING).';
                }
                if (data.robot_safety_mode && data.robot_safety_mode !== "NORMAL" && data.robot_safety_mode !== "unknown") {
                  errorMsg += '<br>⚠️ Safety mode: ' + data.robot_safety_mode + ' (atteso: NORMAL).';
                }
                if (data.remote_control !== true && data.program_state !== "PLAYING" && data.program_state !== "PLAYING remote_control.urp") {
                  errorMsg += '<br>⚠️ Remote Control: ' + (data.remote_control || 'non disponibile') + ', Program: ' + (data.program_state || 'non disponibile');
                }
                errorMsg += '<br><br><small>Se il robot è effettivamente in esecuzione, puoi comunque provare a usare i joystick.</small>';
                updateWizardStep('e', 'error', errorMsg);
                // Mostra comunque i joystick se driver e controller sono OK
                if (driverReady && controllerReady) {
                  const joystickSection = document.getElementById('joystick-section');
                  if (joystickSection) {
                    joystickSection.style.display = 'block';
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
                  updateWizardStep('e', 'active', `🔄 Verifica connessione... (tentativo ${attempts}/${maxAttempts})<br><small>${debugInfo}</small>`);
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

      if (wizardStartDriverBtn) {
        wizardStartDriverBtn.addEventListener("click", wizardStepA);
      }

      if (wizardCheckTeachPendantBtn) {
        wizardCheckTeachPendantBtn.addEventListener("click", () => {
          updateWizardStep('d', 'success', 'Verifica connessione in corso...');
          wizardStepE();
        });
      }

      // Pulsanti Retry
      for (const step of ['a', 'b', 'c', 'd', 'e']) {
        const retryBtn = document.getElementById(`wizard-retry-${step}`);
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
      initWizard();
      initLogging(); // Inizializza sistema di logging

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
        const input = form.querySelector(`input[name='${targetName}']`);
        if (!input) return;
        const current = parseFloat(input.value) || 0;
        const next = current + delta;
        input.value = next.toFixed(3);
      }

      function handleArrowButtons(event) {
        const button = event.target.closest("button[data-target]");
        if (!button) return;
        event.preventDefault();
        const step = parseFloat(stepInput.value) || 0.1;
        const isIncrement = button.classList.contains("joint-increment");
        const delta = isIncrement ? step : -step;
        updateJoint(button.dataset.target, delta);
      }

      document.querySelectorAll(".joint-increment, .joint-decrement").forEach((btn) => {
        btn.addEventListener("click", handleArrowButtons);
      });

      jointInputs.forEach((input) => {
        input.addEventListener("focus", (event) => event.target.select());
      });

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
      let lastCommandTime = null;
      let commandIntervals = [];
      const maxIntervals = 50;
      const fluidityBar = document.getElementById("fluidity-bar");
      const fluidityValue = document.getElementById("fluidity-value");
      const commandCountEl = document.getElementById("command-count");
      const lastUpdateEl = document.getElementById("last-update");
      
      function updateFluidityTracker() {
        commandCount++;
        const now = Date.now();
        
        if (lastCommandTime) {
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
                msg = `⚠️ Processi doppi trovati e killati: ${data.killed.join(", ")}`;
                processStatus.style.background = "#fff3cd";
                processStatus.style.color = "#856404";
              } else {
                msg = "✅ Nessun processo doppio trovato";
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
          if (!confirm("⚠️ Sei sicuro di voler riavviare il web interface? La pagina si ricaricherà automaticamente.")) {
            return;
          }
          
          restartWebInterfaceBtn.disabled = true;
          restartWebInterfaceBtn.textContent = "🔄 Riavvio in corso...";
          
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
              alert(`Errore riavvio: ${payload.message}`);
              restartWebInterfaceBtn.disabled = false;
              restartWebInterfaceBtn.textContent = "🔄 Riavvia Web Interface";
            }
          } catch (err) {
            alert(`Errore: ${err.message}`);
            restartWebInterfaceBtn.disabled = false;
            restartWebInterfaceBtn.textContent = "🔄 Riavvia Web Interface";
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
        handle.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
        joyX.textContent = xNorm.toFixed(2);
        joyY.textContent = (-yNorm).toFixed(2);
      }

      function resetJoystick() {
        joyVector = { x: 0, y: 0 };
        setHandlePosition(0, 0);
        // Update speeds to zero (bridge publishes continuously at 125Hz)
        updateSpeeds();
      }

      // --- Logging System Functions ---
      function initLogging() {
        // Verifica che le variabili siano inizializzate
        if (typeof lastLogId === 'undefined') {
          console.error('❌ Variabili logging non inizializzate!');
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
            toggleLogsBtn.textContent = logsPaused ? '▶️ Riprendi' : '⏸️ Pausa';
          });
        }

        // Aggiorna log ogni 500ms
        if (typeof logUpdateInterval !== 'undefined') {
          logUpdateInterval = setInterval(updateLogs, 500);
          updateLogs(); // Prima chiamata immediata
        } else {
          console.error('❌ logUpdateInterval non definito!');
        }
      }

      async function updateLogs() {
        if (typeof logsPaused === 'undefined' || logsPaused) return;
        
        try {
          const response = await fetch(`/api/logs?last_id=${lastLogId}`);
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
              const message = log.message.replace(/⚠️/g, '⚠').replace(/✅/g, '✓').replace(/📤/g, '→').replace(/⏳/g, '…');
              logLine.innerHTML = `<span style="color: #888;">[${log.timestamp}]</span> <span style="color: #569cd6;">[${log.level}]</span> ${message}`;
              
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

      function updateSpeeds() {
        // Calculate speeds from joystick positions
        const magnitude = Math.hypot(joyVector.x, joyVector.y);
        const magnitude2 = Math.hypot(joy2Vector.x, joy2Vector.y);
        
        let speeds;
        if (magnitude < JOY_DEADZONE && magnitude2 < JOY_DEADZONE) {
          speeds = [0, 0, 0, 0, 0, 0];
        } else {
          const cartesianMode = document.getElementById("cartesian-mode").checked;
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
        
        // Log per debug
        const maxSpeed = Math.max(...speeds.map(Math.abs));
        if (maxSpeed > 0.001) {
          console.log(`🎮 Joystick: speeds=[${speeds.map(s => s.toFixed(4)).join(', ')}], max=${maxSpeed.toFixed(4)}`);
        }
        
        // Update speeds (bridge publishes continuously at 125Hz)
        const cartesianMode = document.getElementById("cartesian-mode").checked;
        fetch("/api/servo_loop_update", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ speeds, cartesian: cartesianMode }),
        })
        .then(response => {
          if (!response.ok) {
            console.error(`❌ Update error: ${response.status} ${response.statusText}`);
            return response.text().then(text => {
              console.error(`❌ Error response: ${text}`);
              setStatus(`Errore invio comando: ${response.status} - ${text}`, false);
            });
          } else {
            updateFluidityTracker(); // Aggiorna tracker fluidità
            return response.json();
          }
        })
        .then(data => {
          if (data && data.message) {
            console.log(`✅ Speed update: ${data.message}`);
            if (data.status === 'error') {
              console.error(`❌ Server error: ${data.message}`);
              setStatus(`Errore server: ${data.message}`, false);
            }
          }
        })
        .catch(err => {
          console.error(`❌ Update error:`, err);
          setStatus(`Errore: ${err.message}`, false);
        });
      }

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
        resetJoystick();
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
        handle2.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
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
        resetJoystick2();
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
    </script>
  </body>
</html>
"""


def get_controller() -> RemoteURController:
    config = load_config()
    return RemoteURController(config.robot_ip, config.port)

def get_ros2_bridge():
    """Ottiene il bridge ROS2 (fallback)."""
    global _ros2_bridge
    if ROS2_AVAILABLE and not _ros2_bridge:
        _ros2_bridge = ROS2Bridge()
    return _ros2_bridge


def load_config() -> ControllerConfig:
    robot_ip = os.environ.get("UR_ROBOT_IP")
    if not robot_ip:
        raise RuntimeError("Set UR_ROBOT_IP env var with the robot IP address")
    port = int(os.environ.get("UR_ROBOT_PORT", 30002))
    return ControllerConfig(robot_ip=robot_ip, port=port)


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


@app.route("/api/servo_loop_start", methods=["POST"])
def api_servo_loop_start():
    """Inizializza ROS2 bridge (publishing starts automatically at 125Hz)."""
    if ROS2_AVAILABLE:
        bridge = get_ros2_bridge()
        if bridge and bridge.ensure_ros():
            return jsonify({"status": "ok", "message": "ROS2 bridge ready - publishing at 125Hz"})
    return jsonify({"status": "ok", "message": "Using socket fallback"})


@app.route("/api/servo_loop_update", methods=["POST"])
def api_servo_loop_update():
    """Aggiorna velocità via socket diretto (più affidabile di ROS2)."""
    try:
        payload = request.get_json(force=True)
        speeds = parse_joints(payload.get("speeds"))
        cartesian_mode = payload.get("cartesian", False)
        
        # ROS2 (SOLUZIONE PRINCIPALE - secondo documentazione ufficiale)
        # Richiede driver UR ROS2 in esecuzione e robot configurato con External Control URCap
        if ROS2_AVAILABLE:
            bridge = get_ros2_bridge()
            if bridge:
                if bridge.ensure_ros():
                    # Se il bridge ROS2 è attivo, usa quello (gestisce già la sicurezza)
                    if bridge.publish_speedj(speeds):
                        max_speed = max(abs(s) for s in speeds)
                        app.logger.info(f"✅ Comando ROS2 pubblicato: speeds={[f'{s:.4f}' for s in speeds]}, max={max_speed:.4f}")
                        return jsonify({"status": "ok", "message": f"ROS2 speedj (max speed: {max_speed:.4f} rad/s)"})
                    else:
                        app.logger.warning("❌ Bridge ROS2 disponibile ma publish_speedj ha restituito False")
                        app.logger.warning(f"   Speeds richieste: {speeds}")
                else:
                    app.logger.warning("Bridge ROS2 disponibile ma ensure_ros() ha fallito")
            else:
                app.logger.warning("Bridge ROS2 non disponibile (get_ros2_bridge() ha restituito None)")
        
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
        controller = get_controller()
        try:
            if cartesian_mode:
                controller.speedl(speeds, duration=0.008, acceleration=0.3)  # 125Hz = 0.008s
                return jsonify({"status": "ok", "message": "Socket control (speedl cartesian) - fallback"})
            else:
                # Usa servoj_velocity per controllo fluido real-time (125Hz)
                controller.servoj_velocity(speeds, t=0.008, lookahead_time=0.1, gain=300.0)
                return jsonify({"status": "ok", "message": "Socket control (servoj_velocity) - fallback"})
        except (socket.timeout, OSError, ConnectionError) as sock_err:
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
        # ⚠️ IMPORTANTE: RTDE può essere usato da UN SOLO processo alla volta!
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
    if ROS2_AVAILABLE and status["ros2_driver"]["running"]:
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
                            break
                        elif 'scaled_joint_trajectory_controller' in controller.name and controller.state == 'active':
                            status["controller"]["active"] = True
                            status["controller"]["name"] = "scaled_joint_trajectory_controller"
                            break
        except Exception as e:
            status["controller"]["error"] = str(e)
    
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
            app.logger.info("🔍 Esecuzione script kill_rtde_processes.sh...")
            kill_result = subprocess.run(
                ['bash', kill_rtde_script],
                capture_output=True,
                text=True,
                timeout=10
            )
            if kill_result.returncode == 0:
                app.logger.info(f"✅ Script kill RTDE completato:\n{kill_result.stdout}")
            else:
                app.logger.warning(f"⚠️  Script kill RTDE ha avuto problemi:\n{kill_result.stderr}")
        else:
            # Fallback: kill manuale se lo script non esiste
            app.logger.warning("⚠️  Script kill_rtde_processes.sh non trovato, uso metodo manuale")
            
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
                port_pid = lsof_result.stdout.strip().split('\n')[0]
                try:
                    subprocess.run(['kill', '-9', port_pid], timeout=2)
                    print(f'🔧 Killato processo {port_pid} che usava porta 50002')
                except:
                    pass
        except:
            # Se lsof non disponibile, prova con fuser
            try:
                fuser_result = subprocess.run(
                    ['fuser', '50002/tcp'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if fuser_result.returncode == 0:
                    port_pid = fuser_result.stdout.strip().split()[0]
                    try:
                        subprocess.run(['kill', '-9', port_pid], timeout=2)
                        print(f'🔧 Killato processo {port_pid} che usava porta 50002')
                    except:
                        pass
            except:
                pass
        
        time.sleep(1)  # Aspetta che la porta sia libera
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
export HOME={os.path.expanduser('~')}
cd {os.path.expanduser('~/MekoAiAccelerator')}

# PRIMA: Esegui script per killare altri processi RTDE
KILL_RTDE_SCRIPT="{kill_rtde_script}"
if [ -f "$KILL_RTDE_SCRIPT" ]; then
    echo "🔍 Esecuzione kill_rtde_processes.sh..." >> /tmp/ros2_driver.log
    bash "$KILL_RTDE_SCRIPT" >> /tmp/ros2_driver.log 2>&1
else
    echo "⚠️  Script kill_rtde_processes.sh non trovato: $KILL_RTDE_SCRIPT" >> /tmp/ros2_driver.log
fi

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# SOLUZIONE RTDE OVERFLOW: Riduci update_rate da 500Hz a 30Hz (come suggerito dall'utente)
# Il file di configurazione viene caricato automaticamente dal launch file
UPDATE_RATE_FILE="$HOME/ros2_ws/install/ur_robot_driver/share/ur_robot_driver/config/ur5e_update_rate.yaml"
if [ -f "$UPDATE_RATE_FILE" ]; then
    echo "🔧 Modifica update_rate da 500Hz a 30Hz per evitare RTDE overflow..." >> /tmp/ros2_driver.log
    cp "$UPDATE_RATE_FILE" "$UPDATE_RATE_FILE.backup" 2>/dev/null || true
    cat > "$UPDATE_RATE_FILE" << 'EOF'
controller_manager:
  ros__parameters:
    update_rate: 30  # Hz - Ridotto da 500Hz a 30Hz per evitare RTDE overflow
EOF
    echo "✅ update_rate modificato a 30Hz" >> /tmp/ros2_driver.log
fi

# Avvia driver ROS2 in background
echo "=== AVVIO DRIVER ROS2 ===" >> /tmp/ros2_driver.log
echo "Data: $(date)" >> /tmp/ros2_driver.log
echo "Robot IP: {config.robot_ip}" >> /tmp/ros2_driver.log

# Avvia driver ROS2 con forward_velocity_controller invece di scaled_joint_trajectory_controller
# (scaled_joint_trajectory_controller causa segmentation fault)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:={config.robot_ip} launch_rviz:=false initial_joint_controller:=forward_velocity_controller >> /tmp/ros2_driver.log 2>&1 &
LAUNCH_PID=$!
echo "PID launch: $LAUNCH_PID" >> /tmp/ros2_driver.log
sleep 8  # Attendi che il processo si avvii completamente

# Verifica che il processo launch sia ancora vivo
if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo "ERROR: Processo launch morto immediatamente" >> /tmp/ros2_driver.log
    tail -50 /tmp/ros2_driver.log >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi

# Cerca il processo ur_ros2_control_node (il processo principale del driver)
FOUND_PID=$(pgrep -f 'ur_ros2_control_node' | head -1)
if [ -z "$FOUND_PID" ]; then
    # Aspetta altri 3 secondi
    sleep 3
    FOUND_PID=$(pgrep -f 'ur_ros2_control_node' | head -1)
fi

if [ -z "$FOUND_PID" ]; then
    echo "ERROR: ur_ros2_control_node non trovato" >> /tmp/ros2_driver.log
    echo "Ultimi 50 righe del log:" >> /tmp/ros2_driver.log
    tail -50 /tmp/ros2_driver.log >> /tmp/ros2_driver.log
    echo "ERROR"
    exit 1
fi

# Verifica che il processo ur_ros2_control_node sia ancora vivo dopo 5 secondi
# (diamo più tempo perché l'inizializzazione può richiedere tempo)
sleep 5
if ! ps -p $FOUND_PID > /dev/null 2>&1; then
    echo "ERROR: ur_ros2_control_node è crashato durante l'inizializzazione" >> /tmp/ros2_driver.log
    echo "Verifica ultimi errori nel log:" >> /tmp/ros2_driver.log
    tail -100 /tmp/ros2_driver.log | grep -i "error\|abort\|fault\|died" >> /tmp/ros2_driver.log || true
    echo "ERROR"
    exit 1
fi

# Verifica che il processo sia ancora vivo dopo altri 3 secondi (totale 8 secondi dall'avvio)
# Questo ci dà tempo per vedere se il processo si autokilla
sleep 3
if ! ps -p $FOUND_PID > /dev/null 2>&1; then
    echo "ERROR: ur_ros2_control_node si è autokillato dopo l'avvio" >> /tmp/ros2_driver.log
    echo "Ultimi errori nel log:" >> /tmp/ros2_driver.log
    tail -150 /tmp/ros2_driver.log | grep -i "error\|abort\|fault\|died\|kill" >> /tmp/ros2_driver.log || true
    echo "ERROR"
    exit 1
fi

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
echo "✅ Driver avviato correttamente: launch PID=$LAUNCH_PID, node PID=$FOUND_PID" >> /tmp/ros2_driver.log
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
            result = subprocess.run(
                ['bash', script_path],
                capture_output=True,
                text=True,
                timeout=30,  # Timeout aumentato per permettere avvio completo
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
                if "process has died" in log_content or "Aborted" in log_content or "Segmentation fault" in log_content:
                    # Analizza il tipo di crash
                    if "Aborted" in log_content and "process has died" in log_content:
                        error_msg = f"""❌ Driver crashato durante l'avvio!
🔴 PROBLEMA: Il processo ur_ros2_control_node è stato terminato (Aborted)
📋 POSSIBILI CAUSE:
1. Problema di inizializzazione del controller manager
2. Conflitto con altri processi RTDE
3. Problema di configurazione dei parametri

✅ SOLUZIONI:
1. Verifica che non ci siano altri processi RTDE attivi: pgrep -f rtde
2. Verifica che il robot sia raggiungibile: ping 192.168.10.194
3. Controlla i log completi qui sotto per dettagli

Log completo: {log_content[-2000:]}"""
                    elif "Segmentation fault" in log_content:
                        error_msg = f"""❌ Driver crashato con segmentation fault!
🔴 PROBLEMA: Errore di memoria nel driver
📋 POSSIBILI CAUSE:
1. Problema con scaled_joint_trajectory_controller (usiamo forward_velocity_controller)
2. Problema di inizializzazione

Log completo: {log_content[-2000:]}"""
                    else:
                        error_msg = f"❌ Driver crashato. Log: {log_content[-2000:]}"
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
                    error_msg = f"""❌ ERRORE RTDE OVERFLOW - Driver crashato immediatamente!

🔴 PROBLEMA: "Pipeline producer overflowed!" - EtherNet/IP è attivo sul robot!

✅ SOLUZIONE:
1. Sul Teach Pendant: Installation → Fieldbus
2. DISABILITA EtherNet/IP ❌
3. DISABILITA PROFINET ❌  
4. Riavvia robot
5. Riprova ad avviare il driver

⚠️  IMPORTANTE: EtherNet/IP e ROS2 driver NON possono essere attivi contemporaneamente!

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


@app.route("/api/system/restart_web_interface", methods=["POST"])
def api_restart_web_interface():
    """Riavvia il web interface killando il processo corrente."""
    import subprocess
    import os
    
    try:
        app.logger.info("🔄 Richiesta riavvio web interface...")
        
        # Trova il PID del processo corrente
        current_pid = os.getpid()
        
        # Crea uno script che killera questo processo e riavvierà
        script_content = f"""#!/bin/bash
# Script temporaneo per riavviare web interface
sleep 2
kill -9 {current_pid} 2>/dev/null || true
cd ~/MekoAiAccelerator
nohup ./avvia_web_interface_joystick.sh > /tmp/web_interface_restart.log 2>&1 &
"""
        
        script_path = "/tmp/restart_web_interface.sh"
        with open(script_path, 'w') as f:
            f.write(script_content)
        os.chmod(script_path, 0o755)
        
        # Esegui lo script in background
        subprocess.Popen(['bash', script_path], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL)
        
        app.logger.info(f"✅ Script di riavvio avviato (PID corrente: {current_pid})")
        return jsonify({
            "status": "ok", 
            "message": f"Riavvio avviato. Il processo corrente (PID: {current_pid}) verrà terminato e riavviato."
        })
    except Exception as e:
        app.logger.error(f"❌ Errore riavvio web interface: {e}")
        import traceback
        app.logger.error(traceback.format_exc())
        return jsonify({"status": "error", "message": str(e)})


@app.route("/api/system/switch_controller", methods=["POST"])
def api_switch_controller():
    """Switch controller tra forward_velocity e scaled_joint_trajectory."""
    if not ROS2_AVAILABLE:
        return jsonify({"status": "error", "message": "ROS2 non disponibile"})
    
    try:
        import subprocess
        
        app.logger.info("🔄 Richiesta switch controller...")
        
        # Verifica che il driver ROS2 sia attivo prima di procedere
        driver_check = subprocess.run(
            ['pgrep', '-f', 'ur_ros2_control_node'],
            capture_output=True,
            text=True
        )
        if driver_check.returncode != 0:
            error_msg = "Driver ROS2 non attivo. Avvia prima il driver ROS2."
            app.logger.error(f"❌ {error_msg}")
            return jsonify({"status": "error", "message": error_msg})
        
        # Determina quale controller attivare
        payload = request.get_json(force=True) if request.is_json else {}
        use_scaled = payload.get("use_scaled", False)
        
        if use_scaled:
            activate_controller = 'scaled_joint_trajectory_controller'
            deactivate_controller = 'forward_velocity_controller'
        else:
            activate_controller = 'forward_velocity_controller'
            deactivate_controller = 'scaled_joint_trajectory_controller'
        
        app.logger.info(f"🔄 Attivazione {activate_controller}...")
        app.logger.info(f"   Deattivazione: {deactivate_controller}")
        
        # Usa script Python separato per evitare problemi wait set e timeout
        # Lo script viene eseguito in un processo separato con il suo contesto ROS2
        script_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'switch_controller.py')
        
        # Se lo script non esiste, crealo
        if not os.path.exists(script_path):
            app.logger.warning(f"Script switch_controller.py non trovato, uso metodo alternativo")
            # Fallback: usa ros2 service call con formato YAML corretto
            ros2_setup = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
            # Formato YAML per ros2 service call
            yaml_request = f"activate_controllers:\\n- '{activate_controller}'\\ndeactivate_controllers:\\n- '{deactivate_controller}'\\nstrictness: 1"
            cmd = f"""{ros2_setup} && echo -e '{yaml_request}' | ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController"""
        else:
            # Usa lo script Python
            ros2_setup = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
            cmd = f"""{ros2_setup} && python3 {script_path} {activate_controller} {deactivate_controller}"""
        
        # Esegui comando con timeout più lungo (lo script aspetta fino a 20s per il servizio)
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True,
            text=True,
            timeout=30,  # Timeout aumentato per dare tempo allo script di aspettare il servizio
            cwd=os.path.expanduser('~')
        )
        
        # Analizza output dello script Python (stampa "OK:" o "ERROR:")
        stdout_text = result.stdout.strip()
        stderr_text = result.stderr.strip()
        
        # Controlla se lo script ha stampato "OK:" o "ERROR:"
        if "OK:" in stdout_text:
            app.logger.info(f"✅ Controller {activate_controller} attivato con successo")
            app.logger.info(f"   Output: {stdout_text}")
            return jsonify({"status": "ok", "message": f"Controller {activate_controller} attivato con successo"})
        elif "ERROR:" in stdout_text or "ERROR:" in stderr_text:
            # Estrai messaggio di errore dopo "ERROR:"
            error_msg = ""
            for line in (stdout_text + "\n" + stderr_text).split("\n"):
                if "ERROR:" in line:
                    error_msg = line.split("ERROR:")[-1].strip()
                    break
            if not error_msg:
                error_msg = stderr_text or stdout_text or "Errore sconosciuto"
            app.logger.error(f"❌ Switch controller fallito: {error_msg}")
            app.logger.error(f"   Return code: {result.returncode}")
            app.logger.error(f"   Stdout: {stdout_text}")
            app.logger.error(f"   Stderr: {stderr_text}")
            return jsonify({"status": "error", "message": f"Switch controller fallito: {error_msg}"})
        elif result.returncode == 0:
            # Nessun "OK:" o "ERROR:" ma return code 0 = successo
            app.logger.info(f"✅ Controller {activate_controller} attivato con successo")
            if stdout_text:
                app.logger.info(f"   Output: {stdout_text}")
            return jsonify({"status": "ok", "message": f"Controller {activate_controller} attivato con successo"})
        else:
            # Return code != 0 = errore
            error_msg = stderr_text or stdout_text or "Errore sconosciuto"
            app.logger.error(f"❌ Switch controller fallito: {error_msg}")
            app.logger.error(f"   Return code: {result.returncode}")
            app.logger.error(f"   Stdout: {stdout_text}")
            app.logger.error(f"   Stderr: {stderr_text}")
            return jsonify({"status": "error", "message": f"Switch controller fallito: {error_msg}"})
            
    except subprocess.TimeoutExpired:
        app.logger.error("❌ Timeout switch controller")
        return jsonify({"status": "error", "message": "Timeout switch controller (comando non risponde)"})
    except Exception as e:
        app.logger.error(f"❌ Errore switch controller: {e}")
        import traceback
        app.logger.error(traceback.format_exc())
        return jsonify({"status": "error", "message": f"Errore: {str(e)}"})


def main() -> None:
    """Entry point for running the Flask development server."""
    # Initialize ROS2 bridge automatically (starts 125Hz publishing loop)
    if ROS2_AVAILABLE:
        try:
            bridge = get_ros2_bridge()
            if bridge and bridge.ensure_ros():
                print("✅ ROS2 bridge initialized - publishing at 125Hz")
            else:
                print("⚠️ ROS2 bridge initialization failed")
        except Exception as e:
            print(f"⚠️ Failed to initialize ROS2 bridge: {e}")
    
    host = os.environ.get("WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("WEB_PORT", 8080))
    debug = bool(int(os.environ.get("WEB_DEBUG", "0")))
    print(f"🌐 Starting web interface on http://{host}:{port}")
    app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    main()

