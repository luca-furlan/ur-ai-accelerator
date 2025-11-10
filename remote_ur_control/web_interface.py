"""
Simple web interface for driving the UR robot from the AI Accelerator.

Run with:
    export UR_ROBOT_IP=192.168.10.194  # adjust to your robot
    python -m remote_ur_control.web_interface
"""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass
from typing import Dict, List

from flask import Flask, jsonify, render_template_string, request

from .remote_ur_controller import MoveParameters, RemoteURController

app = Flask(__name__)


@dataclass
class ControllerConfig:
    robot_ip: str
    port: int = 30002


HTML_TEMPLATE = """
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>UR Remote Control</title>
    <style>
      :root {
        color-scheme: light dark;
        --primary: #0b5394;
        --accent: #38761d;
        --danger: #990000;
        --border: rgba(0, 0, 0, 0.2);
        --bg: rgba(0, 0, 0, 0.03);
      }
      body {
        font-family: Arial, sans-serif;
        margin: 32px;
        max-width: 960px;
      }
      h1 {
        color: var(--primary);
        margin-bottom: 4px;
      }
      p.lead {
        margin-top: 0;
        color: #555;
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
        background: var(--accent);
        color: white;
      }
      .primary-btn:hover {
        background: #2c6e14;
      }
      .secondary-btn {
        background: white;
        color: var(--danger);
        border-color: var(--danger);
      }
      .secondary-btn:hover {
        background: #ffe5e5;
      }
      .status-bar {
        margin-top: 24px;
        padding: 12px 16px;
        border-radius: 6px;
        border: 1px solid var(--border);
        background: white;
        font-weight: 600;
      }
      .status-bar span.ready { color: var(--accent); }
      .status-bar span.error { color: var(--danger); }
      @media (max-width: 480px) {
        body { margin: 16px; }
        input[type="number"] { width: 100%; }
      }
    </style>
  </head>
  <body>
    <h1>UR Remote Control</h1>
    <p class="lead">
      Imposta le posizioni joint o usa le frecce per aumentare/diminuire. Premi <strong>MoveJ</strong> per inviare il comando.
    </p>

    <form id="move-form">
      <fieldset>
        <legend>Joint targets (radians)</legend>
        <div class="parameters">
          <label>Step size (rad)
            <input type="number" step="0.01" name="step" id="step-size" value="0.10">
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

    <div id="status" class="status-bar">
      Stato: <span class="ready">Ready</span>
    </div>

    <script>
      const status = document.getElementById("status");
      const form = document.getElementById("move-form");
      const stepInput = document.getElementById("step-size");
      const jointInputs = Array.from(form.querySelectorAll("input[name^='joint']"));

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
    </script>
  </body>
</html>
"""


def get_controller() -> RemoteURController:
    config = load_config()
    return RemoteURController(config.robot_ip, config.port)


def load_config() -> ControllerConfig:
    robot_ip = os.environ.get("UR_ROBOT_IP")
    if not robot_ip:
        raise RuntimeError("Set UR_ROBOT_IP env var with the robot IP address")
    port = int(os.environ.get("UR_ROBOT_PORT", 30002))
    return ControllerConfig(robot_ip=robot_ip, port=port)


def parse_joints(payload: Dict[str, str]) -> List[float]:
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


@app.route("/api/config", methods=["GET"])
def api_config():
    config = load_config()
    return jsonify({"status": "ok", "config": asdict(config)})


def main() -> None:
    """Entry point for running the Flask development server."""
    host = os.environ.get("WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("WEB_PORT", 8080))
    debug = bool(int(os.environ.get("WEB_DEBUG", "0")))
    app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    main()

