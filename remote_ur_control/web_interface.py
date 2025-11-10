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
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
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
      .layout {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
        gap: 16px;
      }
      @media (min-width: 768px) {
        .layout {
          grid-template-columns: repeat(2, 1fr);
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
        font-size: 14px;
        color: #333;
        text-align: center;
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
    </style>
  </head>
  <body>
    <h1>UR Remote Control</h1>
    <p class="lead">
      Imposta le posizioni joint o usa le frecce per aumentare/diminuire. Premi <strong>MoveJ</strong> per inviare il comando.
    </p>

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

      <div style="grid-column: 1 / -1; text-align: center;">
        <button type="button" class="secondary-btn" id="stop-joystick">Emergency Stop</button>
      </div>

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
    </div>

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

      // --- Joystick logic -------------------------------------------------
      const joystick = document.getElementById("joystick");
      const handle = document.getElementById("joystick-handle");
      const joyX = document.getElementById("joy-x");
      const joyY = document.getElementById("joy-y");
      const stopJoystickBtn = document.getElementById("stop-joystick");

      const JOY_MAX = 1.0;      // max linear velocity scaling (rad/s)
      const JOY_DEADZONE = 0.08;
      const JOY_CART_STEP = 0.0005; // 0.5mm - molto più lento e preciso
      const JOY_INTERVAL = 200; // ms - più frequente per fluidità

      let joystickActive = false;
      let joystickTimer = null;
      let joyVector = { x: 0, y: 0 };

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
        stopJointMotion();
      }

      function startJoystickLoop() {
        if (joystickTimer) return;
        joystickTimer = setInterval(async () => {
          const magnitude = Math.hypot(joyVector.x, joyVector.y);
          if (magnitude < JOY_DEADZONE) return;

          const cartesianMode = document.getElementById("cartesian-mode").checked;
          let speeds;
          let endpoint;

          if (cartesianMode) {
            // Cartesian: continuous velocity control (speedl)
            const velScale = 0.05; // 50mm/s max
            speeds = [
              joyVector.y * velScale,   // vx (m/s)
              joyVector.x * velScale,   // vy (m/s)
              0,                        // vz
              0,                        // wrx (rad/s)
              0,                        // wry
              0,                        // wrz
            ];
            endpoint = "/api/speedl";
          } else {
            // Joint space: continuous velocity control (speedj)
            speeds = [
              joyVector.y * JOY_MAX,
              joyVector.x * JOY_MAX,
              0,
              0,
              0,
              0,
            ];
            endpoint = "/api/speedj";
          }

          try {
            const payload = { 
              speeds, 
              duration: JOY_INTERVAL / 1000.0 + 0.1, 
              acceleration: 0.25
            };
            
            await fetch(endpoint, {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify(payload),
            });
            setStatus("Joystick command sent", true);
          } catch (err) {
            console.error(err);
            setStatus("Joystick error: " + err, false);
          }
        }, JOY_INTERVAL);
      }

      function stopJoystickLoop() {
        if (joystickTimer) {
          clearInterval(joystickTimer);
          joystickTimer = null;
        }
      }

      async function stopJointMotion() {
        stopJoystickLoop();
        joyX.textContent = "0.00";
        joyY.textContent = "0.00";
        try {
          await fetch("/api/stop", { method: "POST" });
          setStatus("Stop command sent", true);
        } catch (err) {
          setStatus("Stop error: " + err, false);
        }
      }

      function onJoystickStart(evt) {
        joystickActive = true;
        startJoystickLoop();
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
        resetJoystick();
        resetJoystick2();
      });

      // --- Second Joystick (Z + Rotation) ------------------------------------
      const joystick2 = document.getElementById("joystick2");
      const handle2 = document.getElementById("joystick2-handle");
      const joy2X = document.getElementById("joy2-x");
      const joy2Y = document.getElementById("joy2-y");

      let joystick2Active = false;
      let joystick2Timer = null;
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
        stopJointMotion();
      }

      function startJoystick2Loop() {
        if (joystick2Timer) return;
        joystick2Timer = setInterval(async () => {
          const magnitude = Math.hypot(joy2Vector.x, joy2Vector.y);
          if (magnitude < JOY_DEADZONE) return;

          const cartesianMode = document.getElementById("cartesian-mode").checked;
          if (!cartesianMode) return;

          // Secondo joystick: Y = Z (su/giù), X = Rotazione Z
          const velScale = 0.05; // 50mm/s max linear
          const rotScale = 0.3;  // rad/s max rotation
          const speeds = [
            0,                              // vx
            0,                              // vy
            -joy2Vector.y * velScale,       // vz (su=+, giù=-)
            0,                              // wrx
            0,                              // wry
            joy2Vector.x * rotScale,        // wrz (rotazione)
          ];

          try {
            await fetch("/api/speedl", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ 
                speeds, 
                duration: JOY_INTERVAL / 1000.0 + 0.1, 
                acceleration: 0.25
              }),
            });
            setStatus("Joystick Z/Rot command sent", true);
          } catch (err) {
            console.error(err);
            setStatus("Joystick2 error: " + err, false);
          }
        }, JOY_INTERVAL);
      }

      function stopJoystick2Loop() {
        if (joystick2Timer) {
          clearInterval(joystick2Timer);
          joystick2Timer = null;
        }
      }

      function onJoystick2Start(evt) {
        joystick2Active = true;
        startJoystick2Loop();
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
      }

      function onJoystick2End() {
        joystick2Active = false;
        stopJoystick2Loop();
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

