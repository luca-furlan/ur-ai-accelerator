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
      body { font-family: Arial, sans-serif; margin: 40px; }
      h1 { color: #0b5394; }
      label { display: block; margin-top: 12px; }
      input[type="number"] { width: 120px; padding: 4px; }
      .row { margin-bottom: 6px; }
      .actions { margin-top: 20px; }
      button { padding: 10px 18px; margin-right: 12px; }
      #status { margin-top: 20px; font-weight: bold; }
    </style>
  </head>
  <body>
    <h1>UR Remote Control</h1>
    <form id="move-form">
      <fieldset>
        <legend>Joint targets (radians)</legend>
        {% for idx in range(6) %}
          <div class="row">
            <label>Joint {{ idx + 1 }}:
              <input type="number" step="0.01" name="joint{{ idx }}" value="0.0">
            </label>
          </div>
        {% endfor %}
      </fieldset>

      <fieldset>
        <legend>Motion parameters</legend>
        <label>Acceleration:
          <input type="number" step="0.1" name="acceleration" value="1.2">
        </label>
        <label>Velocity:
          <input type="number" step="0.05" name="velocity" value="0.25">
        </label>
        <label>Blend radius:
          <input type="number" step="0.01" name="blend_radius" value="0.0">
        </label>
        <label>
          <input type="checkbox" name="async_move"> Non blocking
        </label>
      </fieldset>

      <div class="actions">
        <button type="submit">MoveJ</button>
        <button type="button" onclick="sendStop()">Stop</button>
      </div>
    </form>

    <div id="status">Ready</div>

    <script>
      const status = document.getElementById("status");
      const form = document.getElementById("move-form");

      form.addEventListener("submit", async (event) => {
        event.preventDefault();
        const formData = new FormData(form);
        const payload = {};

        for (let [key, value] of formData.entries()) {
          if (key.startsWith("joint")) {
            payload[key] = parseFloat(value);
          }
        }

        payload["acceleration"] = parseFloat(formData.get("acceleration"));
        payload["velocity"] = parseFloat(formData.get("velocity"));
        payload["blend_radius"] = parseFloat(formData.get("blend_radius"));
        payload["async_move"] = formData.get("async_move") === "on";

        setStatus("Sending command…");
        try {
          const response = await fetch("/api/movej", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
          });
          const data = await response.json();
          setStatus(data.message);
        } catch (err) {
          console.error(err);
          setStatus("Error: " + err);
        }
      });

      async function sendStop() {
        setStatus("Sending stop…");
        try {
          const response = await fetch("/api/stop", { method: "POST" });
          const data = await response.json();
          setStatus(data.message);
        } catch (err) {
          console.error(err);
          setStatus("Error: " + err);
        }
      }

      function setStatus(text) {
        status.innerText = text;
      }
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

