"""
Diagnostic utilities for the remote UR control toolkit.

This script performs non-intrusive checks against the robot:
  1. Optional ICMP ping (if available on the platform).
  2. TCP connectivity test to the URScript port (default 30002).
  3. Attempts to send a harmless URScript snippet (textmsg only).

No motion commands are issued. Ideal for verifying that the robot is ready
to accept remote commands without moving it.
"""

from __future__ import annotations

import argparse
import platform
import socket
import subprocess
import sys
from dataclasses import dataclass
from typing import Optional

from .remote_ur_controller import RemoteURController


DEFAULT_TIMEOUT = 3.0


@dataclass
class DiagnosticResult:
    succeeded: bool
    detail: str


def run_ping(host: str, count: int = 3) -> DiagnosticResult:
    """
    Attempt to ping the robot. Skips on unsupported platforms (Windows without ping).
    """
    ping_cmd = None
    if platform.system().lower() == "windows":
        ping_cmd = ["ping", "-n", str(count), host]
    else:
        ping_cmd = ["ping", "-c", str(count), host]

    try:
        completed = subprocess.run(
            ping_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return DiagnosticResult(False, "ping command not available on this system")
    except Exception as exc:  # pragma: no cover - defensive
        return DiagnosticResult(False, f"ping failed: {exc}")

    if completed.returncode == 0:
        return DiagnosticResult(True, "Ping successful")
    return DiagnosticResult(False, f"Ping failed (rc={completed.returncode})")


def check_tcp_port(host: str, port: int, timeout: float) -> DiagnosticResult:
    """
    Check whether the robot's URScript TCP port is reachable.
    """
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return DiagnosticResult(True, "TCP port reachable")
    except Exception as exc:
        return DiagnosticResult(False, f"TCP connection failed: {exc}")


def send_noop_script(controller: RemoteURController) -> DiagnosticResult:
    """
    Send a harmless URScript snippet that only logs a text message.
    """
    script = (
        "def remote_diag():\n"
        '  textmsg("REMOTE_DIAGNOSTIC_PING")\n'
        "end\n"
        "remote_diag()\n"
    )

    try:
        controller.connect()
        controller._send_script(script, wait=False)  # pylint: disable=protected-access
        return DiagnosticResult(True, "URScript textmsg sent successfully")
    except Exception as exc:  # pragma: no cover - runtime failures
        return DiagnosticResult(False, f"Failed to send URScript: {exc}")
    finally:
        controller.close()


def diagnostics(robot_ip: str, port: int, timeout: float, skip_ping: bool) -> int:
    print(f"=== Remote UR Diagnostics ===")
    print(f"Robot IP     : {robot_ip}")
    print(f"URScript port: {port}")

    if not skip_ping:
        result = run_ping(robot_ip)
        print(f"[PING] {'OK' if result.succeeded else 'FAIL'} - {result.detail}")
    else:
        print("[PING] skipped")

    tcp_result = check_tcp_port(robot_ip, port, timeout)
    print(f"[TCP]  {'OK' if tcp_result.succeeded else 'FAIL'} - {tcp_result.detail}")
    if not tcp_result.succeeded:
        return 1

    controller = RemoteURController(robot_ip=robot_ip, port=port, socket_timeout=timeout)
    script_result = send_noop_script(controller)
    print(f"[SCRIPT] {'OK' if script_result.succeeded else 'FAIL'} - {script_result.detail}")
    return 0 if script_result.succeeded else 1


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Diagnose UR remote control connectivity")
    parser.add_argument("--robot-ip", default="192.168.10.194", help="UR robot IP address")
    parser.add_argument("--port", type=int, default=30002, help="URScript TCP port (default 30002)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Socket timeout seconds")
    parser.add_argument("--skip-ping", action="store_true", help="Skip ICMP ping test")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    exit_code = diagnostics(
        robot_ip=args.robot_ip,
        port=args.port,
        timeout=args.timeout,
        skip_ping=args.skip_ping,
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

