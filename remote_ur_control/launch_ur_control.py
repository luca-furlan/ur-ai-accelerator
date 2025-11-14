"""
Unified launcher for the Remote UR Control toolkit.

This entry point performs a quick connectivity diagnostic against the robot and,
if successful, boots the Flask web interface so that operators can immediately
teleoperate the UR arm from a browser.

Typical usage:

    python -m remote_ur_control.launch_ur_control --robot-ip 192.168.10.194

Environment variables (if already exported) still apply:
  * UR_ROBOT_IP – overrides the default robot IP address.
  * WEB_HOST, WEB_PORT, WEB_DEBUG – forwarded to the Flask app.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

from . import diagnostics, web_interface


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    default_robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    parser = argparse.ArgumentParser(
        description="Run UR diagnostics and launch the web control interface"
    )
    parser.add_argument(
        "--robot-ip",
        default=default_robot_ip,
        help="UR robot IP address (default: env UR_ROBOT_IP or 192.168.10.194)",
    )
    parser.add_argument(
        "--ur-port",
        type=int,
        default=30002,
        help="URScript TCP port used for diagnostics (default 30002)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=diagnostics.DEFAULT_TIMEOUT,
        help=f"Socket timeout for diagnostics in seconds (default {diagnostics.DEFAULT_TIMEOUT})",
    )
    parser.add_argument(
        "--skip-ping",
        action="store_true",
        help="Skip the ICMP ping during diagnostics",
    )
    parser.add_argument(
        "--web-host",
        default=os.environ.get("WEB_HOST", "0.0.0.0"),
        help="Host/IP for the web UI (default: env WEB_HOST or 0.0.0.0)",
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=int(os.environ.get("WEB_PORT", "8080")),
        help="Port for the web UI (default: env WEB_PORT or 8080)",
    )
    parser.add_argument(
        "--web-debug",
        type=int,
        choices=(0, 1),
        default=int(os.environ.get("WEB_DEBUG", "0")),
        help="Enable Flask debug mode (0/1). Default: env WEB_DEBUG or 0",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = _parse_args(argv)

    if not args.robot_ip:
        raise SystemExit("Robot IP must be provided via --robot-ip or UR_ROBOT_IP")

    print(">>> Running pre-flight diagnostics...")
    exit_code = diagnostics.diagnostics(
        robot_ip=args.robot_ip,
        port=args.ur_port,
        timeout=args.timeout,
        skip_ping=args.skip_ping,
    )
    if exit_code != 0:
        raise SystemExit(
            "Diagnostics failed. Resolve the issues above before launching the web UI."
        )

    # Forward configuration to the web interface via environment variables.
    os.environ["UR_ROBOT_IP"] = args.robot_ip
    os.environ["WEB_HOST"] = args.web_host
    os.environ["WEB_PORT"] = str(args.web_port)
    os.environ["WEB_DEBUG"] = str(args.web_debug)

    print(">>> Diagnostics OK. Starting the Remote UR web interface...")
    try:
        web_interface.main()
    except KeyboardInterrupt:
        print("\n>>> Remote UR web interface stopped by user.")


if __name__ == "__main__":
    main()








