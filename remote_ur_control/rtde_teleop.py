"""
RTDE-based teleoperation helpers for Universal Robots.

This module relies on the official ur_rtde SDK, which is the recommended
interface by Universal Robots for real-time streaming control (ROS 2 driver and
MoveIt use the same backend). It provides:

* A minimalist CLI to send joint jogs (`speedJ`) or position moves (`moveJ`).
* A reusable `RTDETeleop` class that other apps (ROS 2 nodes, web servers) can
  import to take advantage of the reliable RTDE transport.

The code never issues aggressive trajectories; default commands are short,
low-speed nudges suitable for manual teleop.

Example usage (AI Accelerator):
    source ~/.venvs/ur-remote/bin/activate
    python -m remote_ur_control.rtde_teleop --robot-ip 192.168.10.194 jog --joint 0 --delta 0.1
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

try:
    from ur_rtde import rtde_control, rtde_receive
except ImportError as exc:  # pragma: no cover - runtime only
    raise RuntimeError(
        "ur-rtde is required. Install with: pip install ur-rtde"
    ) from exc

LOG = logging.getLogger(__name__)


@dataclass
class RTDEConfig:
    robot_ip: str
    frequency: float = 125.0  # Hz for RTDE samples
    velocity: float = 0.20
    acceleration: float = 0.7
    step_duration: float = 0.6


class RTDETeleop:
    """
    Helper around `ur_rtde` jog commands.

    Uses context management to ensure sockets are closed cleanly.
    """

    def __init__(self, config: RTDEConfig) -> None:
        self.config = config
        self._ctrl = rtde_control.RTDEControlInterface(config.robot_ip)
        self._recv = rtde_receive.RTDEReceiveInterface(
            config.robot_ip, frequency=config.frequency
        )

    # ------------------------------------------------------------------ #
    def __enter__(self) -> "RTDETeleop":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------------------------------------------------------ #
    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._ctrl.stopScript()
        with contextlib.suppress(Exception):
            self._ctrl.disconnect()
        with contextlib.suppress(Exception):
            self._recv.disconnect()

    # ------------------------------------------------------------------ #
    def get_joints(self) -> Sequence[float]:
        return self._recv.getActualQ()

    def get_tcp_pose(self) -> Sequence[float]:
        return self._recv.getActualTCPPose()

    # ------------------------------------------------------------------ #
    def movej(self, target: Sequence[float], *, wait: bool = True) -> None:
        LOG.info("movej -> %s", ["%.3f" for _ in target])
        self._ctrl.moveJ(
            target,
            self.config.velocity,
            self.config.acceleration,
            asynchronous=not wait,
        )
        if wait:
            self._ctrl.waitJointsDone(timeout=10.0)

    def jog_joint(self, joint_index: int, delta: float) -> None:
        current = list(self.get_joints())
        if not 0 <= joint_index < len(current):
            raise IndexError("joint index out of range")
        current[joint_index] += delta
        LOG.info("Jog joint %d by %.3f rad", joint_index + 1, delta)
        self.movej(current)

    def speedj(self, joint_speeds: Sequence[float], duration: float | None = None) -> None:
        duration = duration or self.config.step_duration
        LOG.info("speedj -> %s for %.2fs", joint_speeds, duration)
        self._ctrl.speedJ(
            joint_speeds,
            self.config.acceleration,
            duration,
        )

    # ------------------------------------------------------------------ #
    def joystick_vector(self, vx: float, vy: float, max_speed: float = 1.0) -> None:
        """
        Map joystick X/Y to shoulder/elbow joints. This is intentionally conservative.
        """
        joint_speeds = [0.0] * 6
        joint_speeds[0] = vy * max_speed
        joint_speeds[1] = vx * max_speed
        self.speedj(joint_speeds)

    def halt(self) -> None:
        LOG.info("Sending stopJ")
        self._ctrl.stopJ(self.config.acceleration)


# ==============================================================================
# CLI
# ==============================================================================

def _setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="[%(asctime)s] %(levelname)s: %(message)s",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RTDE teleoperation utilities")
    parser.add_argument("--robot-ip", required=True, help="UR controller IP")
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--acceleration", type=float, default=0.7)
    parser.add_argument("--frequency", type=float, default=125.0)
    parser.add_argument("--step-duration", type=float, default=0.6)
    parser.add_argument("-v", "--verbose", action="store_true")

    subparsers = parser.add_subparsers(dest="command", required=True)

    jog_parser = subparsers.add_parser("jog", help="Jog a single joint using moveJ")
    jog_parser.add_argument("--joint", type=int, required=True, help="Joint index (0-5)")
    jog_parser.add_argument("--delta", type=float, default=0.1, help="Radians delta")

    speed_parser = subparsers.add_parser("speed", help="Send raw speedJ vector")
    speed_parser.add_argument(
        "--speeds",
        type=float,
        nargs=6,
        required=True,
        metavar=("q1", "q2", "q3", "q4", "q5", "q6"),
    )
    speed_parser.add_argument("--duration", type=float, default=0.6)

    subparsers.add_parser("halt", help="Send stopJ command")

    state_parser = subparsers.add_parser("state", help="Print current joint/tcp state")
    state_parser.add_argument("--watch", type=float, default=0.0, help="Hz refresh rate")

    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _setup_logging(args.verbose)

    config = RTDEConfig(
        robot_ip=args.robot_ip,
        velocity=args.velocity,
        acceleration=args.acceleration,
        frequency=args.frequency,
        step_duration=args.step_duration,
    )

    try:
        with RTDETeleop(config) as teleop:
            if args.command == "jog":
                teleop.jog_joint(args.joint, args.delta)
            elif args.command == "speed":
                teleop.speedj(args.speeds, args.duration)
            elif args.command == "halt":
                teleop.halt()
            elif args.command == "state":
                if args.watch > 0:
                    interval = 1.0 / args.watch
                    while True:
                        joints = teleop.get_joints()
                        tcp = teleop.get_tcp_pose()
                        print("Joints:", [f"{j:.3f}" for j in joints])
                        print("TCP   :", [f"{p:.3f}" for p in tcp])
                        time.sleep(interval)
                else:
                    joints = teleop.get_joints()
                    tcp = teleop.get_tcp_pose()
                    print("Joints:", [f"{j:.3f}" for j in joints])
                    print("TCP   :", [f"{p:.3f}" for p in tcp])
            else:  # pragma: no cover - argparser enforces
                parser.error(f"Unsupported command {args.command}")
    except KeyboardInterrupt:  # pragma: no cover - runtime convenience
        print("Interrupted by user")
        return 130
    except Exception as exc:  # pragma: no cover - runtime convenience
        LOG.error("RTDE teleop failed: %s", exc, exc_info=args.verbose)
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())

