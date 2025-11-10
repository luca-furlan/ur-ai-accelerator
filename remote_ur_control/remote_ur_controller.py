"""
Remote UR controller utilities.

This module provides a minimal socket-based interface that can be executed on
the AI Accelerator to drive a Universal Robots arm through URScript commands.

The controller opens a TCP connection (default port 30002) to the robot and
sends small URScript snippets to trigger motion primitives.
"""

from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

DEFAULT_PORT = 30002
DEFAULT_TIMEOUT = 5.0

LOG = logging.getLogger(__name__)


@dataclass
class MoveParameters:
    """Parameters for a UR move command."""

    acceleration: float = 1.2
    velocity: float = 0.25
    blend_radius: float = 0.0
    async_move: bool = False


class RemoteURController:
    """
    Minimal socket client that sends URScript commands to a robot.

    Usage:
        controller = RemoteURController("192.168.10.194")
        controller.connect()
        controller.movej([0, -1.57, 1.57, 0, 1.57, 0])
    """

    def __init__(
        self,
        robot_ip: str,
        port: int = DEFAULT_PORT,
        socket_timeout: float = DEFAULT_TIMEOUT,
    ):
        self.robot_ip = robot_ip
        self.port = port
        self.socket_timeout = socket_timeout
        self._sock: socket.socket | None = None

    # --------------------------------------------------------------------- #
    # Connection management
    # --------------------------------------------------------------------- #
    def connect(self) -> None:
        """Establish the TCP connection to the robot."""
        if self._sock:
            return

        LOG.debug("Connecting to UR robot %s:%s", self.robot_ip, self.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.socket_timeout)
        sock.connect((self.robot_ip, self.port))
        self._sock = sock
        LOG.info("Connected to UR robot at %s:%s", self.robot_ip, self.port)

    def close(self) -> None:
        """Close the current socket connection."""
        if self._sock:
            LOG.debug("Closing UR robot connection")
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    # --------------------------------------------------------------------- #
    # Motion primitives
    # --------------------------------------------------------------------- #
    def movej(
        self,
        joints: Sequence[float],
        params: MoveParameters | None = None,
    ) -> None:
        """
        Joint movement using URScript movej command.

        Args:
            joints: target joint positions (radians) length 6.
            params: optional motion parameters.
        """
        if len(joints) != 6:
            raise ValueError("movej expects 6 joint targets")

        params = params or MoveParameters()
        script = (
            "def remote_move():\n"
            f"  movej({self._format_list(joints)}, a={params.acceleration}, "
            f"v={params.velocity}, r={params.blend_radius})\n"
            "end\n"
            "remote_move()\n"
        )
        self._send_script(script, wait=not params.async_move)

    def speedj(
        self,
        joint_speeds: Sequence[float],
        duration: float = 1.0,
        acceleration: float = 1.0,
    ) -> None:
        """
        Joint speed command.

        Args:
            joint_speeds: speed for each joint (rad/s) length 6.
            duration: command duration in seconds.
            acceleration: joint acceleration limit.
        """
        if len(joint_speeds) != 6:
            raise ValueError("speedj expects 6 joint speeds")

        script = (
            "def remote_speed():\n"
            f"  speedj({self._format_list(joint_speeds)}, {acceleration}, {duration})\n"
            "end\n"
            "remote_speed()\n"
        )
        self._send_script(script, wait=True)

    def speedl(
        self,
        cart_speeds: list[float],
        acceleration: float = 1.0,
        duration: float = 0.3,
    ) -> None:
        """Send a speedl command for cartesian velocity control.
        
        Args:
            cart_speeds: [vx, vy, vz, wx, wy, wz] in m/s and rad/s
            acceleration: acceleration in m/s²
            duration: time buffer for command in seconds
        """
        if len(cart_speeds) != 6:
            raise ValueError("speedl expects 6 cartesian speeds [vx,vy,vz,wx,wy,wz]")

        script = (
            "def remote_speedl():\n"
            f"  speedl({self._format_list(cart_speeds)}, {acceleration}, {duration})\n"
            "end\n"
            "remote_speedl()\n"
        )
        self._send_script(script, wait=True)

    def movel_relative(
        self,
        delta_pose: list[float],
        acceleration: float = 0.5,
        velocity: float = 0.1,
    ) -> None:
        """Move tool by relative cartesian offset.
        
        Args:
            delta_pose: [dx, dy, dz, drx, dry, drz] relative to current TCP
            acceleration: acceleration in m/s²
            velocity: velocity in m/s
        """
        if len(delta_pose) != 6:
            raise ValueError("movel_relative expects 6 values [dx,dy,dz,drx,dry,drz]")

        script = (
            "def remote_movel_rel():\n"
            "  current = get_actual_tcp_pose()\n"
            f"  target = pose_add(current, p{self._format_list(delta_pose)})\n"
            f"  movel(target, a={acceleration}, v={velocity})\n"
            "end\n"
            "remote_movel_rel()\n"
        )
        self._send_script(script, wait=False)

    def stop(self) -> None:
        """Send a stopj command to halt motion."""
        script = "def remote_stop():\n  stopj(2.0)\nend\nremote_stop()\n"
        self._send_script(script, wait=False)

    # --------------------------------------------------------------------- #
    # Helpers
    # --------------------------------------------------------------------- #
    def _send_script(self, script: str, wait: bool) -> None:
        """Send raw URScript ensuring connection is open."""
        for attempt in range(2):
            try:
                if not self._sock:
                    self.connect()
                assert self._sock is not None
                LOG.debug("Sending URScript (attempt %d):\n%s", attempt + 1, script)
                self._sock.sendall(script.encode("utf-8"))

                if wait:
                    time.sleep(0.1)
                return
            except (socket.timeout, OSError) as exc:
                LOG.warning("URScript send failed (%s), reconnecting...", exc)
                self.close()
                if attempt == 1:
                    raise
                time.sleep(0.05)
        raise RuntimeError("Failed to send URScript after retries")

    @staticmethod
    def _format_list(values: Iterable[float]) -> str:
        return "[" + ", ".join(f"{v:.5f}" for v in values) + "]"


__all__ = ["RemoteURController", "MoveParameters"]

