"""
RTDE control manager that provides a singleton-style bridge for high-frequency
commands used by the web interface or other lightweight front-ends.

It wraps `RTDETeleop` and keeps a background loop alive so that joystick
commands can stream smooth `speedJ` updates without having to reconnect on each
HTTP request.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Iterable, Optional, Sequence

from .rtde_teleop import RTDEConfig, RTDETeleop

LOG = logging.getLogger(__name__)

DEFAULT_FREQUENCY = 125.0  # Hz
STREAM_INTERVAL = 0.02  # 50 Hz updates are usually sufficient
STREAM_DURATION = 0.08  # seconds for each speedJ command (overlaps interval)


class RTDEManager:
    """
    Thread-safe singleton that keeps an RTDE session alive.

    Attributes:
        robot_ip: current robot IP address.
        teleop: the underlying `RTDETeleop` instance.
    """

    _instance: Optional["RTDEManager"] = None
    _instance_lock = threading.Lock()

    def __init__(
        self,
        robot_ip: str,
        *,
        velocity: float = 0.2,
        acceleration: float = 0.7,
        frequency: float = DEFAULT_FREQUENCY,
    ) -> None:
        self.robot_ip = robot_ip
        self._config = RTDEConfig(
            robot_ip=robot_ip,
            velocity=velocity,
            acceleration=acceleration,
            frequency=frequency,
        )
        self._teleop = RTDETeleop(self._config)

        self._stream_lock = threading.Lock()
        self._stream_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._stream_enabled = False
        self._pending_speed: list[float] = [0.0] * 6

    # --------------------------------------------------------------------- #
    @classmethod
    def get(
        cls,
        robot_ip: str,
        *,
        velocity: float = 0.2,
        acceleration: float = 0.7,
        frequency: float = DEFAULT_FREQUENCY,
    ) -> "RTDEManager":
        """
        Retrieve (or create) the singleton manager.

        If the robot IP changes, the existing instance is shut down and a new
        one is created with the fresh configuration.
        """
        with cls._instance_lock:
            if cls._instance is None:
                LOG.info("RTDE manager: creating new session for %s", robot_ip)
                cls._instance = cls(
                    robot_ip,
                    velocity=velocity,
                    acceleration=acceleration,
                    frequency=frequency,
                )
            elif cls._instance.robot_ip != robot_ip:
                LOG.info(
                    "RTDE manager: robot IP changed (%s -> %s), rebuilding session",
                    cls._instance.robot_ip,
                    robot_ip,
                )
                cls._instance.shutdown()
                cls._instance = cls(
                    robot_ip,
                    velocity=velocity,
                    acceleration=acceleration,
                    frequency=frequency,
                )
        return cls._instance

    # ------------------------------------------------------------------ #
    def movej(self, target: Sequence[float], wait: bool = True) -> None:
        """Proxy to `RTDETeleop.movej`."""
        LOG.debug("RTDE manager: moveJ -> %s", target)
        self._teleop.movej(target, wait=wait)

    def speedj(self, joint_speeds: Iterable[float]) -> None:
        """
        Update the streaming target for joint velocities.
        """
        with self._stream_lock:
            speeds = list(joint_speeds)
            if len(speeds) != 6:
                raise ValueError("speedJ expects 6 joint velocities")
            self._pending_speed = speeds
            if not self._stream_enabled:
                LOG.debug("RTDE manager: enabling speedJ stream")
                self._stream_enabled = True
                self._ensure_thread()

    def halt(self) -> None:
        """Stop any motion immediately."""
        LOG.info("RTDE manager: halt requested")
        with self._stream_lock:
            self._stream_enabled = False
            self._pending_speed = [0.0] * 6
        self._teleop.halt()

    # ------------------------------------------------------------------ #
    def _ensure_thread(self) -> None:
        if self._stream_thread and self._stream_thread.is_alive():
            return
        self._stop_event.clear()
        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="rtde-speedj-stream",
            daemon=True,
        )
        self._stream_thread.start()

    def _stream_loop(self) -> None:
        LOG.info("RTDE manager: starting speedJ stream loop")
        try:
            while not self._stop_event.is_set():
                with self._stream_lock:
                    speeds = list(self._pending_speed)
                    streaming = self._stream_enabled
                if streaming:
                    try:
                        self._teleop.speedj(speeds, duration=STREAM_DURATION)
                    except Exception:  # pragma: no cover - runtime safety
                        LOG.exception("RTDE manager: speedJ streaming failed")
                        # Avoid busy-looping on repeated failures
                        time.sleep(STREAM_INTERVAL)
                else:
                    time.sleep(STREAM_INTERVAL)
                    continue
                time.sleep(STREAM_INTERVAL)
        finally:
            LOG.info("RTDE manager: exiting speedJ stream loop")

    # ------------------------------------------------------------------ #
    def shutdown(self) -> None:
        """Terminate the background stream and close RTDE sockets."""
        LOG.info("RTDE manager: shutting down")
        self._stop_event.set()
        with self._stream_lock:
            self._stream_enabled = False
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=1.0)
        self._teleop.close()


__all__ = ["RTDEManager"]




