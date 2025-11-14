"""
Controller per loop continuo servoj sul robot UR.
Crea un programma URScript che gira continuamente sul robot e aggiorna solo le velocità.
"""

import socket
import time
from typing import Sequence

DEFAULT_PORT = 30002
DEFAULT_TIMEOUT = 5.0


class ServoLoopController:
    """
    Controller che mantiene un loop continuo servoj sul robot.
    Il programma URScript gira sul robot e aggiorniamo solo le velocità via socket.
    """
    
    def __init__(self, robot_ip: str, port: int = DEFAULT_PORT):
        self.robot_ip = robot_ip
        self.port = port
        self._sock: socket.socket | None = None
        self._loop_active = False
        
    def connect(self) -> None:
        """Connessione al robot."""
        if self._sock:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(DEFAULT_TIMEOUT)
        self._sock.connect((self.robot_ip, self.port))
        
    def start_servo_loop(self) -> None:
        """
        Avvia loop continuo servoj sul robot.
        Il programma gira sul robot e aggiorniamo solo le velocità.
        """
        if not self._sock:
            self.connect()
            
        # Programma URScript che gira continuamente
        # Sintassi corretta per URScript
        servo_program = (
            "global target_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
            "global servo_active = True\n"
            "\n"
            "def servo_loop():\n"
            "  while servo_active:\n"
            "    current = get_actual_joint_positions()\n"
            "    t = 0.008\n"
            "    target = [current[0] + target_velocities[0]*t, current[1] + target_velocities[1]*t, current[2] + target_velocities[2]*t, current[3] + target_velocities[3]*t, current[4] + target_velocities[4]*t, current[5] + target_velocities[5]*t]\n"
            "    servoj(target, t=0.008, lookahead_time=0.1, gain=300)\n"
            "    sync()\n"
            "  end\n"
            "end\n"
            "\n"
            "servo_loop()\n"
        )
        
        self._sock.sendall(servo_program.encode('utf-8'))
        time.sleep(0.5)  # Attesa avvio programma
        self._loop_active = True
        
    def update_velocities(self, velocities: Sequence[float]) -> None:
        """
        Aggiorna le velocità target nel loop continuo.
        Non crea nuove funzioni, aggiorna solo le variabili.
        """
        if not self._loop_active or not self._sock:
            return
            
        if len(velocities) != 6:
            raise ValueError("velocities must have 6 elements")
        
        # Aggiorna variabile globale sul robot
        update_script = (
            f"target_velocities = [{velocities[0]:.6f}, {velocities[1]:.6f}, "
            f"{velocities[2]:.6f}, {velocities[3]:.6f}, "
            f"{velocities[4]:.6f}, {velocities[5]:.6f}]\n"
        )
        
        self._sock.sendall(update_script.encode('utf-8'))
        
    def stop_loop(self) -> None:
        """Ferma il loop continuo."""
        if not self._loop_active or not self._sock:
            return
            
        stop_script = "servo_active = False\n"
        self._sock.sendall(stop_script.encode('utf-8'))
        time.sleep(0.2)
        
        # Invia stop
        stop_cmd = "def stop_cmd():\n  stopl(1.5)\nend\nstop_cmd()\n"
        self._sock.sendall(stop_cmd.encode('utf-8'))
        
        self._loop_active = False
        
    def close(self) -> None:
        """Chiude connessione."""
        if self._loop_active:
            self.stop_loop()
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self._sock.close()
            self._sock = None

