#!/usr/bin/env python3
"""
Script completo per verificare lo stato di tutte le connessioni:
- Robot UR (ping, TCP, Dashboard Server)
- AI Accelerator (ping, SSH opzionale)
- Bridge ROS2
"""

import os
import sys
import socket
import subprocess
import platform
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass

# Configurazione da network_info.txt
ROBOT_IP = "192.168.10.194"
AI_ACCELERATOR_IP = "192.168.10.191"
ROBOT_SSH_USER = "root"
ROBOT_SSH_PASS = "easybot"
AI_ACCELERATOR_SSH_USER = "lab"
AI_ACCELERATOR_SSH_PASS = "easybot"

ROBOT_URSCRIPT_PORT = 30002
ROBOT_DASHBOARD_PORT = 29999
SSH_PORT = 22


@dataclass
class CheckResult:
    """Risultato di un controllo."""
    name: str
    success: bool
    message: str
    details: Optional[str] = None


def print_header(title: str):
    """Stampa un'intestazione formattata."""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def print_result(result: CheckResult):
    """Stampa un risultato formattato."""
    status = "✅ OK" if result.success else "❌ FAIL"
    print(f"[{status}] {result.name}")
    print(f"         {result.message}")
    if result.details:
        print(f"         Dettagli: {result.details}")


def check_ping(host: str, count: int = 3) -> CheckResult:
    """Verifica la connettività di rete tramite ping."""
    ping_cmd = None
    if platform.system().lower() == "windows":
        ping_cmd = ["ping", "-n", str(count), "-w", "2000", host]
    else:
        ping_cmd = ["ping", "-c", str(count), "-W", "2", host]
    
    try:
        result = subprocess.run(
            ping_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
            check=False
        )
        if result.returncode == 0:
            # Estrai statistiche dal ping
            lines = result.stdout.split('\n')
            stats = [l for l in lines if 'time=' in l or 'tempo=' in l or 'ms' in l]
            details = stats[0] if stats else None
            return CheckResult(
                name=f"Ping a {host}",
                success=True,
                message="Connessione di rete OK",
                details=details
            )
        else:
            return CheckResult(
                name=f"Ping a {host}",
                success=False,
                message=f"Ping fallito (codice: {result.returncode})"
            )
    except FileNotFoundError:
        return CheckResult(
            name=f"Ping a {host}",
            success=False,
            message="Comando ping non disponibile"
        )
    except subprocess.TimeoutExpired:
        return CheckResult(
            name=f"Ping a {host}",
            success=False,
            message="Timeout durante il ping"
        )
    except Exception as e:
        return CheckResult(
            name=f"Ping a {host}",
            success=False,
            message=f"Errore: {e}"
        )


def check_tcp_port(host: str, port: int, timeout: float = 3.0) -> CheckResult:
    """Verifica la connettività TCP a una porta specifica."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        
        if result == 0:
            return CheckResult(
                name=f"TCP {host}:{port}",
                success=True,
                message="Porta raggiungibile"
            )
        else:
            return CheckResult(
                name=f"TCP {host}:{port}",
                success=False,
                message=f"Porta non raggiungibile (codice: {result})"
            )
    except socket.timeout:
        return CheckResult(
            name=f"TCP {host}:{port}",
            success=False,
            message="Timeout durante la connessione"
        )
    except Exception as e:
        return CheckResult(
            name=f"TCP {host}:{port}",
            success=False,
            message=f"Errore: {e}"
        )


def check_robot_dashboard(robot_ip: str, port: int = ROBOT_DASHBOARD_PORT) -> CheckResult:
    """Verifica lo stato del robot tramite Dashboard Server."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((robot_ip, port))
        
        # Leggi messaggio di benvenuto
        welcome = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        
        # Invia comandi per ottenere lo stato
        commands = {
            "robotmode": "Robot Mode",
            "safetymode": "Safety Mode",
            "programState": "Program State",
            "is in remote control": "Remote Control"
        }
        
        results = {}
        for cmd, label in commands.items():
            try:
                sock.sendall((cmd + "\n").encode('utf-8'))
                time.sleep(0.15)  # Aumentato il delay per risposte più affidabili
                response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
                # Rimuovi prefissi comuni dalle risposte (es: "Robotmode: RUNNING" -> "RUNNING")
                response_clean = response
                for prefix in [label + ":", label.replace(" ", "").lower() + ":", cmd + ":", cmd.title() + ":"]:
                    if response_clean.lower().startswith(prefix.lower()):
                        response_clean = response_clean[len(prefix):].strip()
                results[label] = response_clean if response_clean else response
            except Exception as e:
                results[label] = f"ERRORE: {e}"
        
        sock.close()
        
        # Formatta i dettagli
        details = ", ".join([f"{k}: {v}" for k, v in results.items()])
        
        # Verifica se ci sono problemi - estrai solo il valore principale
        robotmode_raw = results.get("Robot Mode", "").lower()
        # Rimuovi prefissi comuni
        robotmode = robotmode_raw.replace("robotmode:", "").replace("robot mode:", "").strip()
        
        safetymode_raw = results.get("Safety Mode", "").lower()
        safetymode = safetymode_raw.replace("safetymode:", "").replace("safety mode:", "").strip()
        
        program_state_raw = results.get("Program State", "").lower()
        program_state = program_state_raw.replace("programstate:", "").replace("program state:", "").strip()
        
        remote_control_raw = results.get("Remote Control", "").lower()
        remote_control = remote_control_raw.replace("is in remote control:", "").replace("remote control:", "").strip()
        
        issues = []
        # Verifica robotmode - accetta "running" o "run"
        # NOTA: Il Dashboard Server potrebbe non essere sincronizzato, quindi consideriamo
        # anche il fatto che la porta URScript (30002) funzioni come indicatore di operatività
        if "running" not in robotmode and "run" not in robotmode:
            # Se il Dashboard dice POWER_OFF ma la porta URScript funziona, potrebbe essere un falso negativo
            issues.append(f"Dashboard dice non RUNNING (attuale: '{robotmode}') - potrebbe essere desincronizzato")
        # Verifica safetymode - accetta "normal"
        if "normal" not in safetymode:
            issues.append(f"Safety Mode non NORMAL (attuale: '{safetymode}')")
        # Verifica program_state - accetta "playing" o "play"
        if "playing" not in program_state and "play" not in program_state:
            issues.append(f"Dashboard dice programma non PLAYING (attuale: '{program_state}') - potrebbe essere desincronizzato")
        # Verifica remote_control - accetta "true" o "1"
        if "true" not in remote_control and "1" not in remote_control:
            issues.append(f"Dashboard dice Remote Control non attivo (attuale: '{remote_control}') - potrebbe essere desincronizzato")
        
        # Se il robot è in POWER_OFF, prova a vedere se possiamo accenderlo
        if "power_off" in robotmode:
            # Prova a inviare comando power on (solo se siamo sicuri che sia necessario)
            # NOTA: Non lo facciamo automaticamente per sicurezza
            pass
        
        # Se ci sono problemi ma la porta URScript funziona, potrebbe essere solo desincronizzazione
        # Verifica se la porta URScript è raggiungibile (sarà verificata dopo)
        urscript_works = False  # Sarà impostato dopo il controllo URScript
        
        if issues:
            # Se il Dashboard dice POWER_OFF ma la porta URScript funziona, è probabilmente un falso negativo
            warning_note = ""
            if "power_off" in robotmode.lower() or "power-off" in robotmode.lower():
                warning_note = " | NOTA: Se la porta URScript (30002) funziona, il robot potrebbe essere operativo nonostante il Dashboard mostri POWER_OFF"
            
            return CheckResult(
                name="Dashboard Server Robot",
                success=False,
                message="Dashboard mostra stato non ottimale (potrebbe essere desincronizzato)",
                details=f"{details} | Problemi: {'; '.join(issues)}{warning_note}"
            )
        else:
            return CheckResult(
                name="Dashboard Server Robot",
                success=True,
                message="Robot pronto e operativo (secondo Dashboard)",
                details=details
            )
            
    except socket.timeout:
        return CheckResult(
            name="Dashboard Server Robot",
            success=False,
            message="Timeout durante la connessione"
        )
    except Exception as e:
        return CheckResult(
            name="Dashboard Server Robot",
            success=False,
            message=f"Errore: {e}"
        )


def check_robot_script_port(robot_ip: str, port: int = ROBOT_URSCRIPT_PORT) -> CheckResult:
    """Verifica la porta URScript e invia uno script di test."""
    try:
        from remote_ur_control.remote_ur_controller import RemoteURController
        
        controller = RemoteURController(robot_ip=robot_ip, port=port, socket_timeout=3.0)
        try:
            controller.connect()
            
            # Invia uno script innocuo
            script = (
                "def remote_diag():\n"
                '  textmsg("CONNECTION_TEST")\n'
                "end\n"
                "remote_diag()\n"
            )
            controller._send_script(script, wait=False)
            
            return CheckResult(
                name="URScript Port (30002)",
                success=True,
                message="Porta raggiungibile e script eseguito"
            )
        finally:
            controller.close()
            
    except ImportError:
        return CheckResult(
            name="URScript Port (30002)",
            success=False,
            message="Modulo remote_ur_controller non disponibile"
        )
    except Exception as e:
        return CheckResult(
            name="URScript Port (30002)",
            success=False,
            message=f"Errore: {e}"
        )


def check_ros2_bridge() -> CheckResult:
    """Verifica lo stato del bridge ROS2."""
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from ros2_bridge_fixed import ROS2Bridge, ROS2_AVAILABLE
        
        if not ROS2_AVAILABLE:
            return CheckResult(
                name="ROS2 Bridge",
                success=False,
                message="ROS2 non disponibile (rclpy non importabile)"
            )
        
        bridge = ROS2Bridge()
        if bridge.ensure_ros():
            running = bridge._running if hasattr(bridge, '_running') else False
            thread_alive = bridge._publish_thread.is_alive() if bridge._publish_thread else False
            
            details = f"Running: {running}, Thread: {thread_alive}"
            
            if running and thread_alive:
                return CheckResult(
                    name="ROS2 Bridge",
                    success=True,
                    message="Bridge ROS2 inizializzato e attivo",
                    details=details
                )
            else:
                return CheckResult(
                    name="ROS2 Bridge",
                    success=False,
                    message="Bridge ROS2 inizializzato ma non completamente attivo",
                    details=details
                )
        else:
            return CheckResult(
                name="ROS2 Bridge",
                success=False,
                message="Bridge ROS2 non inizializzato"
            )
            
    except ImportError as e:
        return CheckResult(
            name="ROS2 Bridge",
            success=False,
            message=f"Impossibile importare ros2_bridge_fixed: {e}"
        )
    except Exception as e:
        return CheckResult(
            name="ROS2 Bridge",
            success=False,
            message=f"Errore: {e}"
        )


def check_ssh(host: str, user: str, password: str, port: int = SSH_PORT) -> CheckResult:
    """Verifica la connessione SSH (opzionale, richiede paramiko)."""
    try:
        import paramiko
    except ImportError:
        return CheckResult(
            name=f"SSH {user}@{host}",
            success=None,  # None = skipped
            message="Paramiko non installato (controllo SSH saltato)"
        )
    
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(
            hostname=host,
            port=port,
            username=user,
            password=password,
            timeout=5,
            look_for_keys=False,
            allow_agent=False
        )
        
        # Esegui un comando semplice
        stdin, stdout, stderr = ssh.exec_command("hostname")
        hostname = stdout.read().decode('utf-8').strip()
        ssh.close()
        
        return CheckResult(
            name=f"SSH {user}@{host}",
            success=True,
            message="Connessione SSH OK",
            details=f"Hostname: {hostname}"
        )
    except paramiko.AuthenticationException:
        return CheckResult(
            name=f"SSH {user}@{host}",
            success=False,
            message="Autenticazione SSH fallita"
        )
    except socket.timeout:
        return CheckResult(
            name=f"SSH {user}@{host}",
            success=False,
            message="Timeout durante la connessione SSH"
        )
    except Exception as e:
        return CheckResult(
            name=f"SSH {user}@{host}",
            success=False,
            message=f"Errore SSH: {e}"
        )


def main():
    """Esegue tutti i controlli di connessione."""
    print_header("CONTROLLO STATO CONNESSIONI - Sistema Robot e AI Accelerator")
    
    results = []
    
    # 1. Connessioni di rete
    print_header("1. CONNESSIONI DI RETE")
    
    print("\n[ROBOT]")
    results.append(check_ping(ROBOT_IP))
    print_result(results[-1])
    
    results.append(check_tcp_port(ROBOT_IP, ROBOT_URSCRIPT_PORT))
    print_result(results[-1])
    
    results.append(check_tcp_port(ROBOT_IP, ROBOT_DASHBOARD_PORT))
    print_result(results[-1])
    
    print("\n[AI ACCELERATOR]")
    results.append(check_ping(AI_ACCELERATOR_IP))
    print_result(results[-1])
    
    results.append(check_tcp_port(AI_ACCELERATOR_IP, SSH_PORT))
    print_result(results[-1])
    
    # 2. Stato Robot
    print_header("2. STATO ROBOT (Dashboard Server)")
    results.append(check_robot_dashboard(ROBOT_IP))
    print_result(results[-1])
    
    # 3. Porta URScript
    print_header("3. PORTA URSCRIPT (30002)")
    results.append(check_robot_script_port(ROBOT_IP))
    print_result(results[-1])
    
    # 4. Bridge ROS2
    print_header("4. BRIDGE ROS2")
    results.append(check_ros2_bridge())
    print_result(results[-1])
    
    # 5. Connessioni SSH (opzionali)
    print_header("5. CONNESSIONI SSH (Opzionali)")
    ssh_result_robot = check_ssh(ROBOT_IP, ROBOT_SSH_USER, ROBOT_SSH_PASS)
    if ssh_result_robot.success is not None:
        results.append(ssh_result_robot)
        print_result(ssh_result_robot)
    else:
        print_result(ssh_result_robot)
    
    ssh_result_ai = check_ssh(AI_ACCELERATOR_IP, AI_ACCELERATOR_SSH_USER, AI_ACCELERATOR_SSH_PASS)
    if ssh_result_ai.success is not None:
        results.append(ssh_result_ai)
        print_result(ssh_result_ai)
    else:
        print_result(ssh_result_ai)
    
    # Riepilogo finale
    print_header("RIEPILOGO FINALE")
    
    successful = [r for r in results if r.success is True]
    failed = [r for r in results if r.success is False]
    skipped = [r for r in results if r.success is None]
    
    print(f"\n✅ Controlli riusciti: {len(successful)}/{len(results)}")
    print(f"❌ Controlli falliti: {len(failed)}/{len(results)}")
    if skipped:
        print(f"⏭️  Controlli saltati: {len(skipped)}")
    
    if failed:
        print("\n⚠️  PROBLEMI RILEVATI:")
        for r in failed:
            print(f"   - {r.name}: {r.message}")
    
    if not failed:
        print("\n🎉 Tutti i controlli sono passati con successo!")
    
    print("\n" + "=" * 70)
    
    return 0 if len(failed) == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

