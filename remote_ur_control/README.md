# Remote UR Control Toolkit

Script e interfaccia web da eseguire sull'AI Accelerator per comandare il robot UR in rete.

## Contenuto

- `remote_ur_controller.py`: client socket minimale che invia comandi URScript (movej, speedj, stop).
- `web_interface.py`: applicazione Flask con UI avanzata (frecce ±, step configurabile, joystick virtuale con `speedj`, stato live) per comandare il robot da browser.
- `requirements.txt`: dipendenze Python (Flask).
- `diagnostics.py`: script CLI che verifica ping, porta TCP e invio script “textmsg” (nessun movimento).
- `rtde_teleop.py`: teleop di riferimento basato su `ur_rtde`, lo stesso backend usato dal driver ROS 2 ufficiale.

## Setup rapido

1. **Installazione dipendenze (AI Accelerator)**
   ```bash
   python3 -m venv ~/.venvs/ur-remote && source ~/.venvs/ur-remote/bin/activate
   pip install -r remote_ur_control/requirements.txt
   ```

2. **Esportare la configurazione della rete**
   ```bash
   export UR_ROBOT_IP=192.168.10.194   # IP del robot
   export WEB_HOST=0.0.0.0             # opzionale: espone la UI in LAN
   export WEB_PORT=8080                # porta web (default 8080)
   ```

3. **Avviare tutto con un unico comando (diagnostica + web UI)**
   ```bash
   python -m remote_ur_control.launch_ur_control --robot-ip 192.168.10.194
   ```
   Il comando:
   - esegue la diagnostica di rete (`ping`, apertura porta 30002, invio `textmsg`);
   - esporta automaticamente le variabili `UR_ROBOT_IP`, `WEB_HOST`, `WEB_PORT`;
   - avvia il server Flask dell'interfaccia web.

   Opzioni utili:
   - `--skip-ping` se il ping ICMP è bloccato sulla rete;
   - `--web-host 0.0.0.0` e `--web-port 8080` (default) per esporre nella LAN;
   - `--web-debug 1` per attivare il reload automatico in sviluppo.

   > Preferisci avviare manualmente solo la web UI? Usa `python -m remote_ur_control.web_interface`.

4. **Aprire il browser remoto**
   Visitare `http://<AI_ACCELERATOR_IP>:8080` dalla propria postazione per inviare comandi MoveJ/Stop o pilotare il joystick virtuale. La UI permette di:
   - regolare ogni joint con pulsanti ± basati sullo step configurato;
   - impostare manualmente i valori target;
   - utilizzare il joystick circolare per inviare comandi `speedj` (joystick → joint velocity);
   - monitorare lo stato dei comandi (ready, sending, error);
   - verificare lo stato del bridge ROS 2 direttamente dalla pagina (polling automatico del monitor).

## Monitor ROS2 integrato

La pagina espone un riquadro “ROS2 Monitor” sempre visibile in alto. Ogni 3 secondi (o su click “Aggiorna stato”) vengono mostrati:

- stato di inizializzazione del bridge (`ROS2 pronto` / `ROS2 non pronto`);
- stato del loop di pubblicazione a 125 Hz (con eventuali errori catturati);
- tempo trascorso dall’ultimo comando joystick e dall’ultima pubblicazione;
- disponibilità dei publisher fondamentali (`/forward_velocity_controller/commands`, `stop`, ecc.);
- variabili d’ambiente critiche (`ROS_DISTRO`, `LD_LIBRARY_PATH`, `PYTHONPATH`, host corrente).

Aprendo il dettaglio JSON è possibile vedere lo stesso payload ritornato dall’endpoint REST `/api/status`, utile per integrazioni automatiche o per condividere diagnostic snapshot via Slack/Telegram.

## Diagnostica rapida (senza muovere il robot)

```bash
source ~/.venvs/ur-remote/bin/activate
python -m remote_ur_control.diagnostics --robot-ip 192.168.10.194
```

Lo script:
- esegue (se possibile) un ping;
- verifica l’apertura della porta TCP 30002;
- invia un piccolo URScript con `textmsg()` per assicurarsi che la modalità External Control sia attiva.

## Teleoperazione affidabile via RTDE

Per integrazioni ROS 2 o per garantire compatibilità con lo stack ufficiale, si può usare la teleoperazione basata su RTDE:

```bash
source ~/.venvs/ur-remote/bin/activate
python -m remote_ur_control.rtde_teleop --robot-ip 192.168.10.194 state
python -m remote_ur_control.rtde_teleop --robot-ip 192.168.10.194 jog --joint 0 --delta 0.1
python -m remote_ur_control.rtde_teleop --robot-ip 192.168.10.194 speed --speeds 0.2 0 0 0 0 0 --duration 0.4
python -m remote_ur_control.rtde_teleop --robot-ip 192.168.10.194 halt
```

Il modulo si appoggia a `ur-rtde` e apre una sessione `RTDEControlInterface`, la stessa utilizzata da `Universal_Robots_ROS2_Driver`. In questo modo la teleoperazione resta allineata alle best practice UR e può essere richiamata da nodi ROS 2, pipeline MoveIt o servizi REST senza riscrivere la logica di basso livello.

## Note di sicurezza

- Verificare che il robot sia in modalità remota e che non vi siano operatori nell'area di lavoro prima di avviare movimenti.
- In caso di emergenza utilizzare sempre l'e-stop fisico del robot.
- La pagina espone comandi senza autenticazione: utilizzare una rete protetta o un reverse proxy con autenticazione se esposta oltre la LAN di laboratorio.

