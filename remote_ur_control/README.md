# Remote UR Control Toolkit

Script e interfaccia web da eseguire sull'AI Accelerator per comandare il robot UR in rete.

## Contenuto

- `remote_ur_controller.py`: client socket minimale che invia comandi URScript (movej, speedj, stop).
- `web_interface.py`: applicazione Flask con pagina HTML per impostare joint target e comandare il robot da browser.
- `requirements.txt`: dipendenze Python (Flask).

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

3. **Avviare la web UI**
   ```bash
   python -m remote_ur_control.web_interface
   ```

4. **Aprire il browser remoto**
   Visitare `http://<AI_ACCELERATOR_IP>:8080` dalla propria postazione per inviare comandi MoveJ o Stop.

## Note di sicurezza

- Verificare che il robot sia in modalità remota e che non vi siano operatori nell'area di lavoro prima di avviare movimenti.
- In caso di emergenza utilizzare sempre l'e-stop fisico del robot.
- La pagina espone comandi senza autenticazione: utilizzare una rete protetta o un reverse proxy con autenticazione se esposta oltre la LAN di laboratorio.

