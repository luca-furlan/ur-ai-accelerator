# 🔧 Soluzione Senza External Control URCap

## Situazione
Non hai il URCap "External Control" installato, ma hai solo controllo Locale/Remoto.

## Soluzione: Controllo Diretto via Socket

Il controllo via socket (porta 30002) funziona **senza bisogno di External Control**. Serve solo:
1. ✅ Robot in modalità **REMOTE** (non Local)
2. ✅ Un programma **in esecuzione** (anche minimale)
3. ✅ Programma in stato **PLAYING**

## Passo 1: Crea Programma Minimale sul Teach Pendant

1. Sul **Teach Pendant** → **Program**
2. Crea un **nuovo programma**
3. Aggiungi questo codice minimale:

```urscript
# Programma minimale per controllo remoto
def main():
    while True:
        sync()
end

main()
```

4. **Salva** il programma (es: `remote_control.urp`)
5. **Metti il robot in modalità REMOTE** (non Local)
6. **Premi PLAY**

## Passo 2: Verifica che Funzioni

Il programma deve essere in stato **PLAYING** (non STOPPED).

## Passo 3: Usa la Web Interface

La web interface userà automaticamente il **socket fallback** (porta 30002) che funziona senza ROS2.

## Miglioramento: Controllo Fluido via Socket

Il problema attuale è che il joystick invia comandi a 125Hz ma il socket non può gestire quella frequenza. 

**Soluzione**: Usa `servoj` invece di `speedj` per controllo più fluido, oppure riduci la frequenza di aggiornamento del joystick.

## Test Rapido

Dopo aver avviato il programma minimale sul robot:

```bash
# Test comando diretto
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.10.194", 30002))
script = "speedj([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\n"
sock.sendall(script.encode('utf-8'))
sock.close()
print("✅ Comando inviato - il robot dovrebbe muoversi")
PYTHON
```

Se il robot si muove, il controllo socket funziona!

## Nota Importante

- **Non serve ROS2** per il controllo base via socket
- **Non serve External Control URCap**
- Serve solo: Robot REMOTE + Programma PLAYING

