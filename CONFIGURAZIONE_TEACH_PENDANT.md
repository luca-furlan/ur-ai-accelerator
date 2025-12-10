# 📱 Configurazione Teach Pendant - External Control

## Passo 1: Crea/Apri Programma

1. Sul **Teach Pendant**, vai su **Program**
2. Crea un nuovo programma o apri uno esistente
3. Il programma deve contenere il nodo **External Control**

## Passo 2: Aggiungi External Control

1. Nel programma, vai su **Structure** (o **URCaps**)
2. Cerca **External Control** nella lista
3. **Trascina** il nodo External Control nel programma
4. Oppure se non c'è, vai su **URCaps** → **Install** → Cerca "External Control" e installalo

## Passo 3: Configura External Control

1. **Clicca** sul nodo External Control nel programma
2. Configura questi parametri:
   - **IP Host**: `192.168.10.191` (IP dell'AI Accelerator)
   - **Porta**: `50002` (porta standard ROS2)
   - **Timeout**: lascia default o 30 secondi

## Passo 4: Salva Programma

1. Salva il programma (es: `ros_control.urp`)
2. Assicurati che sia in modalità **Remote** (non Local)

## Passo 5: Avvia Programma

1. **Premi PLAY** sul Teach Pendant
2. Dovresti vedere: "Connected to external control" o simile
3. Il programma deve essere in stato **PLAYING** (non STOPPED)

## Verifica

Sull'AI Accelerator, verifica che il driver si sia connesso:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 node list | grep ur
```

Dovresti vedere nodi UR attivi.

## Screenshot/Immagine Riferimento

Il nodo External Control dovrebbe apparire così nel programma:
```
[Start]
  ↓
[External Control] ← Questo nodo
  ↓
[End]
```

## Troubleshooting

### External Control non nella lista?

1. Vai su **URCaps** → **Install**
2. Cerca "External Control" o "ROS Control"
3. Installalo se non presente

### Non si connette?

1. Verifica IP: deve essere `192.168.10.191` (AI Accelerator)
2. Verifica porta: deve essere `50002`
3. Verifica che driver UR ROS2 sia in esecuzione sull'AI Accelerator
4. Verifica connessione di rete: `ping 192.168.10.191` dal robot

### Programma va in STOPPED?

1. Verifica che driver UR ROS2 sia avviato
2. Verifica che porta 50002 sia aperta
3. Controlla errori sul Teach Pendant

