# Configurazione External Control per Controllo Continuo

## Problema
Il programma External Control si ferma continuamente quando riceve comandi di velocità continui tramite ROS2 `forward_velocity_controller`.

## Soluzione

### 1. Configurazione del Programma sul Teach Pendant

Il programma External Control **DEVE** essere configurato correttamente sul robot:

1. **Crea un nuovo programma** sul teach pendant
2. **Aggiungi il nodo "External Control"** dalla lista URCaps
3. **Configura il nodo External Control:**
   - **Host**: IP del computer ROS2 (es. `192.168.10.191`)
   - **Port**: Porta del driver ROS2 (default `50002`)
   - **IMPORTANTE**: Assicurati che il programma sia configurato per **esecuzione continua**, non per fermarsi dopo ogni comando

### 2. Verifica che il Programma Rimanga in Esecuzione

Il programma External Control dovrebbe:
- Rimanere in **PLAYING** quando riceve comandi continui
- **NON** fermarsi quando riceve comandi di velocità zero
- **NON** fermarsi quando cambiano i comandi di velocità

### 3. Verifica la Configurazione ROS2

Assicurati che:
- Il driver ROS2 sia avviato correttamente
- Il `forward_velocity_controller` sia attivo
- I topic `/forward_velocity_controller/commands` siano pubblicati continuamente a 125Hz

### 4. Se il Problema Persiste

Se il programma continua a fermarsi:

1. **Verifica sul teach pendant** che il programma sia effettivamente in **PLAYING** e non si fermi
2. **Controlla i log del robot** per vedere se ci sono errori o avvisi
3. **Verifica la connessione di rete** tra il robot e il computer ROS2
4. **Assicurati che il programma External Control sia l'unico programma in esecuzione**

### 5. Note Importanti

- Il programma External Control **NON** dovrebbe fermarsi quando riceve comandi continui
- Se si ferma, potrebbe essere un problema di configurazione del programma sul robot
- Il driver ROS2 pubblica comandi continuamente a 125Hz, anche quando sono zero
- Il programma sul robot deve essere configurato per accettare questi comandi continui

## Riferimenti

- [Universal Robots ROS2 Driver Documentation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [External Control URCap Documentation](https://www.universal-robots.com/products/ur-software/external-control/)

