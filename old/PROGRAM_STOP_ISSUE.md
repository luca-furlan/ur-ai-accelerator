# Problema: Programma External Control si ferma continuamente

## Sintomo
Il programma External Control sul robot fa continuamente: **PLAY → STOP → PLAY → STOP**

## Possibili Cause

### 1. Programma External Control sul Robot
Il programma External Control sul teach pendant potrebbe avere un **ciclo while** che lo ferma quando riceve comandi. Questo è un problema noto con alcuni URCaps.

**Soluzione**: Verifica sul teach pendant che il programma External Control sia configurato correttamente:
- Non deve avere cicli `while` che fermano il programma
- Deve essere configurato per esecuzione continua
- Deve accettare comandi continui senza fermarsi

### 2. Driver ROS2 che ferma il programma
Il driver ROS2 `ur_robot_driver` potrebbe fermare il programma quando riceve certi tipi di comandi.

**Verifica**: Controlla se il servizio `/io_and_status_controller/resend_robot_program` viene chiamato automaticamente.

### 3. Comandi di velocità zero
Quando pubblichiamo comandi di velocità zero continuamente, il programma potrebbe fermarsi.

**Soluzione**: Assicurati che il programma External Control sia configurato per accettare comandi di velocità zero senza fermarsi.

## Verifica sul Robot

1. **Sul Teach Pendant**:
   - Vai a **Program** → **External Control**
   - Verifica che il programma sia in **PLAYING** (non STOPPED o PAUSED)
   - Controlla se ci sono errori o avvisi

2. **Verifica la configurazione del programma**:
   - Il programma External Control deve essere configurato per **esecuzione continua**
   - Non deve fermarsi quando riceve comandi di velocità
   - Deve accettare comandi continui da ROS2

## Riferimenti

- [Universal Robots Forum - While loop in URCap stops program](https://forum.universal-robots.com/t/while-loop-in-urcap-stops-program/7950)
- [ROS2 UR Driver Documentation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

## Note

Il problema **NON** è nel codice Python (ros2_bridge_fixed.py) - abbiamo verificato che non ci sono chiamate a play/stop nel bridge.

Il problema è probabilmente:
1. **Configurazione del programma External Control sul robot**
2. **Comportamento del driver ROS2 quando riceve comandi continui**
3. **Cicli while nel programma External Control che fermano il programma**

