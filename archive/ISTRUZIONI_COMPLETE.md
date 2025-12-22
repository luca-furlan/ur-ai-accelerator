# 🎯 ISTRUZIONI COMPLETE - Controllo Robot UR5e

## ✅ COSA HO FATTO

1. ✅ Scaricato URCap External Control (`externalcontrol-1.0.5.urcap`)
2. ✅ Creato script controllo diretto via Primary Interface
3. ✅ Verificato connessione robot

## 🚀 DUE METODI PER CONTROLLARE IL ROBOT

### Metodo 1: Controllo Diretto (SENZA URCap) - FUNZIONA SUBITO

**Vantaggi:** Funziona subito, non richiede installazione URCap
**Limitazioni:** Controllo via URScript, più limitato

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 controllo_robot_senza_urcap.py
```

**Cosa puoi fare:**
- Leggere posizioni joints
- Inviare comandi movimento
- Testare movimento robot

### Metodo 2: External Control URCap (CONTROLLO COMPLETO) - RICHIEDE INSTALLAZIONE

**Vantaggi:** Controllo completo via ROS2, movimento fluido
**Richiede:** Installazione URCap sul robot

**File pronto:** `externalcontrol-1.0.5.urcap` (nella directory corrente)

**Come installare:**
1. Copia `externalcontrol-1.0.5.urcap` su chiavetta USB
2. Inserisci USB nel Teach Pendant
3. Sul Teach Pendant: Settings → System → URCaps → +
4. Seleziona il file dalla USB
5. Riavvia robot
6. Crea programma con nodo External Control
7. Configura: IP 192.168.10.191, Porta 50002
8. Premi PLAY

## 📋 FILE CREATI

- `externalcontrol-1.0.5.urcap` - URCap da installare (se vuoi controllo completo)
- `controllo_robot_senza_urcap.py` - Controllo diretto (funziona subito)
- `avvia_driver_ur5e_fixed.sh` - Driver ROS2 con fix segmentation fault
- `verifica_prima_di_avviare.sh` - Verifica tutto prima di avviare

## 🎯 RACCOMANDAZIONE

**Per iniziare SUBITO:**
```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 controllo_robot_senza_urcap.py
```

**Per controllo completo (dopo installazione URCap):**
```bash
# Sul Teach Pendant: Avvia External Control
# Poi sul Jetson:
bash verifica_prima_di_avviare.sh
bash avvia_driver_ur5e_fixed.sh
```

## ✅ TUTTO PRONTO!

Hai due opzioni:
1. **Controllo diretto** - Funziona subito
2. **Controllo completo** - Dopo installazione URCap











