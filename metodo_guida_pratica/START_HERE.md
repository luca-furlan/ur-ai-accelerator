# 🚀 START HERE - Tutto Pronto!

## ✅ Cosa Hai Nella Cartella

Tutti i file necessari sono già pronti! Ecco cosa fare:

## 📋 ORDINE DI ESECUZIONE

### 1️⃣ Verifica Setup
```bash
cd ~/MekoAiAccelerator/metodo_guida_pratica
./VERIFICA_SETUP.sh
```

Questo verifica:
- ✅ URCap scaricato
- ✅ ROS2 installato
- ✅ Driver disponibile
- ✅ Remote Control abilitato
- ✅ Porta 50002 aperta

### 2️⃣ Scarica URCap (se manca)
```bash
./SCARICA_URCAP.sh
```

### 3️⃣ Configura Robot (Teach Pendant)

Segui le istruzioni in `ISTRUZIONI_COMPLETE.txt` o:

1. Copia `externalcontrol-1.0.5.urcap` su USB
2. Installa sul Teach Pendant: Installation → URCaps → +
3. Crea programma con nodo External Control
4. IP: 192.168.10.191, Porta: 50002
5. Avvia programma (PLAY)

### 4️⃣ Avvia Driver ROS2

**Prima volta:**
```bash
./SETUP_COMPLETO.sh
```

**Se già configurato:**
```bash
./START_RAPIDO.sh
```

### 5️⃣ Test Movimento

In altro terminale:
```bash
cd ~/MekoAiAccelerator/metodo_guida_pratica
./TEST_MOVIMENTO.sh
```

## 🛠️ Script Disponibili

| Script | Cosa Fa |
|--------|---------|
| `VERIFICA_SETUP.sh` | Verifica che tutto sia configurato |
| `SCARICA_URCAP.sh` | Scarica l'URCap automaticamente |
| `SETUP_COMPLETO.sh` | Setup completo passo-passo |
| `START_RAPIDO.sh` | Avvio rapido driver (se già configurato) |
| `TEST_MOVIMENTO.sh` | Test movimento robot |
| `FERMA_TUTTO.sh` | Ferma tutti i processi ROS2 |

## 📚 Documentazione

- `README.md` - Guida completa
- `ISTRUZIONI_COMPLETE.txt` - Istruzioni dettagliate
- `REPOSITORY_GIUSTI.md` - Quali repository usare
- `LINK_REPOSITORY.md` - Link diretti

## ⚡ Quick Start (Se Tutto Già Configurato)

```bash
cd ~/MekoAiAccelerator/metodo_guida_pratica
./START_RAPIDO.sh
```

Poi in altro terminale:
```bash
cd ~/MekoAiAccelerator/metodo_guida_pratica
./TEST_MOVIMENTO.sh
```

## 🆘 Problemi?

1. Esegui `./VERIFICA_SETUP.sh` per vedere cosa manca
2. Leggi `ISTRUZIONI_COMPLETE.txt` per troubleshooting
3. Controlla che programma sia in PLAYING sul Teach Pendant

---

**Tutto è pronto! Inizia con `./VERIFICA_SETUP.sh`** 🎯



