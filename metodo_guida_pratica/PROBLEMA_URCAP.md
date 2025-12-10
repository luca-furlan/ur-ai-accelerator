# 🔧 PROBLEMA URCap - Soluzione

## Problema Identificato

L'URCap `externalcontrol-1.0.5.urcap` ha problemi quando viene eseguito sul robot.

## Possibili Cause

1. **File URCap corrotto** - Download incompleto o danneggiato
2. **Versione incompatibile** - Firmware robot non compatibile con questa versione
3. **Installazione errata** - URCap non installato correttamente
4. **Configurazione errata** - Parametri configurati male

## Soluzioni

### Soluzione 1: Scarica URCap Nuovo

Ho scaricato una nuova versione del file URCap:
- File nuovo: `externalcontrol-1.0.5-NEW.urcap`
- Verifica che sia identico al vecchio

### Soluzione 2: Reinstalla URCap

1. **Disinstalla URCap vecchio:**
   - Teach Pendant → Installation → URCaps
   - Seleziona "External Control"
   - Rimuovi/Uninstall

2. **Installa URCap nuovo:**
   - Copia `externalcontrol-1.0.5-NEW.urcap` su USB
   - Teach Pendant → Installation → URCaps → +
   - Seleziona file dalla USB
   - Installa
   - Riavvia robot

### Soluzione 3: Verifica Versione Firmware Robot

L'URCap potrebbe non essere compatibile con la versione firmware del robot.

**Verifica firmware:**
- Teach Pendant → About → Version
- Controlla versione firmware (es: CB3, e-Series, Polyscope versione)

**URCap compatibile:**
- External Control 1.0.5 richiede Polyscope 3.x o superiore
- Per robot e-Series (UR5e) dovrebbe funzionare

### Soluzione 4: Prova Versione Diversa URCap

Se 1.0.5 non funziona, prova altre versioni:

**Versioni disponibili:**
- v1.0.5 (attuale)
- v1.0.4 (precedente)
- v1.0.3 (più vecchia)

**Download alternative:**
```bash
# Versione 1.0.4
https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.4/externalcontrol-1.0.4.urcap

# Versione 1.0.3
https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.3/externalcontrol-1.0.3.urcap
```

## Verifica File URCap

### Controlla integrità file:

**Windows PowerShell:**
```powershell
Get-FileHash -Path "externalcontrol-1.0.5.urcap" -Algorithm MD5
```

**Dimensione corretta:**
- File dovrebbe essere ~35 KB (35,287 bytes)

### Confronta file vecchio e nuovo:

```powershell
# Verifica se file sono identici
Compare-Object (Get-Content "externalcontrol-1.0.5.urcap" -Raw) (Get-Content "externalcontrol-1.0.5-NEW.urcap" -Raw)
```

## Procedura Completa di Reinstallazione

1. **Rimuovi URCap vecchio:**
   - Teach Pendant → Installation → URCaps
   - Seleziona "External Control" → Uninstall
   - Riavvia robot

2. **Scarica URCap nuovo:**
   - Usa file `externalcontrol-1.0.5-NEW.urcap` dalla cartella
   - O scarica di nuovo da GitHub

3. **Copia su USB:**
   - Formatta USB FAT32
   - Copia file URCap nella root USB
   - Verifica che file sia completo (~35 KB)

4. **Installa sul robot:**
   - Inserisci USB nel Teach Pendant
   - Installation → URCaps → +
   - Seleziona file dalla USB
   - Installa
   - Riavvia quando richiesto

5. **Verifica installazione:**
   - Installation → URCaps
   - Dovresti vedere "External Control" nella lista

6. **Crea programma:**
   - Crea nuovo programma
   - Aggiungi nodo "External Control"
   - Configura:
     - IP Host: 192.168.10.191
     - Porta: 50002
   - Salva programma

7. **Test connessione:**
   - Avvia programma (PLAY)
   - Dovresti vedere "Waiting for connection..."
   - Nessun errore di connessione

## Troubleshooting

### Errore: "URCap installation failed"
- Verifica versione firmware robot
- Prova versione diversa URCap
- Riavvia robot prima di installare

### Errore: "Connection could not be established"
- Verifica IP Host: 192.168.10.191 (con punti, no spazi)
- Verifica Porta: 50002 (separata dall'IP)
- Verifica che programma sia in PLAYING

### Errore: "URCap not found"
- Verifica che URCap sia installato correttamente
- Riavvia robot dopo installazione
- Controlla Installation → URCaps

## File Disponibili

Nella cartella `metodo_guida_pratica`:
- `externalcontrol-1.0.5.urcap` - File originale
- `externalcontrol-1.0.5-NEW.urcap` - File scaricato di nuovo (se disponibile)

Usa il file nuovo se quello vecchio dà problemi!


