# STOP - Guida Sicurezza

## ⚠️ IMPORTANTE

Se il robot è in emergenza o hai bisogno di fermare tutto:

1. **E-STOP**: Premi il pulsante di emergenza fisico sul robot
2. **Teach Pendant**: Premi STOP sul teach pendant
3. **Software**: Non eseguire più comandi fino a quando non hai verificato che tutto sia OK

## Movimenti Sicuri

Quando testi i movimenti, usa sempre:
- **Valori piccoli**: max 0.05-0.1 rad (~3-6 gradi) per iniziare
- **Velocità basse**: 0.05-0.1 rad/s
- **Accelerazione basse**: 0.3-0.5 rad/s²
- **Un movimento alla volta**: non inviare comandi multipli rapidamente

## Test Sicuro

Usa solo:
```bash
python -m remote_ur_control.safe_test
```

Questo script usa movimenti minimi e sicuri.

## Non Usare

- ❌ Movimenti grandi (>0.2 rad) senza testare prima
- ❌ Comandi multipli rapidi
- ❌ Velocità/alte (>0.3 rad/s) senza controllo
- ❌ Test automatici senza supervisione

