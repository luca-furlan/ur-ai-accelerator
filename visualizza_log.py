#!/usr/bin/env python3
"""
Script per visualizzare e analizzare i log in modo esaustivo
"""

import sys
from pathlib import Path
from datetime import datetime
import re

LOG_DIR = Path.home() / "MekoAiAccelerator" / "logs"

def list_logs():
    """Lista tutti i log disponibili"""
    if not LOG_DIR.exists():
        print(f"❌ Directory log non trovata: {LOG_DIR}")
        return []
    
    logs = sorted(LOG_DIR.glob("*.log"), key=lambda p: p.stat().st_mtime, reverse=True)
    return logs

def show_log_summary(log_file):
    """Mostra riepilogo log"""
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        
        print(f"\n{'='*80}")
        print(f"LOG: {log_file.name}")
        print(f"Righe totali: {len(lines)}")
        print(f"{'='*80}")
        
        # Conta errori, warning, successi
        errors = [l for l in lines if 'ERROR' in l or '❌' in l]
        warnings = [l for l in lines if 'WARNING' in l or '⚠️' in l]
        success = [l for l in lines if '✅' in l]
        
        print(f"\n📊 Statistiche:")
        print(f"   ✅ Successi: {len(success)}")
        print(f"   ⚠️  Warning: {len(warnings)}")
        print(f"   ❌ Errori: {len(errors)}")
        
        # Mostra ultimi errori
        if errors:
            print(f"\n❌ Ultimi errori ({min(5, len(errors))}):")
            for err in errors[-5:]:
                print(f"   {err.strip()}")
        
        # Mostra ultimi warning
        if warnings:
            print(f"\n⚠️  Ultimi warning ({min(5, len(warnings))}):")
            for warn in warnings[-5:]:
                print(f"   {warn.strip()}")
        
        return True
    except Exception as e:
        print(f"❌ Errore lettura log: {e}")
        return False

def show_log_tail(log_file, n=50):
    """Mostra ultime N righe del log"""
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        
        print(f"\n{'='*80}")
        print(f"ULTIME {n} RIGHE: {log_file.name}")
        print(f"{'='*80}\n")
        
        for line in lines[-n:]:
            print(line.rstrip())
        
        return True
    except Exception as e:
        print(f"❌ Errore lettura log: {e}")
        return False

def search_logs(pattern):
    """Cerca pattern in tutti i log"""
    logs = list_logs()
    if not logs:
        print("❌ Nessun log trovato")
        return
    
    print(f"\n{'='*80}")
    print(f"RICERCA: '{pattern}'")
    print(f"{'='*80}\n")
    
    found = False
    for log_file in logs:
        try:
            with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
                for i, line in enumerate(f, 1):
                    if re.search(pattern, line, re.IGNORECASE):
                        print(f"{log_file.name}:{i}: {line.rstrip()}")
                        found = True
        except Exception as e:
            pass
    
    if not found:
        print("❌ Nessun risultato trovato")

def show_test_results(log_file):
    """Estrae e mostra risultati test da log troubleshooting"""
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        # Cerca sezione riepilogo
        if "RIEPILOGO RISULTATI" in content:
            print(f"\n{'='*80}")
            print(f"RISULTATI TEST: {log_file.name}")
            print(f"{'='*80}\n")
            
            # Estrai risultati
            lines = content.split('\n')
            in_summary = False
            for line in lines:
                if "RIEPILOGO RISULTATI" in line:
                    in_summary = True
                if in_summary:
                    if "✅" in line or "❌" in line or "⚠️" in line:
                        print(line)
                    if "Test passati:" in line:
                        print(line)
                        break
            
            return True
        else:
            print("⚠️  Questo log non contiene risultati test")
            return False
    except Exception as e:
        print(f"❌ Errore: {e}")
        return False

def main():
    """Menu principale"""
    print("\n" + "="*80)
    print("VISUALIZZATORE LOG - TROUBLESHOOTING ROBOT")
    print("="*80)
    
    logs = list_logs()
    if not logs:
        print("❌ Nessun log trovato in:", LOG_DIR)
        print("\nEsegui prima:")
        print("  python3 troubleshoot_robot_completo.py")
        return
    
    print(f"\n📁 Log trovati: {len(logs)}")
    print("\nLog disponibili:")
    for i, log in enumerate(logs[:10], 1):
        size = log.stat().st_size / 1024  # KB
        mtime = datetime.fromtimestamp(log.stat().st_mtime)
        print(f"  {i}. {log.name} ({size:.1f} KB, {mtime.strftime('%Y-%m-%d %H:%M:%S')})")
    
    if len(logs) > 10:
        print(f"  ... e altri {len(logs) - 10} log")
    
    print("\nOpzioni:")
    print("  1. Mostra riepilogo ultimo log")
    print("  2. Mostra ultime 50 righe ultimo log")
    print("  3. Mostra ultime 100 righe ultimo log")
    print("  4. Mostra risultati test ultimo log")
    print("  5. Cerca pattern in tutti i log")
    print("  6. Scegli log specifico")
    print("  0. Esci")
    
    scelta = input("\nScelta: ").strip()
    
    if scelta == "0":
        return
    
    if scelta == "1":
        if logs:
            show_log_summary(logs[0])
    
    elif scelta == "2":
        if logs:
            show_log_tail(logs[0], 50)
    
    elif scelta == "3":
        if logs:
            show_log_tail(logs[0], 100)
    
    elif scelta == "4":
        if logs:
            show_test_results(logs[0])
    
    elif scelta == "5":
        pattern = input("Pattern da cercare: ").strip()
        if pattern:
            search_logs(pattern)
    
    elif scelta == "6":
        print("\nScegli log (1-{}):".format(min(10, len(logs))))
        try:
            idx = int(input("Numero: ")) - 1
            if 0 <= idx < len(logs):
                log = logs[idx]
                print("\nOpzioni per questo log:")
                print("  1. Riepilogo")
                print("  2. Ultime 50 righe")
                print("  3. Ultime 100 righe")
                print("  4. Risultati test")
                sub_scelta = input("Scelta: ").strip()
                
                if sub_scelta == "1":
                    show_log_summary(log)
                elif sub_scelta == "2":
                    show_log_tail(log, 50)
                elif sub_scelta == "3":
                    show_log_tail(log, 100)
                elif sub_scelta == "4":
                    show_test_results(log)
        except ValueError:
            print("❌ Numero non valido")
    
    print(f"\n📁 Directory log: {LOG_DIR}")

if __name__ == "__main__":
    main()




