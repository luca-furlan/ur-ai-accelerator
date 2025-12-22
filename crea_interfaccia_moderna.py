#!/usr/bin/env python3
"""
Script per creare interfaccia moderna senza emoji con Material Design
"""
import re

# Leggi il file esistente
with open('remote_ur_control/web_interface.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Trova il template HTML
start_marker = 'HTML_TEMPLATE = """'
end_marker = '"""'

start_idx = content.find(start_marker)
if start_idx == -1:
    print("ERROR: HTML_TEMPLATE non trovato!")
    exit(1)

# Trova la fine del template (terza occorrenza di """ dopo start)
template_start = start_idx + len(start_marker)
# Cerca la chiusura del template (deve essere su una riga separata)
lines = content[template_start:].split('\n')
template_end_idx = None
for i, line in enumerate(lines):
    if line.strip() == '"""':
        template_end_idx = template_start + sum(len(l) + 1 for l in lines[:i+1])
        break

if template_end_idx is None:
    print("ERROR: Fine template non trovata!")
    exit(1)

print(f"Template trovato: caratteri {start_idx} - {template_end_idx}")

# Leggi il template HTML moderno
with open('remote_ur_control/web_interface_modern_template.html', 'r', encoding='utf-8') as f:
    new_template_html = f.read()

# Leggi tutto il JavaScript esistente dal file originale
old_template = content[template_start:template_end_idx]
js_start = old_template.find('<script>')
js_end = old_template.rfind('</script>')

if js_start == -1 or js_end == -1:
    print("ERROR: JavaScript non trovato nel template!")
    exit(1)

old_js = old_template[js_start+8:js_end]

# Rimuovi tutte le emoji e sostituiscile con icone Material Design
emoji_replacements = {
    '✅': '<span class="material-icons md-18" style="color: var(--mdc-theme-secondary); vertical-align: middle;">check_circle</span>',
    '❌': '<span class="material-icons md-18" style="color: var(--mdc-theme-error); vertical-align: middle;">error</span>',
    '⚠️': '<span class="material-icons md-18" style="color: var(--mdc-theme-warning); vertical-align: middle;">warning</span>',
    '⏳': '<span class="material-icons md-18" style="vertical-align: middle;">hourglass_empty</span>',
    '🔄': '<span class="material-icons md-18" style="vertical-align: middle;">refresh</span>',
    '📋': '<span class="material-icons md-18" style="vertical-align: middle;">description</span>',
    '🎮': '<span class="material-icons md-18" style="vertical-align: middle;">sports_esports</span>',
    '🔍': '<span class="material-icons md-18" style="vertical-align: middle;">search</span>',
    '🗑️': '<span class="material-icons md-18" style="vertical-align: middle;">delete</span>',
    '⏸️': '<span class="material-icons md-18" style="vertical-align: middle;">pause</span>',
    '▶️': '<span class="material-icons md-18" style="vertical-align: middle;">play_arrow</span>',
    '⏹️': '<span class="material-icons md-18" style="vertical-align: middle;">stop</span>',
    '🚀': '<span class="material-icons md-18" style="vertical-align: middle;">rocket_launch</span>',
    '🤖': '<span class="material-icons md-18" style="vertical-align: middle;">smart_toy</span>',
    'ℹ️': '<span class="material-icons md-18" style="vertical-align: middle;">info</span>',
    '📤': '<span class="material-icons md-18" style="vertical-align: middle;">send</span>',
    '⚙️': '<span class="material-icons md-18" style="vertical-align: middle;">settings</span>',
}

# Sostituisci emoji nel JavaScript
new_js = old_js
for emoji, replacement in emoji_replacements.items():
    # Per le stringhe JavaScript, usa solo il nome dell'icona
    icon_name = {
        '✅': 'check_circle',
        '❌': 'error',
        '⚠️': 'warning',
        '⏳': 'hourglass_empty',
        '🔄': 'refresh',
        '📋': 'description',
        '🎮': 'sports_esports',
        '🔍': 'search',
        '🗑️': 'delete',
        '⏸️': 'pause',
        '▶️': 'play_arrow',
        '⏹️': 'stop',
        '🚀': 'rocket_launch',
        '🤖': 'smart_toy',
        'ℹ️': 'info',
        '📤': 'send',
        '⚙️': 'settings',
    }.get(emoji, 'info')
    
    # Sostituisci emoji nelle stringhe JavaScript con nome icona
    new_js = new_js.replace(f"'{emoji}'", f"'<span class=\\'material-icons md-18\\'>{icon_name}</span>'")
    new_js = new_js.replace(f'"{emoji}"', f'"<span class=\\"material-icons md-18\\">{icon_name}</span>"')
    new_js = new_js.replace(f'`{emoji}`', f'`<span class="material-icons md-18">${icon_name}</span>`')
    # Sostituisci anche emoji direttamente nel testo
    new_js = re.sub(re.escape(emoji), f'<span class="material-icons md-18">{icon_name}</span>', new_js)

# Rimuovi anche emoji dai messaggi di testo
for emoji in emoji_replacements.keys():
    icon_name = {
        '✅': 'check_circle',
        '❌': 'error',
        '⚠️': 'warning',
        '⏳': 'hourglass_empty',
        '🔄': 'refresh',
        '📋': 'description',
        '🎮': 'sports_esports',
        '🔍': 'search',
        '🗑️': 'delete',
        '⏸️': 'pause',
        '▶️': 'play_arrow',
        '⏹️': 'stop',
        '🚀': 'rocket_launch',
        '🤖': 'smart_toy',
        'ℹ️': 'info',
        '📤': 'send',
        '⚙️': 'settings',
    }.get(emoji, 'info')
    
    # Sostituisci emoji nei messaggi HTML
    new_js = new_js.replace(emoji, f'<span class="material-icons md-18">{icon_name}</span>')

# Sostituisci anche nelle funzioni updateWizardStep che usano emoji nei messaggi
new_js = re.sub(r"'🔄 ([^']+)'", r"'<span class=\\'material-icons md-18\\'>refresh</span> \1'", new_js)
new_js = re.sub(r"'✅ ([^']+)'", r"'<span class=\\'material-icons md-18\\'>check_circle</span> \1'", new_js)
new_js = re.sub(r"'❌ ([^']+)'", r"'<span class=\\'material-icons md-18\\'>error</span> \1'", new_js)
new_js = re.sub(r"'⚠️ ([^']+)'", r"'<span class=\\'material-icons md-18\\'>warning</span> \1'", new_js)

# Inserisci il nuovo JavaScript nel template HTML moderno
new_template_with_js = new_template_html.replace(
    '      // ... [TUTTO IL RESTO DEL JAVASCRIPT ESISTENTE - wizard, joystick, etc. - SENZA EMOJI] ...\n      \n      // Placeholder per tutto il resto del codice JavaScript esistente\n      // Devo copiare tutto il JavaScript esistente ma sostituire le emoji con icone Material Design',
    new_js
)

# Sostituisci il template nel file Python
new_content = (
    content[:start_idx + len(start_marker)] + 
    '\n' + new_template_with_js + '\n' +
    content[template_end_idx:]
)

# Salva il file aggiornato
with open('remote_ur_control/web_interface.py', 'w', encoding='utf-8') as f:
    f.write(new_content)

print("OK: Interfaccia moderna creata senza emoji!")
print(f"   Template HTML: {len(new_template_with_js)} caratteri")
print(f"   JavaScript: {len(new_js)} caratteri")


