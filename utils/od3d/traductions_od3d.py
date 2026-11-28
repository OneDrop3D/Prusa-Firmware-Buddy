#!/usr/bin/env python3
"""Traductions des textes OneDrop3D dans les langues de Prusa (cs, de, es, it, pl).

Nos textes sont traduits d'abord en français (fin du .po fr) ; ceux qui manquent dans une autre langue
s'y affichent en anglais. Ce script :
  manquants <langue> <sortie.json>   écrit les entrées du .po fr absentes du .po de la langue
  verifier <langue> <traductions.json> [<manquants.json>]
                                     contrôle des traductions [{"id": n, "msgstr": "..."}]
  ajouter <langue> <traductions.json> <manquants.json>
                                     contrôle puis ajoute les traductions à la fin du .po de la langue
  etat                               nombre de textes manquants par langue (code de sortie 0)

Contrôles : mêmes formats printf (%d, %s, %.1f…) dans le même ordre que l'anglais, que des caractères
déjà dessinés par la police « latin + accents » (font.py la construit à partir des .po de ces langues :
un caractère nouveau n'aurait pas d'image), et une longueur en octets qui tient dans la place du texte
(celle du français ou de l'anglais, le plus long des deux, + 15 % ; aucune marge sous 17 octets :
boutons, pastille, colonnes). Une traduction relue à la main sur l'écran peut porter "force": true
(la longueur n'est alors pas contrôlée).
"""

import json
import re
import sys
from pathlib import Path

import polib

RACINE = Path(__file__).resolve().parents[2]
PO = RACINE / 'src' / 'lang' / 'po'
LANGUES = ['cs', 'de', 'es', 'it', 'pl']
FORMAT = re.compile(r'%(?:\d+\$)?[-+ 0#]*(?:\d+|\*)?(?:\.(?:\d+|\*))?(?:hh|h|ll|l|L|z|j|t)?[diouxXeEfgGcsp%]')
# Sans « c-format », un « % » suivi d'une espace est du texte (« 80% of ») : pas de drapeau espace
TEXTE = re.compile(r'%(?:\d+\$)?[-+0#]*(?:\d+|\*)?(?:\.(?:\d+|\*))?(?:hh|h|ll|l|L|z|j|t)?[diouxXeEfgGcsp%]')


def charger(langue):
    return polib.pofile(str(PO / langue / f'Prusa-Firmware-Buddy_{langue}.po'), wrapwidth=0)


def caracteres_permis():
    # Ceux de font.py (latin_and_accents_chars) : les .po des langues accentuées et l'ASCII imprimable
    permis = set(chr(c) for c in range(0x20, 0x7f)) | set('°\n')
    for langue in ['cs', 'de', 'es', 'fr', 'it', 'pl']:
        for e in charger(langue):
            permis.update(ch for ch in e.msgid + e.msgstr if ch.isprintable())
    return permis


def place(e):
    octets = max(len(e['msgid'].encode()), len(e['fr'].encode()))
    return octets if octets < 17 else int(octets * 1.15)


def manquants(langue):
    deja = {e.msgid for e in charger(langue)}
    sortie = []
    for e in charger('fr'):
        if e.msgid and e.msgid not in deja and not e.obsolete:
            sortie.append({
                'id': len(sortie),
                'msgid': e.msgid,
                'fr': e.msgstr,
                'ref': ' '.join(f for f, _ in e.occurrences),
                'c_format': 'c-format' in e.flags,
                'max_octets': 0,
            })
            sortie[-1]['max_octets'] = place(sortie[-1])
    return sortie


def verifier(langue, traductions, entrees):
    par_id = {e['id']: e for e in entrees}
    permis = caracteres_permis()
    erreurs, vus = [], set()
    for t in traductions:
        e = par_id.get(t.get('id'))
        if e is None:
            erreurs.append(f"{t.get('id')} : id inconnu")
            continue
        vus.add(e['id'])
        s = t.get('msgstr', '')
        if not s.strip():
            erreurs.append(f"{e['id']} : vide")
            continue
        f = FORMAT if e['c_format'] else TEXTE
        if f.findall(s) != f.findall(e['msgid']):
            erreurs.append(f"{e['id']} : formats {f.findall(s)} au lieu de {f.findall(e['msgid'])}")
        hors = sorted(set(ch for ch in s if ch not in permis))
        if hors:
            erreurs.append(f"{e['id']} : caractères absents de la police {hors}")
        if len(s.encode()) > e['max_octets'] and not t.get('force'):  # « force » : longueur vérifiée à la main sur l'écran
            erreurs.append(f"{e['id']} : {len(s.encode())} octets pour {e['max_octets']} au plus : « {s} »")
        if s.count('\n') != e['fr'].count('\n'):
            erreurs.append(f"{e['id']} : {s.count(chr(10))} retours à la ligne, le français en a {e['fr'].count(chr(10))}")
    absents = sorted(set(par_id) - vus)
    if absents:
        erreurs.append(f"{len(absents)} entrées sans traduction : {absents[:20]}…")
    return erreurs


def ajouter(langue, traductions, entrees):
    par_id = {e['id']: e for e in entrees}
    po = PO / langue / f'Prusa-Firmware-Buddy_{langue}.po'
    texte = po.read_text(encoding='utf-8').rstrip('\n') + '\n'
    for t in sorted(traductions, key=lambda t: t['id']):
        e = par_id[t['id']]
        entree = polib.POEntry(msgid=e['msgid'], msgstr=t['msgstr'],
                               occurrences=[(f, '') for f in e['ref'].split()],
                               flags=['c-format'] if e['c_format'] else [])
        texte += '\n' + entree.__unicode__(wrapwidth=78)
    po.write_text(texte, encoding='utf-8')


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cmd = sys.argv[1]
    if cmd == 'etat':
        for langue in LANGUES:
            print(f'{langue} : {len(manquants(langue))} textes OneDrop3D sans traduction')
        return
    langue = sys.argv[2]
    if cmd == 'manquants':
        Path(sys.argv[3]).write_text(json.dumps(manquants(langue), ensure_ascii=False, indent=1), encoding='utf-8')
        return
    traductions = json.loads(Path(sys.argv[3]).read_text(encoding='utf-8'))
    entrees = json.loads(Path(sys.argv[4]).read_text(encoding='utf-8')) if len(sys.argv) > 4 else manquants(langue)
    erreurs = verifier(langue, traductions, entrees)
    for err in erreurs:
        print(err)
    if erreurs:
        sys.exit(f'{len(erreurs)} erreurs')
    if cmd == 'ajouter':
        ajouter(langue, traductions, entrees)
        print(f'{len(traductions)} traductions ajoutées en {langue}')
    else:
        print(f'{len(traductions)} traductions correctes')


if __name__ == '__main__':
    main()
