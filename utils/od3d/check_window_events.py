#!/usr/bin/env python3
"""OneDrop3D : contrôle statique des écrans de menus de Prusa (BUG-2 et audit du 22/11).

Nos menus en style G ne capturent plus la liste (src/gui/screen_menu.hpp) : la molette, le clic et le toucher arrivent
à l'écran, qui les passe à la fenêtre qui a le focus. Un écran dérivé de ScreenMenu qui redéfinit windowEvent doit donc
relayer les événements à la base de son menu (ScreenMenuBase_ : appui long = retour, focus rendu après un dialogue).
Sans relais, ou avec un relais direct à screen_t / window_frame_t, l'écran ne répond plus et seul le redémarrage en sort.

Le script lit src/gui (hors src/gui/od3d), retrouve les classes qui descendent de ScreenMenu, et signale chaque
windowEvent de ces classes qui n'appelle pas une base de menu. Code de sortie 1 s'il trouve un cas, 0 sinon.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
GUI = ROOT / 'src' / 'gui'

# Bases de tous les menus Prusa. menu_builder_t : alias de « typename menu_builder<...>::type » (Calibrations et tests),
# dont les spécialisations définissent « using type = ScreenMenu<...> »
MENU_ROOTS = {'ScreenMenuBase_', 'ScreenMenuBase', 'ScreenMenu', 'BasicScreenMenu', 'menu_builder_t'}

# Relais qui contournent la base des menus
BYPASS = {'screen_t', 'window_frame_t', 'window_t', 'IDialog'}

# Écrans qui doivent être reconnus : si l'analyse ne les voit plus, le contrôle ne vaut plus rien
MUST_FIND = {'ScreenMenuSTSCalibrations', 'ScreenMenuSTSWizard', 'ScreenMenuLangAndTime', 'ScreenMenuControl',
             'ScreenMenuHardware', 'ScreenMenuPrusaLink', 'ScreenMenuTune'}

IDENT = r'[A-Za-z_]\w*'
CLASS_RE = re.compile(r'\b(?:class|struct)\s+(' + IDENT + r')\s*(?:final\s*)?:\s*([^{;]*)\{', re.S)
USING_RE = re.compile(r'\busing\s+(' + IDENT + r')\s*=\s*([^;]*);', re.S)
DEF_RE = re.compile(r'\bvoid\s+(?:(' + IDENT + r')\s*::\s*)?windowEvent\s*\(([^)]*)\)\s*(?:const\s*)?(?:override\s*|final\s*)*\{')
RELAY_RE = re.compile(r'((?:' + IDENT + r'\s*::\s*)*' + IDENT + r')\s*::\s*windowEvent\s*\(')
TEMPLATE_ARGS_RE = re.compile(r'<[^<>]*>')


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', lambda m: re.sub(r'[^\n]', ' ', m.group(0)), text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


def strip_templates(text):
    # Retire les arguments de template, imbriqués compris : « A<B<C>>::D » devient « A::D »
    previous = None
    while previous != text:
        previous = text
        text = TEMPLATE_ARGS_RE.sub('', text)
    return text


KEYWORDS = {'public', 'protected', 'private', 'virtual', 'typename'}


def last_component(qualified):
    """Dernier composant d'un nom qualifié (« ns::detail::A » -> « A »), mots-clés retirés"""
    idents = [i for i in re.findall(IDENT, qualified) if i not in KEYWORDS]
    return idents[-1] if idents else None


def base_names(base_list):
    """Noms (dernier composant) des bases d'une liste « public A<x>, protected ns::B »"""
    names = (last_component(part) for part in strip_templates(base_list).split(','))
    return {n for n in names if n}


def body_at(text, open_brace):
    """Texte entre l'accolade ouvrante open_brace et sa fermante"""
    depth = 0
    for i in range(open_brace, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return text[open_brace + 1:i]
    return text[open_brace + 1:]


def main():
    files = sorted(p for p in GUI.rglob('*') if p.suffix in {'.hpp', '.cpp', '.h'} and 'od3d' not in p.relative_to(GUI).parts)
    texts = {p: strip_comments(p.read_text(encoding='utf-8', errors='replace')) for p in files}

    # Héritage : classe ou alias -> noms de ses bases
    bases = {}
    class_bodies = []  # (nom, texte du corps) pour les windowEvent définis dans la classe
    for text in texts.values():
        for m in CLASS_RE.finditer(text):
            bases.setdefault(m.group(1), set()).update(base_names(m.group(2)))
            class_bodies.append((m.group(1), body_at(text, m.end() - 1)))
        for m in USING_RE.finditer(text):
            if m.group(1) == 'type':
                continue  # « using type = ScreenMenu<...> » d'un patron : menu_builder_t est dans MENU_ROOTS
            rhs = strip_templates(m.group(2)).strip()
            qualified = re.match(r'(?:typename\s+)?((?:' + IDENT + r'\s*::\s*)*' + IDENT + r')', rhs)
            if qualified and (name := last_component(qualified.group(1))):
                bases.setdefault(m.group(1), set()).add(name)

    menus = set(MENU_ROOTS)
    changed = True
    while changed:
        changed = False
        for name, parents in bases.items():
            if name not in menus and parents & menus:
                menus.add(name)
                changed = True

    missing = MUST_FIND - menus
    if missing:
        print(f'check_window_events : analyse incomplète, écrans non reconnus : {", ".join(sorted(missing))}')
        return 1

    problems = []
    checked = 0

    def check(cls, body, where):
        nonlocal checked
        if cls not in menus or cls in MENU_ROOTS:
            return
        checked += 1
        relays = [last_component(r) for r in RELAY_RE.findall(body)]
        if not relays:
            problems.append(f'{where} : {cls}::windowEvent ne relaie pas à la base du menu (molette, clic, toucher et retour perdus)')
        elif all(r in BYPASS for r in relays):
            problems.append(f'{where} : {cls}::windowEvent relaie à {relays[0]} et contourne ScreenMenuBase_ (retour et focus perdus)')

    for path, text in texts.items():
        rel = path.relative_to(ROOT)
        for m in DEF_RE.finditer(text):
            if m.group(1):  # définition hors de la classe : « void X::windowEvent(...) { »
                line = text.count('\n', 0, m.start()) + 1
                check(m.group(1), body_at(text, m.end() - 1), f'{rel}:{line}')

    for cls, body in class_bodies:
        for m in DEF_RE.finditer(body):
            if not m.group(1):  # définition dans le corps de la classe
                check(cls, body_at(body, m.end() - 1), f'{cls} (dans la classe)')

    for p in problems:
        print(p)
    print(f'check_window_events : {len(menus)} classes de menus, {checked} windowEvent vérifiés, {len(problems)} problème(s)')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
