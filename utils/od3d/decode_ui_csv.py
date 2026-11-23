#!/usr/bin/env python3
"""OneDrop3D (lot UI-0) : nomme les écrans de OD3D_UI.CSV, la mesure de l'écran écrite sur la clé par M2010.

La colonne screen est l'adresse de la fonction qui crée l'écran dessiné (ScreenFactory::Screen<T>), ou celle de la
vtable du dialogue du dessus. Le script la cherche dans l'ELF du même firmware (artefact « elf-pile » de la CI : le
fichier sans extension) et ajoute la colonne screen_name. Les lignes « # » (un vidage, avec la version du firmware)
sont recopiées telles quelles.

    python3 utils/od3d/decode_ui_csv.py OD3D_UI.CSV coreone_release_boot_8.0.2-OD3D > OD3D_UI_noms.csv
    python3 utils/od3d/decode_ui_csv.py --test
"""
import bisect
import csv
import shutil
import struct
import subprocess
import sys


def symbols(elf: bytes):
    """[(adresse, taille, symbole)] des fonctions et objets d'un ELF 32 bits petit-boutiste, triés par adresse"""
    if elf[:4] != b'\x7fELF' or elf[4] != 1 or elf[5] != 1:
        raise ValueError('ELF 32 bits petit-boutiste attendu (firmware de la CI)')
    (shoff, ) = struct.unpack_from('<I', elf, 32)
    shentsize, shnum = struct.unpack_from('<HH', elf, 46)
    sections = [struct.unpack_from('<10I', elf, shoff + i * shentsize) for i in range(shnum)]
    found = []
    for sh in sections:
        if sh[1] != 2:  # SHT_SYMTAB
            continue
        strtab = sections[sh[6]]
        names = elf[strtab[4]:strtab[4] + strtab[5]]
        for off in range(sh[4], sh[4] + sh[5], 16):
            name, value, size, info = struct.unpack_from('<3IB', elf, off)
            if size and info & 0xf in (1, 2):  # STT_OBJECT, STT_FUNC (adresse avec le bit Thumb, comme le pointeur)
                found.append((value, size, names[name:names.index(b'\0', name)].decode()))
    return sorted(found)


def lookup(table, starts, address):
    """Symbole qui contient address (un objet pointe 8 octets après le début de sa vtable), ou None"""
    i = bisect.bisect_right(starts, address) - 1
    if i >= 0 and address < table[i][0] + table[i][1]:
        return table[i][2]
    return None


def demangle(names):
    """{symbole: nom lisible} ; sans c++filt, le symbole brut"""
    tool = shutil.which('c++filt') or shutil.which('arm-none-eabi-c++filt') or shutil.which('llvm-cxxfilt')
    if not tool or not names:
        return {s: s for s in names}
    out = subprocess.run([tool], input='\n'.join(names), capture_output=True, text=True, check=True).stdout.split('\n')
    return {s: pretty(n) for s, n in zip(names, out)}


def pretty(name):
    """« vtable for X » → X ; « … ScreenFactory::Screen<X>(PrimitiveAny<4u> const&) » → X"""
    name = name.removeprefix('vtable for ')
    if 'ScreenFactory::Screen<' in name:
        name = name.split('ScreenFactory::Screen<', 1)[1].rsplit('>(', 1)[0]
    return name


def decode(lines, table, out):
    starts = [t[0] for t in table]
    rows = []
    for line in lines:
        if line.startswith('#') or not line.strip():
            rows.append(line.rstrip('\n'))
        else:
            rows.append(next(csv.reader([line])))
    found = sorted({lookup(table, starts, int(r[1], 16)) for r in rows if isinstance(r, list) and r[0] != 't_ms'} - {None})
    names = demangle(found)
    writer = csv.writer(out, lineterminator='\n')
    for r in rows:
        if isinstance(r, str):
            out.write(r + '\n')
        elif r[0] == 't_ms':
            writer.writerow(r + ['screen_name'])
        else:
            symbol = lookup(table, starts, int(r[1], 16))
            writer.writerow(r + [names[symbol] if symbol else '?'])


def self_test():
    # ELF minimal : en-tête, sections nulle, symtab, strtab ; une vtable, une fonction de création, une étiquette
    creator = b'_ZN13ScreenFactory6ScreenIN4od3d10ScreenHomeEJEEE17static_unique_ptrI8screen_tERK12PrimitiveAnyILj4EE'
    strtab = b'\0_ZTV8MsgBoxOk\0' + creator + b'\0label\0'
    entries = ((0, 0, 0, 0), (1, 0x08100100, 0x30, 1), (15, 0x08100041, 0x20, 2), (16 + len(creator), 0x08100000, 0x40, 0))
    syms = b''.join(struct.pack('<3IBBH', n, v, size, info, 0, 1) for n, v, size, info in entries)
    shoff = 52 + len(strtab) + len(syms)
    header = b'\x7fELF\x01\x01\x01' + bytes(9) + struct.pack('<2H5I6H', 2, 40, 1, 0, 0, shoff, 0, 52, 0, 0, 40, 3, 2)
    sections = bytes(40) + struct.pack('<10I', 0, 2, 0, 0, 52 + len(strtab), len(syms), 2, 0, 4, 16) + struct.pack(
        '<10I', 0, 3, 0, 0, 52, len(strtab), 0, 0, 1, 0)
    table = symbols(header + strtab + syms + sections)
    assert [t[2] for t in table] == [creator.decode(), '_ZTV8MsgBoxOk'], table  # pas l'étiquette (STT_NOTYPE)
    starts = [t[0] for t in table]
    assert lookup(table, starts, 0x08100041) == creator.decode()  # pointeur de fonction Thumb
    assert lookup(table, starts, 0x08100108) == '_ZTV8MsgBoxOk'  # vptr = vtable + 8
    assert lookup(table, starts, 0x08100080) is None  # entre les deux
    assert pretty('static_unique_ptr<screen_t> ScreenFactory::Screen<od3d::ScreenHome>(PrimitiveAny<4u> const&)') == 'od3d::ScreenHome'
    assert pretty('vtable for MsgBoxOk') == 'MsgBoxOk'

    class Out:
        text = ''

        def write(self, s):
            Out.text += s

    decode(['t_ms,screen,cause\n', '# dump\n', '12,0x08100041,k\n', '13,0x00000004,-\n'], table, Out())
    lines = Out.text.splitlines()
    assert lines[0] == 't_ms,screen,cause,screen_name' and lines[1] == '# dump', lines
    assert lines[2].startswith('12,0x08100041,k,') and lines[2] != '12,0x08100041,k,?', lines
    assert lines[3] == '13,0x00000004,-,?', lines
    print('decode_ui_csv: self-test OK')


if __name__ == '__main__':
    if sys.argv[1:] == ['--test']:
        self_test()
    elif len(sys.argv) == 3:
        with open(sys.argv[2], 'rb') as f:
            table = symbols(f.read())
        with open(sys.argv[1], encoding='utf-8', errors='replace') as f:
            decode(f.readlines(), table, sys.stdout)
    else:
        sys.exit(__doc__)
