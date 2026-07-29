#!/usr/bin/env python3
"""Valida offline a logica nova do controle_pe: X em oposicao (dir +x,
esq -x) e trim de quadril somado apos a IK, checando limites — replica o
que aplicar() faz, sem ROS."""
import math

import numpy as np

from ax12_control.controle_pe import (
    CadeiaURDF, LIMITES, PERNAS, SINAIS_TRIM_QUADRIL, ik_perna,
)
from ax12_control.send_gait import carregar_marcha, resolver_caminho_matriz

URDF = ('/mnt/c/Users/pecci/Desktop/Controle-Ax12---ROS2/'
        'src/adam_urdf/urdf/adam_fixed.urdf')

cadeia = CadeiaURDF(open(URDF).read())
nomes, matriz, _, _ = carregar_marcha(resolver_caminho_matriz('cin_inve_2'))
col1 = {n: float(l[0]) for n, l in zip(nomes, matriz)}

base = {}
for lado, perna in PERNAS.items():
    ang = (col1[perna['quadril']], col1[perna['joelho']],
           col1[perna['tornozelo']])
    q = dict(zip((perna['quadril'], perna['joelho'], perna['tornozelo']), ang))
    p = cadeia.pos_link(perna['link_alvo'], q)
    base[lado] = dict(ang=ang, pos_xz=np.array([p[0], p[2]]))

falhas = 0
testes = 0
for x_mm in (-60, -40, -20, 0, 20, 40, 60):
    for trim_g in (-15, -7, 0, 7, 15):
        trim = math.radians(trim_g)
        ok = {}
        for lado, perna in PERNAS.items():
            dx = x_mm if lado == 'dir' else -x_mm
            alvo = base[lado]['pos_xz'] + np.array([dx, 0.0]) / 1000.0
            sol = ik_perna(cadeia, perna, alvo, base[lado]['ang'][:2])
            if sol is None:
                ok[lado] = 'IK'
                continue
            t = SINAIS_TRIM_QUADRIL[perna['quadril']] * trim
            sol = (sol[0] + t, sol[1], sol[2])
            viol = [n for n, v in zip(
                (perna['quadril'], perna['joelho'], perna['tornozelo']), sol)
                if not (LIMITES[n][0] - 1e-6 <= v <= LIMITES[n][1] + 1e-6)]
            ok[lado] = 'ok' if not viol else f'LIMITE {viol}'
        testes += 1
        if ok['dir'] != 'ok' or ok['esq'] != 'ok':
            print(f'x={x_mm:+d}mm trim={trim_g:+d}g: dir={ok["dir"]} '
                  f'esq={ok["esq"]}')
            falhas += 1

print(f'\n{testes - falhas}/{testes} combinacoes X x trim publicaveis'
      + (' — TUDO OK' if falhas == 0 else f' — {falhas} recusadas'))
