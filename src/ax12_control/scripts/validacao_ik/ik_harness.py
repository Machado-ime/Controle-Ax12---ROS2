#!/usr/bin/env python3
"""Valida offline a IK do controle_pe contra a FK exata do URDF.

Checa: (1) X=Z=0 reproduz a coluna 1 da matriz; (2) grade de alvos converge
com erro < 1 mm; (3) pé continua plano (orientacao do link do pé igual à da
pose zero); (4) nenhum ângulo fora dos limites.
"""
import math

import numpy as np

from ax12_control.controle_pe import (
    CadeiaURDF, LIMITES, PERNAS, cos_lei_cossenos, ik_perna,
)
from ax12_control.send_gait import carregar_marcha, resolver_caminho_matriz

URDF = ('/mnt/c/Users/pecci/Desktop/Controle-Ax12---ROS2/'
        'src/adam_urdf/urdf/adam_fixed.urdf')
PE_LINK = {'dir': 'pd_pe', 'esq': 'pe_pe'}

cadeia = CadeiaURDF(open(URDF).read())
nomes, matriz, _, _ = carregar_marcha(resolver_caminho_matriz('cin_inve_2'))
col1 = {n: float(l[0]) for n, l in zip(nomes, matriz)}

# Orientação "plana" de referência: eixo z do link do pé na pose zero
ez_plano = {lado: cadeia.pose_link(PE_LINK[lado], {})[:3, 2]
            for lado in PERNAS}

falhas = 0
testes = 0

for lado, perna in PERNAS.items():
    ang_base = (col1[perna['quadril']], col1[perna['joelho']],
                col1[perna['tornozelo']])
    q_base = {perna['quadril']: ang_base[0], perna['joelho']: ang_base[1],
              perna['tornozelo']: ang_base[2]}
    p_base = cadeia.pos_link(perna['link_alvo'], q_base)
    base_xz = np.array([p_base[0], p_base[2]])

    # Alcance: |alvo - quadril| <= L1+L2 (posicoes na pose zero)
    p_hip = cadeia.pos_link(perna['link_coxa'], {})
    p_knee = cadeia.pos_link(perna['link_canela'], {})
    p_ank0 = cadeia.pos_link(perna['link_alvo'], {})
    L1 = float(np.linalg.norm(p_knee - p_hip))
    L2 = float(np.linalg.norm(p_ank0 - p_knee))
    L_total = L1 + L2
    hip_xz = np.array([p_hip[0], p_hip[2]])
    print(f'[{lado}] L1(coxa)={L1*1000:.2f}mm L2(canela)={L2*1000:.2f}mm')

    # (1) reprodução da coluna 1
    sol = ik_perna(cadeia, perna, base_xz, ang_base[:2])
    testes += 1
    if sol is None or max(abs(a - b) for a, b in zip(sol, ang_base)) > 1e-3:
        print(f'FALHA [{lado}] reproducao col1: {sol} vs {ang_base}')
        falhas += 1
    else:
        print(f'OK [{lado}] X=Z=0 reproduz a coluna 1 '
              f'(delta max {max(abs(a-b) for a,b in zip(sol,ang_base)):.2e} rad)')

    # (2..4) grade de alvos — cobre o range COMPLETO dos sliders do
    # controle_pe.py (X_MAX_MM=60, Z_MIN_MM=-20, Z_MAX_MM=60)
    for dx_mm in (-60, -50, -25, 0, 25, 50, 60):
        for dz_mm in (-20, -10, 0, 20, 40, 60):
            testes += 1
            alvo = base_xz + np.array([dx_mm, dz_mm]) / 1000.0
            d_alvo = float(np.linalg.norm(alvo - hip_xz))
            alcancavel = d_alvo <= L_total - 5e-4
            sol = ik_perna(cadeia, perna, alvo, ang_base[:2])
            rot = f'[{lado}] dx={dx_mm:+d}mm dz={dz_mm:+d}mm'
            if sol is None:
                if alcancavel:
                    print(f'FALHA {rot}: IK nao convergiu (alvo alcancavel)')
                    falhas += 1
                else:
                    print(f'OK {rot}: fora de alcance, IK recusou (correto)')
                continue
            if not alcancavel:
                print(f'AVISO {rot}: alvo no limite de alcance, IK convergiu')
            q = {perna['quadril']: sol[0], perna['joelho']: sol[1],
                 perna['tornozelo']: sol[2]}
            p = cadeia.pos_link(perna['link_alvo'], q)
            erro_mm = np.linalg.norm(np.array([p[0], p[2]]) - alvo) * 1000
            ez = cadeia.pose_link(PE_LINK[lado], q)[:3, 2]
            plano = float(np.dot(ez, ez_plano[lado]))
            limites_ok = all(
                LIMITES[n][0] - 1e-6 <= v <= LIMITES[n][1] + 1e-6
                for n, v in zip((perna['quadril'], perna['joelho'],
                                 perna['tornozelo']), sol))
            if erro_mm > 1.0 or plano < 0.9999 or not limites_ok:
                print(f'FALHA {rot}: erro={erro_mm:.3f}mm plano={plano:.5f} '
                      f'limites={"OK" if limites_ok else "VIOLADOS"} sol={sol}')
                falhas += 1

print(f'\n{testes - falhas}/{testes} testes passaram'
      + (' — TUDO OK' if falhas == 0 else f' — {falhas} FALHAS'))
raise SystemExit(1 if falhas else 0)
