#!/usr/bin/env python3
"""Gera a matriz marcha_pe: cada coluna e um estado (roll, x, pd_z, pe_z)
resolvido pela MESMA logica do controle_pe.aplicar() — IK Newton por perna
(pe paralelo ao tronco), X em oposicao (dir +x / esq -x), rolls via
SINAIS_ROLL e trim de quadril via SINAIS_TRIM_QUADRIL — com checagem de
limites. Imprime a matriz YAML e uma tabela de conferencia."""
import math

import numpy as np

from ax12_control.controle_pe import (
    CadeiaURDF, LIMITES, PERNAS, SINAIS_ROLL, SINAIS_TRIM_QUADRIL, ik_perna,
)
from ax12_control.send_gait import carregar_marcha, resolver_caminho_matriz

URDF = ('/mnt/c/Users/pecci/Desktop/Controle-Ax12---ROS2/'
        'src/adam_urdf/urdf/adam_fixed.urdf')

# (roll_graus, x_mm, pd_z_mm, pe_z_mm) por coluna + trim constante
# Teste de equilibrio NO LUGAR, 8 etapas (X=0 sempre), peso e levantamento
# em colunas SEPARADAS (diferente da marcha_pe, que funde os dois):
#   1 neutro -> 2 peso a direita -> 3 PE no ar -> 4 PE pousa ->
#   5 neutro -> 6 peso a esquerda -> 7 PD no ar -> 8 PD pousa -> repete
# Copia da teste_equilibrio (roll +-14, Z=15mm, trim -8) ADICIONANDO
# avanco: X = +-20mm (mesma magnitude usada na marcha_pe). O pe que
# "voa" (coluna 3 e 7) ja migra para a posicao X oposta DURANTE o "no ar"
# (a troca de lado acontece toda enquanto o pe esta no ar, elevado); a
# coluna de "pousa" so abaixa o pe, mantendo o X ja alcancado.
# dz_base = +5mm em TODAS as colunas (nos dois pes): agacha o robo
# uniformemente de 195mm p/ 190mm de altura base_link->pe (confirmado por
# FK), mantendo os 15mm de levantamento do pe de balanco como um
# incremento RELATIVO sobre esse novo piso (5+15=20mm nas colunas "no ar").
DZ_BASE_MM = 5.0
ETAPAS = [
    (  0.0,  10.0,  0.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 1 neutro: PD a frente
    ( 16.0,  10.0,  0.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 2 peso a direita (pe ainda no chao)
    ( 16.0, -10.0,  0.0+DZ_BASE_MM, 15.0+DZ_BASE_MM),   # 3 PE no ar (avanca de +10 p/ -10)
    ( 16.0, -10.0,  0.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 4 PE pousa (roll ainda positivo)
    (  0.0, -10.0,  0.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 5 neutro: PE a frente
    (-16.0, -10.0,  0.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 6 peso a esquerda
    (-16.0,  10.0, 15.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 7 PD no ar (avanca de -10 p/ +10)
    (-16.0,  10.0,  0.0+DZ_BASE_MM,  0.0+DZ_BASE_MM),   # 8 PD pousa (roll ainda negativo)
]
TRIM_GRAUS = -7.0
PASSO_S = 0.5

# Quebra de paralelogramo PROPOSITAL no tornozelo de apoio: nas colunas
# onde uma perna e a de apoio (2-4 = PD, espelho 6-8 = PE), o roll do
# TORNOZELO fica alguns graus A MENOS (em magnitude) que o do QUADRIL da
# mesma perna -- sem carga o pe fica levemente na borda interna; quando o
# peso chega e o servo cede sob carga, o cedimento assenta a sola chapada
# em vez de rola-la para fora. Pre-compensacao do sag (nao mexe na perna
# oposta nem nas colunas neutras 1/5).
QUEBRA_PARALELOGRAMO_GRAUS = 3.0
QUEBRA_APOIO = {
    2: 'pd_roll_tornozelo_1', 3: 'pd_roll_tornozelo_1', 4: 'pd_roll_tornozelo_1',
    6: 'pe_roll_tornozelo_2', 7: 'pe_roll_tornozelo_2', 8: 'pe_roll_tornozelo_2',
}
# LIMITES nao cobre juntas de roll (so pitch/joelho/quadril) -- mesmos
# limites medidos em ax12_controller.py, mantenha em sincronia.
LIMITES_ROLL = {
    'pd_roll_tornozelo_1': (-0.8727, 0.5934),
    'pe_roll_tornozelo_2': (-0.5934, 0.8727),
}

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

trim_rad = math.radians(TRIM_GRAUS)
colunas = []
erros = 0
for i, (roll_g, x_mm, pdz, pez) in enumerate(ETAPAS, 1):
    roll_rad = math.radians(roll_g)
    pose = dict(col1)
    dz = {'dir': pdz, 'esq': pez}
    for lado, perna in PERNAS.items():
        dx = x_mm if lado == 'dir' else -x_mm
        alvo = base[lado]['pos_xz'] + np.array([dx, dz[lado]]) / 1000.0
        sol = ik_perna(cadeia, perna, alvo, base[lado]['ang'][:2])
        if sol is None:
            print(f'ERRO coluna {i} [{lado}]: IK nao convergiu')
            erros += 1
            continue
        t = SINAIS_TRIM_QUADRIL[perna['quadril']] * trim_rad
        sol = (sol[0] + t, sol[1], sol[2])
        for n, v in zip(
                (perna['quadril'], perna['joelho'], perna['tornozelo']), sol):
            lo, hi = LIMITES[n]
            if not (lo - 1e-6 <= v <= hi + 1e-6):
                print(f'ERRO coluna {i} [{lado}] {n}: {v:+.4f} fora de '
                      f'[{lo:+.4f}, {hi:+.4f}]')
                erros += 1
            pose[n] = float(v)
    for junta, sinal in SINAIS_ROLL.items():
        pose[junta] = sinal * roll_rad

    # --- quebra de paralelogramo (tornozelo de apoio) ---
    junta_apoio = QUEBRA_APOIO.get(i)
    if junta_apoio is not None:
        reducao = math.radians(QUEBRA_PARALELOGRAMO_GRAUS)
        atual = pose[junta_apoio]
        sinal_atual = 1.0 if atual >= 0 else -1.0
        novo = atual - sinal_atual * reducao
        # nao deixa a reducao passar de zero (nem inverter o sinal)
        novo = max(0.0, novo) if sinal_atual > 0 else min(0.0, novo)
        lo, hi = LIMITES_ROLL[junta_apoio]
        if not (lo - 1e-6 <= novo <= hi + 1e-6):
            print(f'ERRO coluna {i} [{junta_apoio}]: {novo:+.4f} fora de '
                  f'[{lo:+.4f}, {hi:+.4f}] apos quebra de paralelogramo')
            erros += 1
        pose[junta_apoio] = novo

    colunas.append(pose)

if erros:
    print(f'\n{erros} erro(s) — matriz NAO gerada.')
    raise SystemExit(1)

print('TABELA (rad):')
cab = ' '.join(f'  col{i+1}   ' for i in range(len(colunas)))
print(f'{"junta":22s} {cab}')
for n in nomes:
    vals = ' '.join(f'{c[n]:+.6f}' for c in colunas)
    print(f'{n:22s} {vals}')

print('\nYAML matriz_movimento:')
for n in nomes:
    vals = ', '.join(f'{c[n]:9.6f}' for c in colunas)
    print(f'  - [ {vals} ]  # {n}')
