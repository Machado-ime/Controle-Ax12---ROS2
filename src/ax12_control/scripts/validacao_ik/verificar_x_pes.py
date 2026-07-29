#!/usr/bin/env python3
"""Calcula, via FK real do URDF, a posicao X (mm) do tornozelo de cada
perna em CADA coluna da matriz, para conferir se o movimento e
simetrico/oposto como pretendido (dir=+x_input, esq=-x_input)."""
import sys

import numpy as np

from ax12_control.controle_pe import CadeiaURDF, PERNAS
from ax12_control.send_gait import carregar_marcha, resolver_caminho_matriz

URDF = ('/mnt/c/Users/pecci/Desktop/Controle-Ax12---ROS2/'
        'src/adam_urdf/urdf/adam_fixed.urdf')

matriz_nome = sys.argv[1] if len(sys.argv) > 1 else 'marcha_avanco'
cadeia = CadeiaURDF(open(URDF).read())
nomes, matriz, _, _ = carregar_marcha(resolver_caminho_matriz(matriz_nome))
col = {n: [float(v) for v in l] for n, l in zip(nomes, matriz)}
num_cols = len(matriz[0])

# Referencia (X=0): coluna 1 da cin_inve_2, igual ao controle_pe/gerador
nomes_b, matriz_b, _, _ = carregar_marcha(resolver_caminho_matriz('cin_inve_2'))
base_c1 = {n: float(l[0]) for n, l in zip(nomes_b, matriz_b)}
base_xz = {}
for lado, perna in PERNAS.items():
    ang = (base_c1[perna['quadril']], base_c1[perna['joelho']], base_c1[perna['tornozelo']])
    q = dict(zip((perna['quadril'], perna['joelho'], perna['tornozelo']), ang))
    p = cadeia.pos_link(perna['link_alvo'], q)
    base_xz[lado] = np.array([p[0], p[2]])

print(f'Matriz: {matriz_nome} ({num_cols} colunas)\n')
print(f'{"col":>4} | {"PD (dir) X_abs mm":>20} {"rel.base":>10} | '
      f'{"PE (esq) X_abs mm":>20} {"rel.base":>10}')
for c in range(num_cols):
    linha = f'{c+1:>4} |'
    for lado, perna in PERNAS.items():
        ang = (col[perna['quadril']][c], col[perna['joelho']][c], col[perna['tornozelo']][c])
        q = dict(zip((perna['quadril'], perna['joelho'], perna['tornozelo']), ang))
        p = cadeia.pos_link(perna['link_alvo'], q)
        x_abs_mm = p[0] * 1000
        x_rel_mm = (p[0] - base_xz[lado][0]) * 1000
        linha += f' {x_abs_mm:>+19.2f} {x_rel_mm:>+9.2f} mm |'
    print(linha)
