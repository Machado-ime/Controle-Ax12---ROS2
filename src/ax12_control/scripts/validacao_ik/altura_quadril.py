#!/usr/bin/env python3
"""Calcula a altura do quadril acima do chao na postura base (coluna 1
da cin_inve_2, usada como referencia por controle_pe/marcha_avanco/
teste_equilibrio) via FK real do URDF."""
import numpy as np

from ax12_control.controle_pe import CadeiaURDF, PERNAS
from ax12_control.send_gait import carregar_marcha, resolver_caminho_matriz

URDF = ('/mnt/c/Users/pecci/Desktop/Controle-Ax12---ROS2/'
        'src/adam_urdf/urdf/adam_fixed.urdf')

cadeia = CadeiaURDF(open(URDF).read())
nomes, matriz, _, _ = carregar_marcha(resolver_caminho_matriz('cin_inve_2'))
col1 = {n: float(l[0]) for n, l in zip(nomes, matriz)}

for lado, perna in PERNAS.items():
    ang = (col1[perna['quadril']], col1[perna['joelho']], col1[perna['tornozelo']])
    q = dict(zip((perna['quadril'], perna['joelho'], perna['tornozelo']), ang))
    p_quadril = cadeia.pos_link(perna['link_coxa'], {})  # pivo, independe do angulo
    p_tornozelo = cadeia.pos_link(perna['link_alvo'], q)
    altura = p_quadril[2] - p_tornozelo[2]
    print(f'[{lado}] quadril Z={p_quadril[2]*1000:+.2f}mm  '
          f'tornozelo Z={p_tornozelo[2]*1000:+.2f}mm  '
          f'altura quadril-sobre-pe = {altura*1000:.2f}mm')
