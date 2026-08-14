#!/usr/bin/env python3
"""Converte a tabela em graus da 'matriz_lugar' (fornecida pelo usuario,
gerada externamente por gerar_matriz_lugar.m) para radianos, valida contra
LIMITES de controle_pe.py, e imprime o bloco YAML pronto."""
import math

from ax12_control.controle_pe import LIMITES

# Tabela em GRAUS, exatamente como fornecida (8 colunas)
GRAUS = {
    'pd_roll_tornozelo_1':  [0, -14.47, -14.47, -14.47, 0, 14.47, 0, 0],
    'pe_roll_tornozelo_2':  [0, -14.47, 0, 0, 0, 14.47, 14.47, 14.47],
    'pd_picht_tornozelo_3': [-36.94, -36.94, -36.94, -36.94, -36.94, -36.94, -53.83, -36.94],
    'pe_picht_tornozelo_4': [36.94, 36.94, 53.83, 36.94, 36.94, 36.94, 36.94, 36.94],
    'pd_picht_joelho_5':    [73.84, 73.84, 73.84, 73.84, 73.84, 73.84, 107.26, 73.84],
    'pe_picht_joelho_6':    [-73.84, -73.84, -107.26, -73.84, -73.84, -73.84, -73.84, -73.84],
    'pd_picht_quadril_7':   [-45.90, -45.90, -45.90, -45.90, -45.90, -45.90, -44.43, -45.90],
    'pe_pich_quadril_8':    [45.90, 45.90, 44.43, 45.90, 45.90, 45.90, 45.90, 45.90],
    'pd_roll_quadril_9':    [0, 14.47, 14.47, 14.47, 0, -14.47, 0, 0],
    'pe_roll_quadril_10':   [0, 14.47, 0, 0, 0, -14.47, -14.47, -14.47],
}
ORDEM = list(GRAUS.keys())

erros = 0
rad = {}
for nome in ORDEM:
    linha = [math.radians(g) for g in GRAUS[nome]]
    rad[nome] = linha
    if nome in LIMITES:
        lo, hi = LIMITES[nome]
        for i, v in enumerate(linha, 1):
            if not (lo - 1e-6 <= v <= hi + 1e-6):
                print(f'ERRO {nome} col{i}: {v:+.6f} rad fora de '
                      f'[{lo:+.4f}, {hi:+.4f}]')
                erros += 1

if erros:
    print(f'\n{erros} erro(s) de limite.')
else:
    print('Todos os angulos dentro dos limites (juntas de pitch checadas).')

print('\nYAML matriz_movimento:')
for nome in ORDEM:
    vals = ', '.join(f'{v:9.6f}' for v in rad[nome])
    print(f'  - [ {vals} ]  # {nome}')
