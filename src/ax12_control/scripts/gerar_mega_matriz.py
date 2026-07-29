#!/usr/bin/env python3
"""Gera uma 'mega matriz' a partir da marcha_avanco: expande cada
transicao entre colunas numa rampa cosseno amostrada a dt=0.02s,
seguindo:

    T = passos[coluna_destino]; dt = 0.02
    for t = dt:dt:T:
        u = t/T
        s = (1 - cos(pi*u)) / 2
        q(t) = q_anterior + s*(q_alvo - q_anterior)   # vetor 10x1

8 segmentos (fecha o ciclo: coluna 8 -> coluna 1 tambem vira rampa).
T de cada segmento = passos[coluna_destino] (a duracao ja definida na
marcha_avanco para chegar aquela coluna); a pausa original daquela coluna
vira a pausa do ULTIMO micro-passo do segmento (as demais ficam com
pausa 0 -- o passo em si e o dt).

Como os T (0.30/0.25s) nao sao multiplos exatos de dt=0.02s em todos os
casos, cada segmento usa N = round(T/dt) micro-passos com dt_efetivo =
T/N (ajuste mínimo pra fechar exato em T -- documentado no cabecalho do
YAML gerado).
"""
import math

import yaml

from ax12_control.send_gait import carregar_marcha, carregar_pausas, carregar_passos, resolver_caminho_matriz

DT = 0.02
MATRIZ_BASE = 'marcha_avanco'

nomes, matriz, passo_esc, pausa_esc = carregar_marcha(resolver_caminho_matriz(MATRIZ_BASE))
num_cols = len(matriz[0])
passos = carregar_passos(resolver_caminho_matriz(MATRIZ_BASE), num_cols, passo_esc)
pausas = carregar_pausas(resolver_caminho_matriz(MATRIZ_BASE), num_cols, pausa_esc)

mega_matriz = [[] for _ in nomes]
mega_passos = []
mega_pausas = []

print(f'Base: {MATRIZ_BASE} ({num_cols} colunas)\n')
for i in range(num_cols):
    dest = (i + 1) % num_cols
    T = passos[dest]
    N = max(1, round(T / DT))
    dt_ef = T / N
    print(f'Segmento col{i+1}->col{dest+1}: T={T:.3f}s  N={N}  '
          f'dt_efetivo={dt_ef:.5f}s  pausa_final={pausas[dest]:.3f}s')

    for k in range(1, N + 1):
        u = k / N
        s = (1 - math.cos(math.pi * u)) / 2
        for j, nome in enumerate(nomes):
            q_ant = matriz[j][i]
            q_alvo = matriz[j][dest]
            mega_matriz[j].append(q_ant + s * (q_alvo - q_ant))
        mega_passos.append(dt_ef)
        mega_pausas.append(pausas[dest] if k == N else 0.0)

total_cols = len(mega_passos)
print(f'\nTotal de micro-colunas geradas: {total_cols}')

# --- monta o YAML de saida ---
saida = {
    'passo': DT,
    'pausa': 0.0,
    'passos': [round(p, 6) for p in mega_passos],
    'pausas': [round(p, 6) for p in mega_pausas],
    'nomes_juntas': list(nomes),
    'matriz_movimento': [[round(v, 6) for v in linha] for linha in mega_matriz],
}

CAMINHO_SAIDA = '/tmp/marcha_avanco_suave.yaml'
with open(CAMINHO_SAIDA, 'w', encoding='utf-8') as f:
    yaml.safe_dump(saida, f, default_flow_style=None, sort_keys=False, width=200)
print(f'Escrito em {CAMINHO_SAIDA}')
