#!/usr/bin/env python3
"""Teste simples: faz PING em cada motor esperado do projeto e informa
quais respondem. Nao depende de ROS -- so da porta serial + dynamixel_sdk
(mesmas configuracoes do ax12_controller.py: protocolo 1.0, 1M bps).

Uso:
    python3 teste_motores.py [/dev/ttyACM0]
"""
import sys

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICE = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

# Nome da junta (URDF) -> ID do motor no barramento (copiado do joint_map
# de ax12_controller.py -- mantenha em sincronia se o mapa mudar).
JOINT_MAP = {
    'pd_roll_tornozelo_1': 13,
    'pe_roll_tornozelo_2': 18,
    'pd_picht_tornozelo_3': 12,
    'pe_picht_tornozelo_4': 17,
    'pd_picht_joelho_5': 11,
    'pe_picht_joelho_6': 16,
    'pd_picht_quadril_7': 10,
    'pe_pich_quadril_8': 15,
    'pd_roll_quadril_9': 9,
    'pe_roll_quadril_10': 14,
}

port = PortHandler(DEVICE)
packet = PacketHandler(PROTOCOL_VERSION)

if not port.openPort():
    print(f'ERRO: nao foi possivel abrir a porta {DEVICE}.')
    sys.exit(1)
if not port.setBaudRate(BAUDRATE):
    print(f'ERRO: nao foi possivel configurar {BAUDRATE} bps.')
    sys.exit(1)

print(f'Porta {DEVICE} aberta a {BAUDRATE} bps (protocolo {PROTOCOL_VERSION}).\n')

ok, falha = [], []
for nome, dxl_id in sorted(JOINT_MAP.items(), key=lambda kv: kv[1]):
    modelo, resultado, erro = packet.ping(port, dxl_id)
    if resultado == COMM_SUCCESS:
        print(f'  OK   ID {dxl_id:3d}  {nome:24s}  modelo={modelo}')
        ok.append((dxl_id, nome))
    else:
        print(f'  FALHA ID {dxl_id:3d}  {nome:24s}  '
              f'({packet.getTxRxResult(resultado)})')
        falha.append((dxl_id, nome))

port.closePort()

print(f'\n{len(ok)}/{len(JOINT_MAP)} motores responderam.')
if falha:
    print('Nao responderam:')
    for dxl_id, nome in falha:
        print(f'  ID {dxl_id:3d}  {nome}')
