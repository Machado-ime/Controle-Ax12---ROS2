#!/usr/bin/env python3
"""
Gera adam.urdf.xacro a partir de adam_fixed.urdf (origens visuais corrigidas).

O que faz:
  1. Corrige os limites das 8 juntas motorizadas das pernas com os valores
     reais medidos (o export vem tudo em placeholder +-2.618).
  2. Ajusta effort/velocity dessas juntas para as specs do AX-12.
  3. Injeta o cabecalho xacro + o include do bloco <ros2_control>.

adam.urdf = export cru do SolidWorks (visual origins erradas).
adam_fixed.urdf = visual origins corrigidas manualmente; e' a base de verdade.
Re-rode este script se o SolidWorks reexportar e as origens forem recalibradas.

Uso:
    cd ~/dev/Controle-Ax12---ROS2/src/adam_urdf/urdf
    python3 gen_xacro.py
"""

import os
import re
import xml.etree.ElementTree as ET

AQUI = os.path.dirname(os.path.abspath(__file__))
ENTRADA = os.path.join(AQUI, 'adam_fixed.urdf')
SAIDA = os.path.join(AQUI, 'adam.urdf.xacro')

# Junta do URDF -> (lower, upper) em radianos. rad = (grau_AX12 - 150)*pi/180.
# PD e' espelho de PE. Quadril ainda sem medicao (placeholder +-2.618).
LIMITES = {
    'pd_picht_quadril_7':   (-2.618,  2.618),   # sem medicao
    'pd_picht_joelho_5':    (0.0,     2.618),    # (150,300) -> (0,+150)
    'pd_picht_tornozelo_3': (-1.4661, 0.5585),   # espelho de PE
    'pd_roll_tornozelo_1':  (-0.8727, 0.5934),   # espelho de PE
    'pe_pich_quadril_8':    (-2.618,  2.618),    # sem medicao
    'pe_picht_joelho_6':    (-2.618,  0.0),      # espelho de PD
    'pe_picht_tornozelo_4': (-0.5585, 1.4661),   # (118,234) -> (-32,+84)
    'pe_roll_tornozelo_2':  (-0.5934, 0.8727),   # (116,200) -> (-34,+50)
}

# Specs do AX-12 a 12 V: stall ~1.5 N.m, no-load ~59 rpm ~= 6.0 rad/s.
EFFORT = '1.5'
VELOCITY = '6.0'

CABECALHO = (
    '<?xml version="1.0" encoding="utf-8"?>\n'
    '<!-- GERADO por gen_xacro.py a partir de adam.urdf. Nao editar a mao;\n'
    '     edite gen_xacro.py ou adam.urdf e rode o script de novo. -->\n'
)


def main() -> None:
    tree = ET.parse(ENTRADA)
    root = tree.getroot()

    achadas = set()
    for joint in root.findall('joint'):
        nome = joint.get('name')
        if nome not in LIMITES:
            continue
        limit = joint.find('limit')
        if limit is None:
            limit = ET.SubElement(joint, 'limit')
        lower, upper = LIMITES[nome]
        limit.set('lower', f'{lower:.4f}')
        limit.set('upper', f'{upper:.4f}')
        limit.set('effort', EFFORT)
        limit.set('velocity', VELOCITY)
        achadas.add(nome)

    faltando = set(LIMITES) - achadas
    if faltando:
        raise SystemExit(f'Juntas nao encontradas no URDF: {sorted(faltando)}')

    corpo = ET.tostring(root, encoding='unicode')

    # Cabecalho xacro no <robot> + arg + include do bloco ros2_control.
    corpo = re.sub(
        r'<robot\s+name="adam"\s*>',
        '<robot name="adam" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:arg name="use_mock_hardware" default="true"/>',
        corpo, count=1)

    # Include relativo: resolve ao lado deste .xacro, sem depender do
    # pacote estar instalado (xacro resolve relativo ao arquivo que inclui).
    injecao = (
        '  <xacro:include filename="adam.ros2_control.xacro"/>\n'
        '  <xacro:adam_ros2_control name="AdamSystem"\n'
        '      use_mock_hardware="$(arg use_mock_hardware)"/>\n'
        '</robot>'
    )
    corpo = corpo.replace('</robot>', injecao)

    with open(SAIDA, 'w') as f:
        f.write(CABECALHO + corpo + '\n')

    print(f'OK -> {SAIDA}')
    print(f'Limites corrigidos em {len(achadas)} juntas: {sorted(achadas)}')


if __name__ == '__main__':
    main()
