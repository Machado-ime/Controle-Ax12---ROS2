#!/usr/bin/env python3
"""
Publica JointState a partir de um YAML de marcha para visualização no RViz.

Juntas presentes na matriz recebem os ângulos do ciclo.
Juntas do URDF ausentes na matriz são publicadas em 0 rad.

Parâmetros ROS:
    matriz            (str)   — arquivo de marcha sem extensão (padrão: cin_inve)
    passo_s           (float) — segundos por passo em modo automático;
                                0.0 (padrão) = modo manual
    robot_description (str)   — conteúdo do URDF (passado pelo launch file)

Modos de operação:
    Automático (passo_s > 0): timer avança os passos no intervalo configurado.
    Manual     (passo_s = 0): aguarda mensagem Int32 em /passo_marcha.
                              O número indica o índice do passo (0-based).
                              Exemplo:
                                ros2 topic pub --once /passo_marcha std_msgs/msg/Int32 "data: 2"
"""

import os
import xml.etree.ElementTree as ET

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

# Nomes legados (antes da padronização URDF) → nomes atuais
_MAPA_URDF: dict[str, str] = {
    'PD_tornozelo_pitch_1': 'pd_picht_tornozelo_3',
    'PE_tornozelo_pitch_2': 'pe_picht_tornozelo_4',
    'PD_tornozelo_roll_3':  'pd_roll_tornozelo_1',
    'PE_tornozelo_roll_4':  'pe_roll_tornozelo_2',
    'PD_joelho_pitch_5':    'pd_picht_joelho_5',
    'PE_joelho_pitch_6':    'pe_picht_joelho_6',
    'PD_quadril_pitch_7':   'pd_picht_quadril_7',
    'PE_quadril_pitch_8':   'pe_pich_quadril_8',
}


def _juntas_do_urdf(urdf_str: str) -> list[str]:
    """Retorna nomes de todas as juntas não-fixas do URDF."""
    try:
        root = ET.fromstring(urdf_str)
        return [name for j in root.findall('joint')
                if j.get('type', 'fixed') != 'fixed'
                and (name := j.get('name'))]
    except ET.ParseError:
        return []


class VisualizarMarcha(Node):
    def __init__(self) -> None:
        super().__init__('visualizar_marcha')

        self.declare_parameter('matriz', 'cin_inve')
        self.declare_parameter('passo_s', 0.0)
        self.declare_parameter('robot_description', '')

        matriz_nome = self.get_parameter('matriz').value
        passo_s: float = self.get_parameter('passo_s').value
        urdf_str: str = self.get_parameter('robot_description').value

        pasta = os.path.dirname(os.path.abspath(__file__))
        yaml_path = os.path.join(pasta, f'{matriz_nome}.yaml')
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'Arquivo de marcha não encontrado: {yaml_path}')
            return

        with open(yaml_path) as f:
            marcha = yaml.safe_load(f)

        nomes_yaml: list[str] = marcha['nomes_juntas']
        self._matriz: list[list[float]] = marcha['matriz_movimento']
        self._n_passos: int = len(self._matriz[0])
        self._passo_atual: int = 0

        nomes_urdf = [_MAPA_URDF.get(n, n) for n in nomes_yaml]

        todas_juntas_urdf = _juntas_do_urdf(urdf_str)
        if not todas_juntas_urdf:
            self.get_logger().warn(
                'URDF não recebido ou inválido — publicando só as juntas da matriz.')

        extras = [j for j in todas_juntas_urdf if j not in nomes_urdf]
        self._nomes_pub: list[str] = nomes_urdf + extras
        self._n_matriz = len(nomes_urdf)

        self._pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber para controle manual de passo
        self.create_subscription(Int32, '/passo_marcha', self._on_passo, 10)

        if passo_s > 0.0:
            self.create_timer(passo_s, self._tick_auto)
            self.get_logger().info(
                f'Marcha "{matriz_nome}" — {self._n_passos} passos, {passo_s:.2f}s/passo '
                f'[automático]. {len(extras)} junta(s) extra(s) fixadas em 0.')
        else:
            # Publica o passo 0 imediatamente para o RViz mostrar a pose inicial
            self._publicar()
            self.get_logger().info(
                f'Marcha "{matriz_nome}" — {self._n_passos} passos [manual]. '
                f'Controle via: ros2 topic pub --once /passo_marcha std_msgs/msg/Int32 "data: N"'
                f'\n  N vai de 0 a {self._n_passos - 1}. '
                f'{len(extras)} junta(s) extra(s) fixadas em 0.')

    def _publicar(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._nomes_pub
        msg.position = (
            [float(linha[self._passo_atual]) for linha in self._matriz]
            + [0.0] * (len(self._nomes_pub) - self._n_matriz)
        )
        self._pub.publish(msg)

    def _tick_auto(self) -> None:
        self._publicar()
        self._passo_atual = (self._passo_atual + 1) % self._n_passos

    def _on_passo(self, msg: Int32) -> None:
        self._passo_atual = max(0, min(msg.data, self._n_passos - 1))
        self._publicar()
        self.get_logger().info(f'Passo {self._passo_atual}/{self._n_passos - 1}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizarMarcha()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
