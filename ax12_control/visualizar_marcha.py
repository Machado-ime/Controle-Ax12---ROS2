#!/usr/bin/env python3
"""
Publica JointState a partir de um YAML de marcha para visualização no RViz.

Inicie via launch file:
    ros2 launch ax12_control visualizar_marcha.launch.py \
        urdf:=/caminho/para/adam.urdf \
        matriz:=cin_inve

Parâmetros ROS:
    matriz   (str)   — arquivo de marcha sem extensão (padrão: cin_inve)
    passo_s  (float) — tempo por passo em segundos; 0 = usa o valor do YAML
"""

import os

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Pasta onde ficam as matrizes (.yaml): a mesma deste script — igual ao send_gait.
# Os YAMLs viajam ao lado do módulo via package_data (ver setup.py).
PASTA_MATRIZES = os.path.dirname(os.path.abspath(__file__))


def resolver_caminho_matriz(nome: str) -> str:
    """NOME da matriz -> caminho do .yaml ao lado deste script.

    'cin_inve' / 'cin_inve.yaml' -> <pasta deste script>/cin_inve.yaml
    '/outra/pasta/x.yaml'        -> usado como veio (já tem pasta).
    """
    nome = str(nome).strip()
    if os.path.dirname(nome):
        return nome
    if not nome.lower().endswith(('.yaml', '.yml')):
        nome += '.yaml'
    return os.path.join(PASTA_MATRIZES, nome)


# Nomes no YAML (convenção do código) → nomes no URDF (adam.urdf)
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


class VisualizarMarcha(Node):
    def __init__(self) -> None:
        super().__init__('visualizar_marcha')

        self.declare_parameter('matriz', 'cin_inve')
        self.declare_parameter('passo_s', 0.0)

        matriz_nome = self.get_parameter('matriz').value
        passo_override: float = self.get_parameter('passo_s').value

        yaml_path = resolver_caminho_matriz(matriz_nome)
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'Arquivo de marcha não encontrado: {yaml_path}')
            return

        with open(yaml_path) as f:
            marcha = yaml.safe_load(f)

        nomes_yaml: list[str] = marcha['nomes_juntas']
        self._matriz: list[list[float]] = marcha['matriz_movimento']
        self._n_passos: int = len(self._matriz[0])
        self._passo_atual: int = 0

        passo_s = passo_override if passo_override > 0.0 else float(marcha.get('passo', 1.0))

        self._nomes_urdf: list[str] = []
        for nome in nomes_yaml:
            urdf_nome = _MAPA_URDF.get(nome, nome)
            if urdf_nome == nome and nome not in _MAPA_URDF:
                self.get_logger().warn(
                    f'"{nome}" sem mapeamento URDF — publicando com esse nome mesmo.')
            self._nomes_urdf.append(urdf_nome)

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self._timer = self.create_timer(passo_s, self._tick)

        self.get_logger().info(
            f'Marcha "{matriz_nome}" — {self._n_passos} passos, {passo_s:.2f}s/passo.')

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._nomes_urdf
        msg.position = [float(linha[self._passo_atual]) for linha in self._matriz]
        self._pub.publish(msg)
        self._passo_atual = (self._passo_atual + 1) % self._n_passos


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
