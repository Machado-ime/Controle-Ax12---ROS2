"""Bridge: /joint_trajectory → /perna_X_controller/joint_trajectory.

send_gait publica em /joint_trajectory com QoS BEST_EFFORT.
O JointTrajectoryController do ros2_control assina com RELIABLE.
Este nó faz a conversão de QoS e divide os joints pelo controlador certo.

Uso:
    # Terminal 1: digital twin
    ros2 launch adam_moveit_config demo.launch.py

    # Terminal 2: bridge (deixar rodando)
    ros2 run ax12_control gait_bridge

    # Terminal 3: enviar marcha
    ros2 run ax12_control send_gait
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Nomes legados (antes do URDF virar padrao) → nomes atuais do URDF.
# O bridge aceita os dois sem precisar alterar os YAMLs de marcha.
_LEGADO = {
    'PD_tornozelo_pitch_1': 'pd_picht_tornozelo_3',
    'PE_tornozelo_pitch_2': 'pe_picht_tornozelo_4',
    'PD_tornozelo_roll_3':  'pd_roll_tornozelo_1',
    'PE_tornozelo_roll_4':  'pe_roll_tornozelo_2',
    'PD_joelho_pitch_5':    'pd_picht_joelho_5',
    'PE_joelho_pitch_6':    'pe_picht_joelho_6',
    'PD_quadril_pitch_7':   'pd_picht_quadril_7',
    'PE_quadril_pitch_8':   'pe_pich_quadril_8',
}

# Conjunto de juntas de cada controlador (nomes do URDF)
_PD = frozenset({'pd_picht_quadril_7', 'pd_roll_quadril_9', 'pd_picht_joelho_5',
                  'pd_picht_tornozelo_3', 'pd_roll_tornozelo_1'})
_PE = frozenset({'pe_pich_quadril_8', 'pe_roll_quadril_10', 'pe_picht_joelho_6',
                  'pe_picht_tornozelo_4', 'pe_roll_tornozelo_2'})


def _filtrar(msg_orig, conjunto):
    """Retorna nova JointTrajectory com apenas as juntas do conjunto."""
    idx = [i for i, n in enumerate(msg_orig.joint_names) if n in conjunto]
    if not idx:
        return None
    msg = JointTrajectory()
    msg.header = msg_orig.header
    msg.joint_names = [msg_orig.joint_names[i] for i in idx]
    for po in msg_orig.points:
        pt = JointTrajectoryPoint()
        pt.time_from_start = po.time_from_start
        if po.positions:
            pt.positions = [po.positions[i] for i in idx]
        if po.velocities:
            pt.velocities = [po.velocities[i] for i in idx]
        if po.accelerations:
            pt.accelerations = [po.accelerations[i] for i in idx]
        if po.effort:
            pt.effort = [po.effort[i] for i in idx]
        msg.points.append(pt)
    return msg


class GaitBridge(Node):
    def __init__(self):
        super().__init__('gait_bridge')

        # Mesma QoS do send_gait para que a conexao seja formada
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # RELIABLE para o JointTrajectoryController
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            JointTrajectory, '/joint_trajectory', self._cb, qos_be)
        self._pub_pd = self.create_publisher(
            JointTrajectory,
            '/perna_direita_controller/joint_trajectory', qos_rel)
        self._pub_pe = self.create_publisher(
            JointTrajectory,
            '/perna_esquerda_controller/joint_trajectory', qos_rel)

        self._avisou_legado = False
        self.get_logger().info(
            'gait_bridge ativo — aguardando send_gait em /joint_trajectory')

    def _cb(self, msg):
        # Traduz nomes legados, se houver
        legados = [n for n in msg.joint_names if n in _LEGADO]
        if legados:
            if not self._avisou_legado:
                self.get_logger().warn(
                    f'Nomes legados {legados!r} traduzidos automaticamente. '
                    'Atualize nomes_juntas no YAML de marcha para silenciar este aviso.')
                self._avisou_legado = True
            msg.joint_names = [_LEGADO.get(n, n) for n in msg.joint_names]

        pd = _filtrar(msg, _PD)
        pe = _filtrar(msg, _PE)
        if pd:
            self._pub_pd.publish(pd)
        if pe:
            self._pub_pe.publish(pe)


def main():
    rclpy.init()
    node = GaitBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
