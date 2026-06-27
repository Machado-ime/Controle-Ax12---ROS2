#!/usr/bin/env python3
"""
Janela Qt com um slider por junta para jog manual dos motores AX-12 reais.

Cada slider publica /joint_trajectory com a junta movida (mensagem de uma
junta só), o MESMO tópico que o ax12_controller já assina — então mover o
slider comanda o motor real imediatamente. O RViz não recebe nada direto
desta janela: ele mostra a posição publicada em /joint_states pelo próprio
ax12_controller a partir da telemetria real do motor. Resultado: mover o
slider manda o ângulo para o motor E para o RViz "ao mesmo tempo" (o
atraso é só o do movimento físico + da leitura de telemetria, tipicamente
< 1s) — sem duplicar publisher de /joint_states e sem briga de QoS.

Uso (com ax12_controller já rodando, motores ligados):
    ros2 launch ax12_control controle_manual.launch.py
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from python_qt_binding.QtCore import Qt
    from python_qt_binding.QtWidgets import (
        QApplication, QLabel, QSlider, QVBoxLayout, QWidget,
    )
except ImportError:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (
        QApplication, QLabel, QSlider, QVBoxLayout, QWidget,
    )

# Mesmos limites medidos em ax12_controller.py — mantenha os dois em sincronia
# se uma junta nova for medida ou ligada.
LIMITE_RAD = 2.618
JUNTAS = {
    'pd_picht_tornozelo_3': (-1.4661,     0.5585),
    'pe_picht_tornozelo_4': (-0.5585,     1.4661),
    'pd_roll_tornozelo_1':  (-0.8727,     0.5934),
    'pe_roll_tornozelo_2':  (-0.5934,     0.8727),
    'pd_picht_joelho_5':    (0.0,         LIMITE_RAD),
    'pe_picht_joelho_6':    (-LIMITE_RAD, 0.0),
    'pd_picht_quadril_7':   (-LIMITE_RAD, LIMITE_RAD),
    'pe_pich_quadril_8':    (-LIMITE_RAD, LIMITE_RAD),
    'pd_roll_quadril_9':    (-LIMITE_RAD, LIMITE_RAD),
    'pe_roll_quadril_10':   (-LIMITE_RAD, LIMITE_RAD),
}

# QSlider só trabalha com inteiros: cada unidade do slider = 1/100 rad.
ESCALA = 100


class ControleManual(Node):
    def __init__(self) -> None:
        super().__init__('controle_manual')
        self.declare_parameter('velocidade', 1.0)  # rad/s enviado em cada comando de slider
        self.velocidade = self.get_parameter('velocidade').value

        # Mesmo perfil do ax12_controller: comando de slider velho nunca deve
        # chegar atrasado por trás de um comando mais novo (fila depth=1).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(JointTrajectory, '/joint_trajectory', qos)

    def mover(self, junta: str, rad: float) -> None:
        msg = JointTrajectory()
        msg.joint_names = [junta]
        ponto = JointTrajectoryPoint()
        ponto.positions = [rad]
        ponto.velocities = [self.velocidade]
        msg.points = [ponto]
        self._pub.publish(msg)


class JanelaSliders(QWidget):
    def __init__(self, node: ControleManual) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle('Controle manual — motores AX-12')
        self.setFixedWidth(420)

        layout = QVBoxLayout()
        for nome, (low, high) in JUNTAS.items():
            valor_inicial = 0 if low <= 0.0 <= high else low

            label = QLabel(self._texto(nome, valor_inicial, low, high))

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(round(low * ESCALA))
            slider.setMaximum(round(high * ESCALA))
            slider.setValue(round(valor_inicial * ESCALA))
            slider.valueChanged.connect(
                lambda valor, n=nome, lb=label, lo=low, hi=high:
                    self._on_change(n, valor, lb, lo, hi))

            layout.addWidget(label)
            layout.addWidget(slider)

        self.setLayout(layout)

    @staticmethod
    def _texto(junta: str, rad: float, low: float, high: float) -> str:
        return f'{junta}:  {rad:+.2f} rad   [{low:+.2f}, {high:+.2f}]'

    def _on_change(self, junta: str, valor: int, label: QLabel, low: float, high: float) -> None:
        rad = valor / ESCALA
        label.setText(self._texto(junta, rad, low, high))
        self._node.mover(junta, rad)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControleManual()

    app = QApplication(sys.argv)
    win = JanelaSliders(node)
    win.show()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
