#!/usr/bin/env python3
"""
Medidor de roll: UM controlador (slider) comanda as 4 juntas de roll juntas.

Mantém os pitchs das pernas fixos na COLUNA 1 da matriz de marcha (postura
base; parâmetro 'matriz', padrão cin_inve_2) e aplica o valor único do slider
às 4 juntas de roll, cada uma com o próprio sinal (SINAIS_ROLL). Serve para
medir, no robô real, o ângulo de roll necessário para transferir o peso para
uma perna — o análogo físico da amplitude do sway lateral do CoM que o
gerador de marcha por preview control de ZMP produz (Kajita et al., 2003,
"Biped Walking Pattern Generation by using Preview Control of the
Zero-Moment Point", ICRA).

Publica só /joint_trajectory, sempre com as 10 juntas (pitchs da coluna 1 +
rolls do slider); o RViz espelha a posição real via /joint_states do
ax12_controller — mesma decisão de design do controle_manual.

Uso (com os motores ligados):
    ros2 launch ax12_control medir_roll.launch.py
    ros2 launch ax12_control medir_roll.launch.py matriz:=matriz_zmp velocidade:=0.3
"""

import math
import signal
import sys
import threading

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, ShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ax12_control.send_gait import carregar_marcha, resolver_caminho_matriz

try:
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import (
        QApplication, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout,
        QWidget,
    )

# Sinal de cada junta de roll em relação ao valor único do controlador.
# Slider POSITIVO = direção da coluna 2 ("peso_dir") da cin_inve_2, conforme
# calibrado no robô real em 2026-07-07; negativo = espelhado. Se a convenção
# física mudar (motor remontado etc.), ajuste apenas aqui.
SINAIS_ROLL = {
    'pd_roll_tornozelo_1': -1.0,
    'pe_roll_tornozelo_2': -1.0,
    # Os dois rolls de QUADRIL entraram em juntas_invertidas no ax12_controller
    # (o motor gira ao contrário do URDF). Como a inversão agora acontece lá, na
    # fronteira com o motor, estes sinais foram trocados de -1 para +1 para o
    # slider continuar inclinando o robô no mesmo sentido físico da calibração
    # de 2026-07-07 — sem isso a correção seria aplicada duas vezes.
    'pd_roll_quadril_9':   +1.0,
    'pe_roll_quadril_10':  +1.0,
}

# Curso do slider: ±30°. O limite mecânico mais apertado é o roll de
# tornozelo (±0.5934 rad ≈ ±34° na direção restritiva), então ±30° nunca
# atinge o batente (e o ax12_controller ainda clamparia se atingisse).
MAX_GRAUS = 30.0


class MedirRoll(Node):
    def __init__(self) -> None:
        super().__init__('medir_roll')
        self.declare_parameter('matriz', 'cin_inve_2')
        self.declare_parameter('velocidade', 0.5)  # rad/s de cada comando
        self.velocidade = self.get_parameter('velocidade').value

        matriz_nome = self.get_parameter('matriz').value
        caminho = resolver_caminho_matriz(matriz_nome)
        nomes, matriz, _passo, _pausa = carregar_marcha(caminho)

        # Postura base: coluna 1 da matriz. Os rolls entram zerados aqui e
        # passam a ser sobrescritos pelo slider a cada comando.
        self._pose = {nome: float(linha[0]) for nome, linha in zip(nomes, matriz)}
        for junta in SINAIS_ROLL:
            self._pose.setdefault(junta, 0.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(JointTrajectory, '/joint_trajectory', qos)
        self.get_logger().info(
            f'Postura base = coluna 1 de "{caminho}". '
            'Mova o slider para comandar os 4 rolls.')

    def aplicar_roll(self, rad: float) -> None:
        """Publica a postura completa: pitchs da coluna 1 + rolls = sinais*rad."""
        alvo = dict(self._pose)
        for junta, sinal in SINAIS_ROLL.items():
            alvo[junta] = sinal * rad

        msg = JointTrajectory()
        msg.joint_names = list(alvo.keys())
        ponto = JointTrajectoryPoint()
        ponto.positions = list(alvo.values())
        ponto.velocities = [self.velocidade] * len(alvo)
        msg.points = [ponto]
        self._pub.publish(msg)


class JanelaRoll(QWidget):
    # Slider em décimos de grau (inteiro): -300..+300 = -30.0°..+30.0°
    ESCALA = 10

    def __init__(self, node: MedirRoll) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle('Medidor de roll — 4 juntas juntas')
        self.setFixedWidth(420)

        self._label = QLabel(self._texto(0.0))
        self._label.setAlignment(Qt.AlignCenter)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setMinimum(int(-MAX_GRAUS * self.ESCALA))
        self._slider.setMaximum(int(MAX_GRAUS * self.ESCALA))
        self._slider.setValue(0)
        self._slider.setTickPosition(QSlider.TicksBelow)
        self._slider.setTickInterval(5 * self.ESCALA)  # tick a cada 5°
        self._slider.setSingleStep(self.ESCALA)        # 1° por passo de teclado
        self._slider.valueChanged.connect(self._on_change)

        btn_menos = QPushButton('◀ 1°')
        btn_mais = QPushButton('1° ▶')
        btn_zero = QPushButton('Zerar')
        btn_menos.clicked.connect(
            lambda: self._slider.setValue(self._slider.value() - self.ESCALA))
        btn_mais.clicked.connect(
            lambda: self._slider.setValue(self._slider.value() + self.ESCALA))
        btn_zero.clicked.connect(lambda: self._slider.setValue(0))

        row = QHBoxLayout()
        row.addWidget(btn_menos)
        row.addStretch()
        row.addWidget(btn_zero)
        row.addStretch()
        row.addWidget(btn_mais)

        layout = QVBoxLayout()
        layout.addWidget(self._label)
        layout.addWidget(self._slider)
        layout.addLayout(row)
        self.setLayout(layout)

        # NÃO comanda nada ao abrir: o robô só assume a postura base (e os
        # rolls) na primeira interação com o slider/botões.

    @staticmethod
    def _texto(graus: float) -> str:
        return f'Roll: {graus:+.1f}°   ({math.radians(graus):+.4f} rad)'

    def _on_change(self, valor: int) -> None:
        graus = valor / self.ESCALA
        self._label.setText(self._texto(graus))
        self._node.aplicar_roll(math.radians(graus))


def _spin(node: MedirRoll) -> None:
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, ShutdownException):
        pass  # ros2 launch pediu pra encerrar — não é erro


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MedirRoll()
    except FileNotFoundError:
        print('ERRO: arquivo de marcha nao encontrado. '
              'Use -p matriz:=cin_inve_2 (ou outra matriz instalada).')
        rclpy.shutdown()
        return
    except (yaml.YAMLError, ValueError, TypeError) as e:
        print(f'ERRO no arquivo de marcha: {e}')
        rclpy.shutdown()
        return

    app = QApplication(sys.argv)
    win = JanelaRoll(node)
    win.show()

    # O loop de eventos do Qt (em C++) bloqueia o Python de processar SIGINT.
    # Sem isto, Ctrl+C (ou o SIGINT do "ros2 launch") trava até escalar para
    # SIGKILL. O timer acorda o interpretador periodicamente.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    keep_alive = QTimer()
    keep_alive.timeout.connect(lambda: None)
    keep_alive.start(200)

    spin_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
