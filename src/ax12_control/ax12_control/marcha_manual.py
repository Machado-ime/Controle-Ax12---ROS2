#!/usr/bin/env python3
"""
Percorre MANUALMENTE as colunas de uma matriz de marcha no robô REAL.

Une o seletor de passo do passo_slider/visualizar_marcha (escolher a coluna
da matriz pelo slider) com o envio ao hardware do controle_manual: você
escolhe a etapa da marcha e o robô vai fisicamente para aquela pose, com o
RViz espelhando a posição real (telemetria do ax12_controller).

Como se diferencia dos vizinhos:
  - send_gait           : percorre TODAS as colunas sozinho, no tempo (automático).
  - visualizar_marcha   : mesma escolha de coluna, mas SÓ no RViz (sem hardware).
  - controle_manual     : move o robô real, mas por ângulo livre (sem matriz).
  - marcha_manual (este): escolhe a COLUNA da matriz e move o robô REAL.

Publica só /joint_trajectory (o mesmo tópico que o ax12_controller assina);
o RViz mostra a posição real via /joint_states publicado pelo controlador —
sem duplicar publisher de /joint_states (mesma decisão de design do
controle_manual). Como passa pelo ax12_controller, herda a correção de
juntas invertidas automaticamente.

Uso (com os motores ligados):
    ros2 launch ax12_control marcha_manual.launch.py matriz:=otimizada
"""

import signal
import sys
import threading

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, ShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Reaproveita o carregador/validador de matriz do send_gait (mesma convenção
# de nomes de arquivo e mesma validação — evita duplicar a lógica).
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


class MarchaManual(Node):
    def __init__(self) -> None:
        super().__init__('marcha_manual')
        self.declare_parameter('matriz', 'otimizada')
        matriz_nome = self.get_parameter('matriz').value
        caminho = resolver_caminho_matriz(matriz_nome)

        # Carrega e valida a matriz (mesma função do send_gait). Erro aqui
        # sobe para o main(), que aborta com mensagem antes de abrir a janela.
        self.nomes_juntas, self.matriz, self.passo, _pausa = carregar_marcha(caminho)
        self.n_passos = len(self.matriz[0])
        self.caminho = caminho

        # Coluna 0 é a pose inicial de referência; guardamos para calcular
        # velocidades coordenadas (todas as juntas chegam juntas) na transição.
        self._anteriores = [linha[0] for linha in self.matriz]

        # QoS idêntico ao do ax12_controller — BEST_EFFORT/depth=1. QoS
        # diferente = os nós nem se conectam.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(JointTrajectory, '/joint_trajectory', qos)

    def ir_para_coluna(self, coluna: int) -> None:
        """Envia a coluna (etapa) escolhida da matriz para o robô real."""
        coluna = max(0, min(coluna, self.n_passos - 1))
        posicoes = [float(linha[coluna]) for linha in self.matriz]

        # Velocidade por junta = |Δ| / passo, para todas chegarem AO MESMO
        # TEMPO (mesma lógica do send_gait). Δ=0 vira velocidade 0 aqui, mas o
        # ax12_controller trata isso como "mínimo 1 unidade" (0 = vel. máxima
        # no AX-12), então a junta parada não dispara em alta velocidade.
        velocidades = [
            abs(alvo - ant) / self.passo
            for alvo, ant in zip(posicoes, self._anteriores)
        ]
        self._anteriores = posicoes

        msg = JointTrajectory()
        msg.joint_names = self.nomes_juntas
        ponto = JointTrajectoryPoint()
        ponto.positions = posicoes
        ponto.velocities = velocidades
        ponto.time_from_start.sec = int(self.passo)
        ponto.time_from_start.nanosec = int((self.passo % 1.0) * 1e9)
        msg.points = [ponto]
        self._pub.publish(msg)


class JanelaMarcha(QWidget):
    def __init__(self, node: MarchaManual) -> None:
        super().__init__()
        self._node = node
        n = node.n_passos

        self.setWindowTitle('Marcha no robô real — passo')
        self.setFixedWidth(340)

        self._label = QLabel(f'Passo  0 / {n - 1}')
        self._label.setAlignment(Qt.AlignCenter)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(n - 1)
        self._slider.setValue(0)
        self._slider.setTickPosition(QSlider.TicksBelow)
        self._slider.setTickInterval(1)
        self._slider.setSingleStep(1)
        self._slider.valueChanged.connect(self._on_change)

        btn_prev = QPushButton('◀')
        btn_next = QPushButton('▶')
        btn_prev.setFixedWidth(44)
        btn_next.setFixedWidth(44)
        btn_prev.clicked.connect(
            lambda: self._slider.setValue(max(0, self._slider.value() - 1)))
        btn_next.clicked.connect(
            lambda: self._slider.setValue(min(n - 1, self._slider.value() + 1)))

        row = QHBoxLayout()
        row.addWidget(btn_prev)
        row.addStretch()
        row.addWidget(self._label)
        row.addStretch()
        row.addWidget(btn_next)

        layout = QVBoxLayout()
        layout.addLayout(row)
        layout.addWidget(self._slider)
        self.setLayout(layout)

        # NÃO comanda nada ao abrir: o robô só se move quando você mexe no
        # slider/botões (evita movimento súbito ao iniciar). O primeiro comando
        # usa a coluna 0 como referência de velocidade.

    def _on_change(self, valor: int) -> None:
        self._label.setText(f'Passo  {valor} / {self._node.n_passos - 1}')
        self._node.ir_para_coluna(valor)


def _spin(node: MarchaManual) -> None:
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, ShutdownException):
        pass  # ros2 launch pediu pra encerrar — não é erro


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MarchaManual()
    except FileNotFoundError:
        print('ERRO: arquivo de marcha nao encontrado. '
              'Use -p matriz:=otimizada (ou cin_inve).')
        rclpy.shutdown()
        return
    except (yaml.YAMLError, ValueError, TypeError) as e:
        print(f'ERRO no arquivo de marcha: {e}')
        rclpy.shutdown()
        return

    node.get_logger().info(
        f'Marcha "{node.caminho}" — {node.n_passos} passos. '
        'Mova o slider para o robô ir a cada etapa.')

    app = QApplication(sys.argv)
    win = JanelaMarcha(node)
    win.show()

    # O loop de eventos do Qt (em C++) bloqueia o Python de processar SIGINT.
    # Sem isto, Ctrl+C (ou o SIGINT do "ros2 launch") trava até escalar para
    # SIGKILL. O timer acorda o interpretador periodicamente para o handler
    # padrão pegar o sinal.
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
