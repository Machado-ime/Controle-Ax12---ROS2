#!/usr/bin/env python3
"""
Janela Qt com slider para controlar o passo da marcha.

Publica Int32 em /passo_marcha. O visualizar_marcha responde atualizando o RViz.
Lê o mesmo YAML de marcha para saber quantos passos existem.

Lançado automaticamente por visualizar_marcha.launch.py.
"""

import os
import sys
import threading

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Int32

try:
    from python_qt_binding.QtCore import Qt
    from python_qt_binding.QtWidgets import (
        QApplication, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (
        QApplication, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout,
        QWidget,
    )


def _n_passos_do_yaml(matriz_nome: str) -> int:
    pasta = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(pasta, f'{matriz_nome}.yaml')
    try:
        with open(yaml_path) as f:
            marcha = yaml.safe_load(f)
        return len(marcha['matriz_movimento'][0])
    except Exception:
        return 4


class SliderNode(Node):
    def __init__(self) -> None:
        super().__init__('passo_slider')
        self.declare_parameter('matriz', 'cin_inve')
        matriz_nome = self.get_parameter('matriz').value
        self.n_passos = _n_passos_do_yaml(matriz_nome)
        self._pub = self.create_publisher(Int32, '/passo_marcha', 10)

    def publicar(self, step: int) -> None:
        msg = Int32()
        msg.data = int(step)
        self._pub.publish(msg)


class JanelaSlider(QWidget):
    def __init__(self, node: SliderNode) -> None:
        super().__init__()
        self._node = node
        n = node.n_passos

        self.setWindowTitle('Marcha — passo')
        self.setFixedWidth(300)

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

    def _on_change(self, value: int) -> None:
        self._label.setText(f'Passo  {value} / {self._node.n_passos - 1}')
        self._node.publicar(value)


def main() -> None:
    rclpy.init()
    node = SliderNode()

    app = QApplication(sys.argv)
    win = JanelaSlider(node)
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
