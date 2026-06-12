"""Painel de telemetria do robô AX-12 (roda no PC de comando).

Mostra em tabela, atualizada no terminal 2x por segundo, tudo que o
ax12_controller publica: ângulo, velocidade, torque (Present Load),
tensão, temperatura e erros de cada motor — mais os últimos alertas
de /hardware_errors.

Não exige interface gráfica: funciona até por SSH. Para gráficos de
curvas ao longo do tempo, use PlotJuggler ou rqt_plot (ver README).
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Códigos ANSI: limpa a tela e volta o cursor ao topo (efeito "htop")
LIMPA_TELA = '\x1b[2J\x1b[H'

NIVEIS = {0: 'OK', 1: 'ATENCAO', 2: 'ERRO', 3: 'SEM DADO'}


class AX12Monitor(Node):

    def __init__(self):
        super().__init__('ax12_monitor')

        # Estado mais recente de cada junta (os callbacks preenchem,
        # o timer de desenho só lê)
        self.juntas = {}
        self.alertas = deque(maxlen=5)

        # QoS de sensor (BEST_EFFORT) — PRECISA casar com o publisher
        # de /joint_states do controlador
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_states_callback, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, '/diagnostics',
            self._diagnostics_callback, 10)
        self.create_subscription(
            String, '/hardware_errors',
            self._erro_callback, 10)

        # Redesenha a tela 2x por segundo
        self.create_timer(0.5, self._desenhar)

    # ------------------ Callbacks (recebem os dados) ------------------

    def _joint_states_callback(self, msg):
        for i, nome in enumerate(msg.name):
            j = self.juntas.setdefault(nome, {})
            j['pos_rad'] = msg.position[i] if i < len(msg.position) else 0.0
            j['vel'] = msg.velocity[i] if i < len(msg.velocity) else 0.0
            j['torque_nm'] = msg.effort[i] if i < len(msg.effort) else 0.0

    def _diagnostics_callback(self, msg):
        for status in msg.status:
            # Só nos interessam os status publicados pelo ax12_controller
            if not status.name.startswith('ax12/'):
                continue
            nome = status.name.split('/', 1)[1]
            j = self.juntas.setdefault(nome, {})
            # level chega como byte (b'\x00'); convertemos para inteiro
            nivel = status.level
            if isinstance(nivel, bytes):
                nivel = nivel[0] if nivel else 0
            j['status'] = NIVEIS.get(nivel, '?')
            j['status_msg'] = status.message
            # Copia os pares chave/valor (torque_pct, tensao_v, ...)
            for kv in status.values:
                j[kv.key] = kv.value

    def _erro_callback(self, msg):
        self.alertas.append(msg.data)

    # ------------------ Desenho da tela ------------------

    def _desenhar(self):
        linhas = [LIMPA_TELA]
        linhas.append('=== MONITOR AX-12 ===  (Ctrl+C para sair)')
        linhas.append('')

        if not self.juntas:
            linhas.append('Aguardando dados de /joint_states e /diagnostics...')
            linhas.append('(o ax12_controller esta rodando na Raspberry Pi?)')
        else:
            cab = (f"{'Junta':<22}{'Ang(g)':>8}{'Vel(rad/s)':>11}"
                   f"{'Torque(%)':>10}{'N.m':>6}{'V':>6}{'Temp':>6}  Status")
            linhas.append(cab)
            linhas.append('-' * len(cab))
            for nome in sorted(self.juntas):
                j = self.juntas[nome]
                ang = math.degrees(j.get('pos_rad', 0.0))
                status = j.get('status', '?')
                if status != 'OK':
                    status = f"{status}: {j.get('status_msg', '')}"
                linhas.append(
                    f"{nome:<22}{ang:>8.1f}{j.get('vel', 0.0):>11.2f}"
                    f"{j.get('torque_pct', '--'):>10}"
                    f"{j.get('torque_nm_estimado', '--'):>6}"
                    f"{j.get('tensao_v', '--'):>6}"
                    f"{j.get('temperatura_c', '--'):>6}  {status}")

        linhas.append('')
        linhas.append('Ultimos alertas (/hardware_errors):')
        if not self.alertas:
            linhas.append('  (nenhum)')
        for alerta in self.alertas:
            linhas.append(f'  - {alerta}')

        print('\n'.join(linhas), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = AX12Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nMonitor encerrado.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
