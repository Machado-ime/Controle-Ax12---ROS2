#!/usr/bin/env python3
"""
IK cartesiana do pé: roll central + posição X / altura Z de cada pé.

Evolução do medir_roll no estilo do walking tuner do ROBOTIS OP3
(x/z_move_amplitude -> IK -> ângulos) e do pé de balanço do arcabouço ZMP de
Kajita et al. 2003 ("Biped Walking Pattern Generation by using Preview
Control of the Zero-Moment Point"): o roll desloca o centro de massa
lateralmente; o pé segue alvos X (frente/trás) e Z (altura) com o pé sempre
paralelo ao chão.

5 controles:
  - Roll central (um valor -> 4 juntas de roll, com sinais de SINAIS_ROLL);
  - X da PASSADA (um valor -> pé direito +X, pé esquerdo -X, em OPOSIÇÃO —
    mesmo esquema do walking module do OP3, que comanda right_leg = -x e
    left_leg = +x a cada ciclo: com os dois pés no chão, opor as pernas
    translada o TRONCO mantendo-o ereto; comandar só uma perna cria uma
    cadeia fechada inconsistente e o robô acomoda inclinando o tronco);
  - Z do pé DIREITO;  Z do pé ESQUERDO;
  - Trim do quadril (pitch) — offset cru somado aos DOIS quadris depois da
    IK, espelhado entre PD/PE (mesma direção física). É o hip_pitch_offset
    do OP3 ("pitch offset at the hip level", default 13° lá): compensa a
    deflexão dos servos sob o momento do peso do tronco, que faz o tronco
    ceder pra frente mesmo com os ângulos cinematicamente corretos.
X/Z são deslocamentos em mm em relação à POSTURA BASE (coluna 1 da matriz do
parâmetro 'matriz'; tudo em 0 reproduz exatamente a coluna 1). A IK resolve
os pitchs (quadril/joelho) por perna com Newton 2x2 sobre a FK exata do
URDF, com tornozelo = -(quadril+joelho) impondo pé paralelo ao tronco (a
restrição verificada por FK em todas as matrizes validadas). O trim é a
única exceção deliberada à soma zero: ele inclina o tronco em relação às
pernas de propósito, para o tronco REAL ficar vertical.

Publica só /joint_trajectory, sempre com as 10 juntas; o RViz espelha a
posição real via /joint_states do ax12_controller (mesmo design dos irmãos
controle_manual / medir_roll). Botão "Exportar coluna" imprime os 10 ângulos
na ordem de nomes_juntas da matriz, prontos para virar uma coluna do YAML.

Uso (com os motores ligados):
    ros2 launch ax12_control controle_pe.launch.py
    ros2 launch ax12_control controle_pe.launch.py matriz:=matriz_zmp velocidade:=0.3
"""

import math
import signal
import sys
import threading
import xml.etree.ElementTree as ET

import numpy as np
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
        QApplication, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
        QVBoxLayout, QWidget,
    )
except ImportError:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
        QVBoxLayout, QWidget,
    )

# Sinais do roll central (copiados do medir_roll em 2026-07-08 — mantenha os
# dois arquivos em sincronia se a convenção física mudar).
SINAIS_ROLL = {
    'pd_roll_tornozelo_1': -1.0,
    'pe_roll_tornozelo_2': -1.0,
    'pd_roll_quadril_9':   -1.0,
    'pe_roll_quadril_10':  -1.0,
}

# Curso dos controles
MAX_ROLL_GRAUS = 30.0   # limite mecânico mais apertado do roll ~34°
X_MAX_MM = 60.0         # meia-passada: pé dir +X e pé esq -X (oposição)
Z_MIN_MM = -20.0        # abaixa/estende a perna
Z_MAX_MM = 60.0         # levanta o pé
TRIM_MAX_GRAUS = 15.0   # curso do trim de pitch do quadril (OP3 usa 13°)

# Sinais do trim do quadril: PD/PE são espelhados no URDF (mesmo padrão dos
# limites e do roll), então a MESMA direção física de inclinação do tronco
# exige incrementos de sinais opostos. Se no robô o slider positivo inclinar
# o tronco para o lado errado, inverta os DOIS sinais (como foi feito na
# calibração de SINAIS_ROLL via medir_roll).
SINAIS_TRIM_QUADRIL = {
    'pd_picht_quadril_7': +1.0,
    'pe_pich_quadril_8':  -1.0,
}

# Margem de segurança do alcance máximo (perna 100% esticada = singularidade
# do Jacobiano) — mesma constante 0.999 do ik_2link.m.
MARGEM_ALCANCE = 0.999

# Cadeia de cada perna no URDF: joints de pitch e o link cuja origem é o
# joint de pitch do tornozelo (alvo da IK).
PERNAS = {
    # link_coxa/link_canela = link cuja ORIGEM é o pivô do quadril/joelho
    # (independe do próprio ângulo da junta — girar em torno de um ponto não
    # move o ponto). link_alvo = origem do pivô do tornozelo (alvo da IK).
    'dir': dict(quadril='pd_picht_quadril_7', joelho='pd_picht_joelho_5',
                tornozelo='pd_picht_tornozelo_3', link_coxa='pd_coxa',
                link_canela='pd_canela', link_alvo='pd_tornozelo'),
    'esq': dict(quadril='pe_pich_quadril_8', joelho='pe_picht_joelho_6',
                tornozelo='pe_picht_tornozelo_4', link_coxa='pe_coxa',
                link_canela='pe_canela', link_alvo='pe_tornozelo'),
}

# Limites por junta (rad) — mesmos valores de ax12_controller.joint_limits;
# mantenha em sincronia (mesma regra do controle_manual.py).
LIMITE_RAD = 2.618
LIMITES = {
    'pd_picht_tornozelo_3': (-1.4661,     0.5585),
    'pe_picht_tornozelo_4': (-0.5585,     1.4661),
    'pd_picht_joelho_5':    (0.0,         LIMITE_RAD),
    'pe_picht_joelho_6':    (-LIMITE_RAD, 0.0),
    'pd_picht_quadril_7':   (-LIMITE_RAD, LIMITE_RAD),
    'pe_pich_quadril_8':    (-LIMITE_RAD, LIMITE_RAD),
}


# =====================================================================
# Cinemática pura (sem ROS) — parse do URDF, FK e IK por perna
# =====================================================================

def _rpy_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _axis_R(axis, th):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


class CadeiaURDF:
    """FK mínima sobre o URDF (mesmo chain-walk validado na análise offline)."""

    def __init__(self, urdf_str: str) -> None:
        root = ET.fromstring(urdf_str)
        self._joint_por_filho = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
            ax = j.find('axis')
            axis = ([float(v) for v in ax.get('xyz').split()]
                    if ax is not None else [0.0, 0.0, 1.0])
            self._joint_por_filho[j.find('child').get('link')] = dict(
                name=j.get('name'), type=j.get('type'),
                parent=j.find('parent').get('link'),
                xyz=xyz, rpy=rpy, axis=axis)

    def _cadeia(self, link):
        c = []
        while link in self._joint_por_filho:
            j = self._joint_por_filho[link]
            c.append(j)
            link = j['parent']
        return list(reversed(c))

    def pose_link(self, link, q):
        """Matriz 4x4 da origem do link no frame base, com ângulos q (dict)."""
        T = np.eye(4)
        for j in self._cadeia(link):
            Tj = np.eye(4)
            Tj[:3, :3] = _rpy_R(*j['rpy'])
            Tj[:3, 3] = j['xyz']
            T = T @ Tj
            if j['type'] == 'revolute':
                Tr = np.eye(4)
                Tr[:3, :3] = _axis_R(j['axis'], q.get(j['name'], 0.0))
                T = T @ Tr
        return T

    def pos_link(self, link, q):
        return self.pose_link(link, q)[:3, 3]


def cos_lei_cossenos(d, L1, L2):
    """Ângulo interno do triângulo (lados L1, L2, d) oposto ao lado d —
    geometricamente invariante, não depende de convenção de eixo/zero de
    junta nenhuma (mesma lei dos cossenos de ik_2link.m). Usado só como
    referência/diagnóstico; a IK de verdade usa Newton sobre a FK real do
    URDF (ik_perna), porque a convenção de "ângulo zero" de cada junta
    deste URDF vem de offsets arbitrários do CAD, não de uma referência
    vertical conhecida como no modelo de passo_pitch.m/Kajita 2003 — uma
    fórmula fechada baseada em atan2 a partir da vertical dá ângulos
    completamente errados aqui (testado: erros de centenas de mm).
    """
    d2 = d * d
    cos_th = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    return max(-1.0, min(1.0, cos_th))


def ik_perna(cadeia: CadeiaURDF, perna: dict, alvo_xz, seed):
    """IK da perna: acha (quadril, joelho) que leva a origem do tornozelo a
    alvo_xz = (x, z) no frame base, com tornozelo = -(quadril+joelho) [pé
    plano] e rolls em 0. Newton 2x2 sobre a FK exata do URDF — agnóstico a
    qualquer convenção de "ângulo zero" de junta, porque usa a FK de
    verdade em vez de fórmulas fechadas que assumem eixo vertical.

    Retorna (quadril, joelho, tornozelo) ou None se não convergir.
    """
    def fk(th, tk):
        q = {perna['quadril']: th, perna['joelho']: tk,
             perna['tornozelo']: -(th + tk)}
        p = cadeia.pos_link(perna['link_alvo'], q)
        return np.array([p[0], p[2]])

    alvo = np.asarray(alvo_xz, float)
    th, tk = float(seed[0]), float(seed[1])
    d = 1e-6
    for _ in range(40):
        p = fk(th, tk)
        e = alvo - p
        if np.linalg.norm(e) < 1e-5:
            break
        J = np.column_stack([(fk(th + d, tk) - p) / d,
                             (fk(th, tk + d) - p) / d])
        try:
            passo = np.linalg.solve(J, e)
        except np.linalg.LinAlgError:
            return None
        n = np.linalg.norm(passo)
        if n > 0.3:  # limita o passo para não pular de ramo (joelho invertido)
            passo *= 0.3 / n
        th += passo[0]
        tk += passo[1]
    if np.linalg.norm(fk(th, tk) - alvo) > 1e-4:  # 0.1 mm
        return None
    return th, tk, -(th + tk)


# =====================================================================
# Nó ROS
# =====================================================================

class ControlePe(Node):
    def __init__(self) -> None:
        super().__init__('controle_pe')
        self.declare_parameter('matriz', 'cin_inve_2')
        self.declare_parameter('velocidade', 0.5)   # rad/s de cada comando
        self.declare_parameter('robot_description', '')
        self.velocidade = self.get_parameter('velocidade').value

        urdf_str = self.get_parameter('robot_description').value
        if not urdf_str:
            raise ValueError('robot_description vazio — use o launch file '
                             '(controle_pe.launch.py), que injeta o URDF.')
        self.cadeia = CadeiaURDF(urdf_str)

        matriz_nome = self.get_parameter('matriz').value
        caminho = resolver_caminho_matriz(matriz_nome)
        nomes, matriz, _passo, _pausa = carregar_marcha(caminho)
        self.nomes_matriz = nomes
        self._pose_base = {n: float(l[0]) for n, l in zip(nomes, matriz)}
        for junta in SINAIS_ROLL:
            self._pose_base.setdefault(junta, 0.0)

        # Ângulos e posição do tornozelo da postura base (rolls zerados na FK)
        self._base = {}
        for lado, perna in PERNAS.items():
            ang = (self._pose_base[perna['quadril']],
                   self._pose_base[perna['joelho']],
                   self._pose_base[perna['tornozelo']])
            q = {perna['quadril']: ang[0], perna['joelho']: ang[1],
                 perna['tornozelo']: ang[2]}
            p = self.cadeia.pos_link(perna['link_alvo'], q)
            self._base[lado] = dict(ang=ang, pos_xz=np.array([p[0], p[2]]))

        soma_d = sum(self._base['dir']['ang'])
        soma_e = sum(self._base['esq']['ang'])
        if abs(soma_d) > 1e-3 or abs(soma_e) > 1e-3:
            self.get_logger().warn(
                f'Coluna 1 da matriz não tem pé plano (somas: dir={soma_d:+.4f}, '
                f'esq={soma_e:+.4f}) — a IK vai nivelar o pé mesmo assim.')

        self._ultima_pose = dict(self._pose_base)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(JointTrajectory, '/joint_trajectory', qos)
        self.get_logger().info(
            f'Postura base = coluna 1 de "{caminho}". X/Z em mm relativos à '
            'base; a IK mantém o pé paralelo ao chão.')

    def aplicar(self, roll_rad, x_mm, dz_dir_mm, dz_esq_mm, trim_rad):
        """Resolve a IK das duas pernas e publica a pose completa.

        x_mm é a MEIA-PASSADA: pé direito recebe +x, pé esquerdo -x (pernas
        em oposição, como o walking do OP3) — com os dois pés no chão isso
        translada o tronco mantendo-o ereto. trim_rad é somado aos quadris
        DEPOIS da IK (espelhado via SINAIS_TRIM_QUADRIL), como o
        hip_pitch_offset do OP3.

        Retorna dict lado -> (quadril, joelho, tornozelo) já com trim, ou
        None por perna (falha de IK/limite); só publica se as DUAS pernas
        forem válidas.
        """
        deltas = {'dir': (x_mm, dz_dir_mm), 'esq': (-x_mm, dz_esq_mm)}
        resultado = {}
        pose = dict(self._pose_base)

        for lado, perna in PERNAS.items():
            dx, dz = deltas[lado]
            alvo = self._base[lado]['pos_xz'] + np.array([dx, dz]) / 1000.0
            seed = self._base[lado]['ang'][:2]
            sol = ik_perna(self.cadeia, perna, alvo, seed)
            if sol is not None:
                # Trim entra ANTES da checagem de limites: o valor comandado
                # ao motor é o que precisa estar dentro do curso.
                trim = SINAIS_TRIM_QUADRIL[perna['quadril']] * trim_rad
                sol = (sol[0] + trim, sol[1], sol[2])
                nomes_j = (perna['quadril'], perna['joelho'], perna['tornozelo'])
                for nome, valor in zip(nomes_j, sol):
                    lo, hi = LIMITES[nome]
                    if not (lo - 1e-6 <= valor <= hi + 1e-6):
                        self.get_logger().warn(
                            f'{nome}: {valor:+.4f} rad fora do limite '
                            f'[{lo:+.4f}, {hi:+.4f}] — pose não enviada.')
                        sol = None
                        break
            if sol is None:
                resultado[lado] = None
                continue
            resultado[lado] = sol
            for nome, valor in zip(
                    (perna['quadril'], perna['joelho'], perna['tornozelo']), sol):
                pose[nome] = float(valor)

        if resultado['dir'] is None or resultado['esq'] is None:
            return resultado  # mantém a última pose válida no robô

        for junta, sinal in SINAIS_ROLL.items():
            pose[junta] = sinal * roll_rad

        # Velocidade PROPORCIONAL à distância de cada junta (mesma lógica de
        # send_gait.py/marcha_manual.py): sem isso, juntas com deslocamento
        # menor "chegam" antes das com deslocamento maior, e a restrição de
        # pé plano — que depende dos 3 pitchs terem chegado juntos — fica
        # temporariamente violada durante o movimento, mesmo a pose final
        # sendo plana (confirmado por FK offline). self.velocidade continua
        # sendo a velocidade MÁXIMA: a junta com maior delta usa exatamente
        # esse valor; as demais são proporcionalmente mais lentas para
        # chegarem no mesmo instante.
        nomes = list(pose.keys())
        deltas = [abs(pose[n] - self._ultima_pose.get(n, pose[n])) for n in nomes]
        delta_max = max(deltas) if deltas else 0.0
        tempo = delta_max / self.velocidade if delta_max > 1e-9 else 0.0
        velocidades = [
            (d / tempo if tempo > 1e-9 else self.velocidade) for d in deltas]

        msg = JointTrajectory()
        msg.joint_names = nomes
        ponto = JointTrajectoryPoint()
        ponto.positions = [pose[n] for n in nomes]
        ponto.velocities = velocidades
        msg.points = [ponto]
        self._pub.publish(msg)
        self._ultima_pose = pose
        return resultado

    def exportar_coluna(self):
        """Loga a última pose publicada como coluna YAML (ordem da matriz)."""
        valores = [self._ultima_pose.get(n, 0.0) for n in self.nomes_matriz]
        linhas = '\n'.join(
            f'  {n:22s} {v:+.6f}' for n, v in zip(self.nomes_matriz, valores))
        coluna = ', '.join(f'{v:.6f}' for v in valores)
        self.get_logger().info(
            '\n--- Exportar coluna (ordem de nomes_juntas da matriz) ---\n'
            f'{linhas}\n'
            f'coluna: [ {coluna} ]')


# =====================================================================
# Janela Qt
# =====================================================================

class JanelaPe(QWidget):
    ESC_ROLL = 10   # décimos de grau
    ESC_MM = 1      # 1 mm por unidade

    def __init__(self, node: ControlePe) -> None:
        super().__init__()
        self._node = node
        self._armado = False   # nada é comandado até a 1ª interação
        self.setWindowTitle('Controle do pé — IK (roll + X/Z por pé)')
        self.setFixedWidth(460)

        grid = QGridLayout()
        linha = 0

        self._s_roll, linha = self._add_slider(
            grid, linha, 'Roll central',
            int(-MAX_ROLL_GRAUS * self.ESC_ROLL), int(MAX_ROLL_GRAUS * self.ESC_ROLL))
        self._s_x, linha = self._add_slider(
            grid, linha, 'X da passada (dir +X / esq −X, mm)',
            int(-X_MAX_MM), int(X_MAX_MM))
        self._s_zd, linha = self._add_slider(
            grid, linha, 'Pé DIREITO — Z (mm)', int(Z_MIN_MM), int(Z_MAX_MM))
        self._s_ze, linha = self._add_slider(
            grid, linha, 'Pé ESQUERDO — Z (mm)', int(Z_MIN_MM), int(Z_MAX_MM))
        self._s_trim, linha = self._add_slider(
            grid, linha, 'Trim quadril (pitch)',
            int(-TRIM_MAX_GRAUS * self.ESC_ROLL), int(TRIM_MAX_GRAUS * self.ESC_ROLL))

        self._status = QLabel('Mova um controle para comandar o robô.')
        self._status.setWordWrap(True)

        btn_zero = QPushButton('Zerar tudo (postura base)')
        btn_zero.clicked.connect(self._zerar)
        btn_export = QPushButton('Exportar coluna')
        btn_export.clicked.connect(self._node.exportar_coluna)

        botoes = QHBoxLayout()
        botoes.addWidget(btn_zero)
        botoes.addWidget(btn_export)

        layout = QVBoxLayout()
        layout.addLayout(grid)
        layout.addWidget(self._status)
        layout.addLayout(botoes)
        self.setLayout(layout)

    def _add_slider(self, grid, linha, titulo, minimo, maximo):
        rotulo = QLabel(titulo)
        valor = QLabel('0')
        valor.setMinimumWidth(90)
        valor.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(minimo)
        slider.setMaximum(maximo)
        slider.setValue(0)
        slider.valueChanged.connect(lambda _v, s=slider, l=valor: self._mudou(s, l))
        slider._label_valor = valor
        grid.addWidget(rotulo, linha, 0)
        grid.addWidget(valor, linha, 1)
        grid.addWidget(slider, linha + 1, 0, 1, 2)
        return slider, linha + 2

    def _texto_valor(self, slider):
        if slider in (self._s_roll, self._s_trim):
            graus = slider.value() / self.ESC_ROLL
            return f'{graus:+.1f}°  ({math.radians(graus):+.4f} rad)'
        return f'{slider.value():+d} mm'

    def _mudou(self, slider, label):
        label.setText(self._texto_valor(slider))
        self._armado = True
        self._recalcular()

    def _zerar(self):
        self._armado = True
        for s in (self._s_roll, self._s_x, self._s_zd, self._s_ze, self._s_trim):
            s.blockSignals(True)
            s.setValue(0)
            s._label_valor.setText(self._texto_valor(s))
            s.blockSignals(False)
        self._recalcular()

    def _recalcular(self):
        if not self._armado:
            return
        roll = math.radians(self._s_roll.value() / self.ESC_ROLL)
        trim = math.radians(self._s_trim.value() / self.ESC_ROLL)
        res = self._node.aplicar(
            roll,
            self._s_x.value(),
            self._s_zd.value(), self._s_ze.value(),
            trim)
        partes = []
        for lado, rotulo in (('dir', 'DIR'), ('esq', 'ESQ')):
            sol = res[lado]
            if sol is None:
                partes.append(f'{rotulo}: FORA DE ALCANCE/LIMITE — pose não enviada')
            else:
                partes.append(
                    f'{rotulo}: quadril {sol[0]:+.3f}  joelho {sol[1]:+.3f}  '
                    f'tornozelo {sol[2]:+.3f}')
        self._status.setText('\n'.join(partes))


def _spin(node: ControlePe) -> None:
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, ShutdownException):
        pass  # ros2 launch pediu pra encerrar — não é erro


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ControlePe()
    except FileNotFoundError:
        print('ERRO: arquivo de marcha nao encontrado. '
              'Use -p matriz:=cin_inve_2 (ou outra matriz instalada).')
        rclpy.shutdown()
        return
    except (yaml.YAMLError, ValueError, TypeError) as e:
        print(f'ERRO: {e}')
        rclpy.shutdown()
        return

    app = QApplication(sys.argv)
    win = JanelaPe(node)
    win.show()

    # O loop de eventos do Qt (em C++) bloqueia o Python de processar SIGINT.
    # O timer acorda o interpretador periodicamente (mesmo fix dos irmãos).
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
