"""Nó da cabeça do Adam: versão resumida do ax12_controller para 2 motores.

Controla só os AX-12 de ID 1 e 2 (pescoço), com a mesma interface do
ax12_controller: assina /joint_trajectory (rad, rad/s), escreve posição e
velocidade dos dois motores num único SyncWrite e publica a posição lida
em /joint_states. Juntas que não são da cabeça (as das pernas, por exemplo)
são ignoradas.

ATENÇÃO: no barramento das pernas, os IDs 1 e 2 JÁ SÃO os rolls de
tornozelo (pd_roll_tornozelo_1 e pe_roll_tornozelo_2). Este nó precisa de
um barramento próprio, num adaptador USB próprio (parâmetro `device`): no
mesmo barramento ele moveria os tornozelos, e dois processos não podem
abrir a mesma porta serial.

O que ficou de fora em relação ao ax12_controller: reconexão automática,
modo observador, /diagnostics, /hardware_errors e o balde de falhas de
leitura. Se a porta cair, o nó para de escrever e avisa no log; basta
reiniciá-lo.

Referência da tabela de controle:
https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_MAKEWORD,
)

# Tabela de controle, conversões e exceções de porta caída vêm do
# ax12_controller, para os dois nós nunca divergirem nesses números.
from ax12_control.ax12_controller import (
    ERROS_SERIAL,
    MODELOS,
    PROTOCOL_VERSION,
    ADDR_TORQUE_ENABLE,
    ADDR_GOAL_POSITION,
    LEN_GOAL_POS_E_SPEED,
    ADDR_PRESENT_POSITION,
)

# Present Position (36) + Present Speed (38): 4 bytes numa só leitura
LEN_POS_E_VEL = 4

MODELO = MODELOS['AX12']

# Juntas da cabeça (nome ROS -> ID do motor). Como nas pernas, o sufixo do
# nome é o ID no barramento. Elas ainda não existem no URDF do
# adam_description: quem definir os nomes lá troca aqui também.
JUNTAS = {
    'cabeca_yaw_1': 1,     # gira a cabeça para os lados
    'cabeca_picht_2': 2,   # inclina a cabeça para cima/baixo
}

# Limites por junta (rad). Sem medição ainda: curso inteiro do AX-12 (±150°).
# Meça no hardware e aperte, como foi feito nos tornozelos.
LIMITES = {
    'cabeca_yaw_1': (-MODELO['limite_rad'], MODELO['limite_rad']),
    'cabeca_picht_2': (-MODELO['limite_rad'], MODELO['limite_rad']),
}


class Cabeca(Node):
    """Escreve e lê os 2 motores da cabeça num barramento próprio."""

    def __init__(self):
        super().__init__('cabeca')

        # Ex.: ros2 run ax12_control cabeca --ros-args -p device:=/dev/ttyUSB0
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('velocidade_padrao', 100)   # usada se a msg vier sem velocities
        self.declare_parameter('taxa_leitura', 5.0)        # Hz do /joint_states (0 desliga)
        # A OpenCR com firmware usb_to_dxl só passa o barramento para o baud
        # do USB na volta seguinte do loop() dela (ver o ax12_controller).
        self.declare_parameter('pausa_pos_abertura', 0.2)  # segundos

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        self.velocidade_padrao = self.get_parameter('velocidade_padrao').value
        self.pausa_pos_abertura = max(0.0, self.get_parameter('pausa_pos_abertura').value)
        self.port_ok = False

        self.portHandler = PortHandler(self.device)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_GOAL_POS_E_SPEED)

        # 1. Abre a porta e liga o torque (sem porta, o nó não tem função)
        self._abrir_porta()
        self._ligar_torque()

        # 2. Comandos: BEST_EFFORT + fila de 1, como no ax12_controller
        #    (comando velho é pior que comando perdido)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            JointTrajectory, '/joint_trajectory', self.listener_callback, qos_profile)

        # 3. Telemetria: só posição e velocidade
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', qos_profile_sensor_data)
        taxa = self.get_parameter('taxa_leitura').value
        if taxa > 0:
            self.create_timer(1.0 / taxa, self.ler_motores_callback)

    # =================================================================
    # CONEXÃO E TORQUE
    # =================================================================

    def _abrir_porta(self):
        """Abre a porta no baud pedido. Levanta RuntimeError se não conseguir."""
        try:
            # setBaudRate() já abre a porta; chamar openPort() antes só a
            # abriria e fecharia uma vez a mais.
            if not self.portHandler.setBaudRate(self.baudrate):
                raise RuntimeError(
                    f'baudrate {self.baudrate} nao e suportado pelo Dynamixel SDK.')
            time.sleep(self.pausa_pos_abertura)
            self.portHandler.ser.reset_input_buffer()
        except ERROS_SERIAL as e:
            raise RuntimeError(f'Nao foi possivel abrir a porta {self.device}: {e}') from e
        self.port_ok = True
        self.get_logger().info(f'Porta {self.device} aberta.')

    def _ligar_torque(self):
        """Liga o torque motor a motor e resume quem respondeu."""
        conectados = 0
        for joint_name, dxl_id in JUNTAS.items():
            try:
                result, error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            except ERROS_SERIAL:
                self._porta_caiu('ao ligar o torque')
                return
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'Motor ID {dxl_id} ({joint_name}) NAO respondeu: '
                    f'{self.packetHandler.getTxRxResult(result)}')
            elif error != 0:
                conectados += 1
                self.get_logger().warn(
                    f'Motor ID {dxl_id} ({joint_name}) conectou, MAS reportou erro: '
                    f'{self.packetHandler.getRxPacketError(error)}')
            else:
                conectados += 1
                self.get_logger().info(f'Motor ID {dxl_id} ({joint_name}): torque LIGADO.')
            # 50 ms para a fonte estabilizar antes de ligar o próximo
            time.sleep(0.05)

        if conectados == 0:
            # Porta abriu, então o adaptador está vivo: falta energia ou cabo
            self.get_logger().warn(
                'NENHUM motor respondeu. Verifique a fonte 12 V, a chave de power '
                'da placa e o cabo de 3 pinos ate os motores.')

    def _porta_caiu(self, contexto):
        """Marca a porta como caída. Este nó não reconecta sozinho."""
        self.port_ok = False
        self.get_logger().error(
            f'PORTA SERIAL CAIU {contexto}! Este no nao reconecta: reinicie-o.')

    # =================================================================
    # ESCRITA (trajetória em rad e rad/s)
    # =================================================================

    def listener_callback(self, msg):
        if not self.port_ok or not msg.points:
            return

        # Controle imediato: só o primeiro ponto
        point = msg.points[0]

        # Mensagem malformada é descartada, nunca "consertada"
        if len(point.positions) < len(msg.joint_names):
            self.get_logger().warn(
                f'Mensagem descartada: {len(msg.joint_names)} juntas, '
                f'mas so {len(point.positions)} posicoes.')
            return
        tem_velocidades = len(point.velocities) >= len(msg.joint_names)

        self.groupSyncWrite.clearParam()
        juntas_no_pacote = 0

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name not in JUNTAS:
                continue
            dxl_id = JUNTAS[joint_name]

            # NaN passaria pelo clamp como o LIMITE SUPERIOR (toda comparação
            # com NaN é falsa), mandando a junta ao batente sem aviso.
            cmd_rad = point.positions[i]
            if not math.isfinite(cmd_rad):
                self.get_logger().warn(
                    f'{joint_name}: posicao invalida ({cmd_rad}) — junta ignorada.')
                continue

            low, high = LIMITES[joint_name]
            rads = max(low, min(high, cmd_rad))

            # rad -> 0 a 1023
            goal_pos = round((rads + MODELO['limite_rad']) * MODELO['pos_por_rad'])
            goal_pos = max(0, min(MODELO['max_pos'], goal_pos))

            # rad/s -> 1 a 1023. Mínimo 1, porque 0 significa velocidade MÁXIMA!
            if tem_velocidades and math.isfinite(point.velocities[i]):
                velocidade = round(abs(point.velocities[i]) * MODELO['unidades_por_rad_s'])
            else:
                velocidade = self.velocidade_padrao
            velocidade = max(1, min(1023, velocidade))

            # Posição (end. 30) e velocidade (end. 32) vão juntas, 4 bytes
            param = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos),
                     DXL_LOBYTE(velocidade), DXL_HIBYTE(velocidade)]
            if self.groupSyncWrite.addParam(dxl_id, param):
                juntas_no_pacote += 1

        # Nenhuma junta da cabeça na mensagem: um pacote vazio voltaria como
        # erro "Protocol does not support this function!", que não é falha.
        if juntas_no_pacote == 0:
            return

        try:
            result = self.groupSyncWrite.txPacket()
        except ERROS_SERIAL:
            self._porta_caiu('durante o envio de comando')
            return
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Falha no SyncWrite: {self.packetHandler.getTxRxResult(result)}')

    # =================================================================
    # LEITURA (posição e velocidade -> /joint_states)
    # =================================================================

    def ler_motores_callback(self):
        if not self.port_ok:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for joint_name, dxl_id in JUNTAS.items():
            try:
                dados, result, _ = self.packetHandler.readTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_POSITION, LEN_POS_E_VEL)
            except ERROS_SERIAL:
                self._porta_caiu('durante a leitura')
                return
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'Motor ID {dxl_id} ({joint_name}) nao respondeu a leitura.',
                    throttle_duration_sec=5.0)
                continue

            pos_raw = DXL_MAKEWORD(dados[0], dados[1])
            vel_raw = DXL_MAKEWORD(dados[2], dados[3])

            # Velocidade: 10 bits + bit de direção (>= 1024 = horário)
            vel_rad_s = (vel_raw & 0x3FF) / MODELO['unidades_por_rad_s']
            if vel_raw >= 1024:
                vel_rad_s = -vel_rad_s

            js.name.append(joint_name)
            js.position.append(pos_raw / MODELO['pos_por_rad'] - MODELO['limite_rad'])
            js.velocity.append(vel_rad_s)

        # rclpy.ok(): no Ctrl+C o contexto morre antes do fim do callback
        if js.name and rclpy.ok():
            self.joint_state_publisher.publish(js)

    # =================================================================
    # ENCERRAMENTO SEGURO
    # =================================================================

    def destroy_node(self):
        # Desliga o torque ao sair (só se a porta estiver viva)
        if self.port_ok:
            try:
                for dxl_id in JUNTAS.values():
                    self.packetHandler.write1ByteTxRx(
                        self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.portHandler.closePort()
            except ERROS_SERIAL:
                self.get_logger().warn('Porta indisponivel no encerramento; torque nao desligado.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Cabeca()
    except RuntimeError as e:
        print(f'ERRO: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass            # Ctrl+C ou SIGTERM (ros2 launch, systemd)
    except Exception:
        # Corrida de encerramento: contexto morto = encerrando, engole;
        # contexto vivo = erro de verdade, repassa (ver o ax12_controller).
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
