"""Interface de hardware ROS 2 para servomotores Dynamixel AX-12.

Este nó é o ÚNICO processo que toca o barramento serial dos motores.
Ele assina /joint_trajectory (posições em rad, velocidades em rad/s),
converte para as unidades do AX-12 e escreve tudo num único pacote
SyncWrite. Falhas de hardware são publicadas em /hardware_errors.

Referência da tabela de controle do AX-12:
https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

# pyserial: usado apenas para reconhecer a exceção lançada quando a USB cai
import serial

# Importações explícitas (em vez de "import *") para sabermos o que vem do SDK
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
)

# =====================================================================
# Tabela de controle do AX-12 (Protocolo 1.0)
# =====================================================================
PROTOCOL_VERSION        = 1.0
ADDR_TORQUE_ENABLE      = 24   # 1 byte  (0 = desliga, 1 = liga)
ADDR_GOAL_POSITION      = 30   # 2 bytes (0 a 1023 = 0° a 300°)
ADDR_MOVING_SPEED       = 32   # 2 bytes (0 a 1023; 0 = velocidade MÁXIMA!)

# Goal Position (30) e Moving Speed (32) são vizinhos na tabela de controle.
# Escrevendo 4 bytes a partir do endereço 30, enviamos posição E velocidade
# num único pacote SyncWrite: todos os motores partem juntos, já na
# velocidade certa, sem precisar de uma escrita individual por motor.
LEN_GOAL_POS_E_SPEED    = 4

# =====================================================================
# Fatores de conversão (unidades do ROS <-> unidades do AX-12)
# =====================================================================
LIMITE_RAD          = 2.618                       # ±150° (curso útil do AX-12)
POS_POR_RAD         = 1023.0 / (2 * LIMITE_RAD)   # rad -> unidades de posição
UNIDADES_POR_RAD_S  = 86.03                       # rad/s -> unidades (1 un. = 0,111 rpm)


class AX12HardwareInterface(Node):

    def __init__(self):
        super().__init__('ax12_hardware_interface')

        # --- Parâmetros ROS (mude sem editar o código) ---
        # Ex.: ros2 run ax12_control ax12_controller --ros-args -p device:=/dev/ttyUSB0
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('tentativas_abertura', 5)    # tentativas ao iniciar o nó
        self.declare_parameter('max_falhas_reconexao', 10)  # desiste após N reconexões falhas
        self.declare_parameter('velocidade_padrao', 100)    # usada se a msg vier sem velocities

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        self.velocidade_padrao = self.get_parameter('velocidade_padrao').value
        self.max_falhas_reconexao = self.get_parameter('max_falhas_reconexao').value

        # Mapa das juntas (nome ROS -> ID do motor no barramento)
        self.joint_map = {
            'PD_tornozelo_pitch_1': 1,
            'PE_tornozelo_pitch_2': 2,
            #'PD_tornozelo_roll_3': 3,
            #'PE_tornozelo_roll_4': 4,
            'PD_joelho_pitch_5': 5,
            'PE_joelho_pitch_6': 6,
            'PD_quadril_pitch_7': 7,
            'PE_quadril_pitch_8': 8,
            #'PD_quadril-roll_9': 9,
            #'PE_quadril-roll_10': 10,
            #'BD_ombro-roll_11': 11,
            #'BE_ombro-roll_12': 12,
            #'BD_ombro-pitch_13': 13,
            #'BE_ombro-pitch_14': 14,
            #'BD_cotovelo_15': 15,
            #'BE_cotovelo_16': 16,
            #'C_pescoco_tilt_17': 17,
            #'C_pescoco_pan_18': 18
        }
        self.active_ids = list(self.joint_map.values())

        # --- Estado da conexão serial ---
        self.port_ok = False          # a porta está aberta e funcionando?
        self.falhas_reconexao = 0     # reconexões falhas consecutivas
        self.desativado = False       # True = desistimos do hardware (falha fatal)

        # --- Publisher de erros de hardware ---
        # QoS RELIABLE (padrão): aviso de erro é raro e PRECISA chegar,
        # ao contrário dos comandos de marcha, que são frequentes e descartáveis.
        self.error_publisher = self.create_publisher(String, '/hardware_errors', 10)

        # --- Objetos do Dynamixel SDK ---
        self.portHandler = PortHandler(self.device)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_GOAL_POS_E_SPEED)

        # 1. Abre a porta serial (com várias tentativas: a USB pode demorar
        #    a enumerar logo após o boot da Raspberry Pi)
        if not self._abrir_porta_com_tentativas():
            # Sem hardware o nó não tem função: aborta a criação.
            # O main() captura este erro e encerra de forma limpa.
            raise RuntimeError(f'Nao foi possivel abrir a porta {self.device}.')

        # 2. Liga o torque dos motores mapeados
        self._ligar_torque()
        self.get_logger().info('Torque LIGADO. Pronto para receber e escrever comandos.')

        # --- PERFIL DE REDE (QoS) BLINDADO PARA WI-FI ---
        # BEST_EFFORT + fila de 1: comando perdido é descartado, nunca
        # reenviado atrasado (comando velho é pior que comando perdido).
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 3. Assina o tópico de trajetórias
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.listener_callback,
            qos_profile
        )

    # =================================================================
    # FUNÇÕES DE APOIO (conexão, torque e avisos de erro)
    # =================================================================

    def _avisar_erro(self, texto):
        """Mostra o aviso no terminal E publica em /hardware_errors."""
        self.get_logger().warn(texto)
        msg = String()
        msg.data = texto
        self.error_publisher.publish(msg)

    def _abrir_porta(self):
        """Uma única tentativa de abrir a porta. Retorna True se conseguiu."""
        try:
            if self.portHandler.openPort() and self.portHandler.setBaudRate(self.baudrate):
                self.port_ok = True
                return True
        except (serial.SerialException, OSError):
            pass  # porta inexistente/ocupada: tratado como falha normal
        self.port_ok = False
        return False

    def _abrir_porta_com_tentativas(self):
        """Tenta abrir a porta N vezes antes de desistir (N = parâmetro ROS)."""
        tentativas = self.get_parameter('tentativas_abertura').value
        for tentativa in range(1, tentativas + 1):
            if self._abrir_porta():
                self.get_logger().info(
                    f'Porta {self.device} aberta com sucesso (tentativa {tentativa}).')
                return True
            self.get_logger().warn(
                f'Falha ao abrir {self.device} (tentativa {tentativa}/{tentativas}). '
                'Tentando de novo em 2 s...')
            time.sleep(2.0)
        return False

    def _ligar_torque(self):
        """Liga o torque motor a motor, conferindo se cada um respondeu."""
        for dxl_id in self.active_ids:
            try:
                result, error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            except (serial.SerialException, OSError):
                self._porta_caiu('ao ligar o torque')
                return
            if result != COMM_SUCCESS:
                self._avisar_erro(
                    f'Motor ID {dxl_id} nao respondeu ao ligar torque: '
                    f'{self.packetHandler.getTxRxResult(result)}')
            elif error != 0:
                self._avisar_erro(
                    f'Motor ID {dxl_id} reportou erro de hardware: '
                    f'{self.packetHandler.getRxPacketError(error)}')
            # VITAL: 50 ms para a fonte estabilizar antes de ligar o próximo
            time.sleep(0.05)

    def _porta_caiu(self, contexto):
        """Marca a porta como caída e avisa. A reconexão acontece no callback."""
        self.port_ok = False
        self._avisar_erro(
            f'PORTA SERIAL CAIU {contexto}! Tentando reconectar a cada comando recebido...')

    def _tentar_reconectar(self):
        """Tenta reerguer a conexão. Após max_falhas_reconexao, desiste de vez."""
        try:
            self.portHandler.closePort()
        except (serial.SerialException, OSError):
            pass  # a porta já estava morta; só queríamos liberar o descritor

        if self._abrir_porta():
            self.falhas_reconexao = 0
            self._avisar_erro('Porta serial RECONECTADA. Religando o torque dos motores.')
            # Se os motores perderam energia no evento, voltaram com torque OFF
            self._ligar_torque()
            return

        self.falhas_reconexao += 1
        if self.falhas_reconexao >= self.max_falhas_reconexao:
            self.desativado = True
            self._avisar_erro(
                f'FALHA FATAL: {self.falhas_reconexao} reconexoes falharam. '
                'Desativando a escrita nos motores. Verifique o cabo USB e reinicie o no.')

    # =================================================================
    # FUNÇÃO DE ESCRITA (recebe trajetória em radianos e rad/s)
    # =================================================================

    def listener_callback(self, msg):
        # Falha fatal anterior: ignora tudo até o nó ser reiniciado
        if self.desativado:
            return

        # Porta caída: usa a chegada de cada comando como gatilho de reconexão
        if not self.port_ok:
            self._tentar_reconectar()
            if not self.port_ok:
                return

        # Trajetória vazia não comanda nada
        if not msg.points:
            return

        # Para controle imediato, usamos apenas o primeiro ponto
        point = msg.points[0]

        # --- VALIDAÇÃO: mensagem malformada é DESCARTADA, nunca "consertada" ---
        # (o código antigo assumia 0.0 rad para posição faltante, o que mandava
        # o motor para o centro sem ninguém pedir)
        if len(point.positions) < len(msg.joint_names):
            self._avisar_erro(
                f'Mensagem descartada: {len(msg.joint_names)} juntas, '
                f'mas so {len(point.positions)} posicoes.')
            return

        # Velocidades são opcionais no JointTrajectory; se não vierem,
        # usamos a velocidade padrão (parâmetro) em vez de quase-zero.
        tem_velocidades = len(point.velocities) >= len(msg.joint_names)

        self.groupSyncWrite.clearParam()

        for i, joint_name in enumerate(msg.joint_names):
            # Junta que este controlador não conhece: ignora
            if joint_name not in self.joint_map:
                continue
            dxl_id = self.joint_map[joint_name]

            # --- CONVERSÃO DE POSIÇÃO (rad -> 0 a 1023) ---
            rads = max(-LIMITE_RAD, min(LIMITE_RAD, point.positions[i]))
            goal_pos = round((rads + LIMITE_RAD) * POS_POR_RAD)
            goal_pos = max(0, min(1023, goal_pos))

            # --- CONVERSÃO DE VELOCIDADE (rad/s -> 1 a 1023) ---
            # Mínimo 1, porque 0 significa "velocidade máxima" no AX-12!
            if tem_velocidades:
                velocidade = round(abs(point.velocities[i]) * UNIDADES_POR_RAD_S)
            else:
                velocidade = self.velocidade_padrao
            velocidade = max(1, min(1023, velocidade))

            # --- EMPACOTA posição (2 bytes) + velocidade (2 bytes) juntas ---
            param = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos),
                     DXL_LOBYTE(velocidade), DXL_HIBYTE(velocidade)]
            if not self.groupSyncWrite.addParam(dxl_id, param):
                self._avisar_erro(f'addParam falhou para o motor ID {dxl_id} (ID repetido?).')

        # --- ENVIA TUDO num único pacote broadcast ---
        try:
            result = self.groupSyncWrite.txPacket()
        except (serial.SerialException, OSError):
            self._porta_caiu('durante o envio de comando')
            return

        if result != COMM_SUCCESS:
            self._avisar_erro(
                f'Falha no SyncWrite: {self.packetHandler.getTxRxResult(result)}')

    # =================================================================
    # ENCERRAMENTO SEGURO
    # =================================================================

    def destroy_node(self):
        # Só toca na serial se ela estiver viva (evita traceback no Ctrl+C
        # quando a porta nunca abriu ou caiu no meio da operação)
        if self.port_ok:
            try:
                for dxl_id in self.active_ids:
                    self.packetHandler.write1ByteTxRx(
                        self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.portHandler.closePort()
            except (serial.SerialException, OSError):
                self.get_logger().warn('Porta indisponivel no encerramento; torque nao desligado.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AX12HardwareInterface()
    except RuntimeError as e:
        # Porta não abriu nem com as tentativas: encerra limpo, sem traceback
        print(f'ERRO: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
