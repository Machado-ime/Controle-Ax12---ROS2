import rclpy
from rclpy.node import Node
import time

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncWrite,
    DXL_LOBYTE, DXL_HIBYTE,
    COMM_SUCCESS
)

# =============================================================================
# CONFIGURAÇÕES DO HARDWARE
# =============================================================================
BAUDRATE         = 1_000_000
DEVICENAME       = '/dev/ttyACM0'   # Verifique com: ls /dev/tty* na Raspberry Pi
PROTOCOL_VERSION = 1.0

# Endereços da tabela de controle do AX-12A (Protocol 1.0)
ADDR_TORQUE_ENABLE  = 24
ADDR_GOAL_POSITION  = 30
ADDR_MOVING_SPEED   = 32
LEN_GOAL_POSITION   = 2

# Limites físicos do AX-12A
RAD_LIMITE     = 2.618          # ±150° em radianos (range completo = 5.236 rad)
RAD_RANGE      = 5.236          # 300° convertidos para radianos
TICKS_MAX      = 1023           # Resolução do encoder (0 a 1023)

# Fator de conversão: Rad/s → ticks de velocidade
# AX-12A: 1 unidade = 0.111 RPM → 0.111 * 2π/60 ≈ 0.01163 rad/s por tick
# Inverso: 1 rad/s ÷ 0.01163 ≈ 86.03 ticks/(rad/s)
VEL_TICKS_PER_RAD_S = 86.03


class AX12HardwareInterface(Node):
    """
    Nó ROS 2 — Hardware Interface de ESCRITA para motores Dynamixel AX-12A.

    Assina o tópico /joint_trajectory (JointTrajectory) e envia posição +
    velocidade aos motores via Dynamixel SDK (Protocol 1.0).
    Publica erros de hardware no tópico /hardware_errors (String).
    """

    def __init__(self):
        super().__init__('ax12_hardware_interface')

        # -----------------------------------------------------------------
        # Mapa junta → ID do motor  (comente as juntas inativas)
        # -----------------------------------------------------------------
        self.joint_map = {
            'PD_tornozelo_pitch_1': 1,
            'PE_tornozelo_pitch_2': 2,
            # 'PD_tornozelo_roll_3': 3,
            # 'PE_tornozelo_roll_4': 4,
            'PD_joelho_pitch_5':   5,
            'PE_joelho_pitch_6':   6,
            'PD_quadril_pitch_7':  7,
            'PE_quadril_pitch_8':  8,
        }
        self.active_ids = list(self.joint_map.values())

        # -----------------------------------------------------------------
        # 1. Publisher de erros — criado ANTES da porta serial para garantir
        #    que qualquer falha possa ser publicada no tópico.
        # -----------------------------------------------------------------
        self.error_publisher = self.create_publisher(String, '/hardware_errors', 10)

        # -----------------------------------------------------------------
        # 2. Abre a porta serial — levanta exceção em caso de falha
        # -----------------------------------------------------------------
        self.portHandler  = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        )

        if not self.portHandler.openPort():
            self._publicar_erro(f"Falha ao ABRIR a porta serial '{DEVICENAME}'. "
                                "Verifique o cabo USB e se o dispositivo está conectado.")
            raise RuntimeError(f"Não foi possível abrir '{DEVICENAME}'.")

        if not self.portHandler.setBaudRate(BAUDRATE):
            self._publicar_erro(f"Falha ao configurar o baudrate para {BAUDRATE}. "
                                "Verifique se o motor está configurado com o mesmo baudrate.")
            raise RuntimeError(f"Não foi possível configurar baudrate {BAUDRATE}.")

        self.get_logger().info(f"Porta '{DEVICENAME}' aberta @ {BAUDRATE} bps.")

        # -----------------------------------------------------------------
        # 3. Liga o torque motor por motor com delay de proteção da fonte
        # -----------------------------------------------------------------
        falhas_torque = []
        for dxl_id in self.active_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1
            )
            if dxl_comm_result != COMM_SUCCESS:
                msg = (f"Falha de COMUNICAÇÃO ao ligar torque do motor ID {dxl_id}. "
                       f"Resultado: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                self.get_logger().warn(msg)
                falhas_torque.append(dxl_id)
                self._publicar_erro(msg)
            elif dxl_error != 0:
                msg = (f"ERRO DE HARDWARE no motor ID {dxl_id} ao ligar torque: "
                       f"{self.packetHandler.getRxPacketError(dxl_error)} (cod={dxl_error})")
                self.get_logger().warn(msg)
                self._publicar_erro(msg)

            time.sleep(0.05)   # 50 ms: dá tempo à fonte estabilizar antes do próximo motor

        if falhas_torque:
            self.get_logger().warn(
                f"Torque NÃO foi ligado nos motores: {falhas_torque}. "
                "Verifique alimentação e IDs."
            )
        else:
            self.get_logger().info("Torque LIGADO em todos os motores. Pronto para comandos.")

        # -----------------------------------------------------------------
        # 4. Subscriber de trajetória
        #    QoS BEST_EFFORT + depth=1: descarta mensagens antigas, ideal para Wi-Fi
        # -----------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self._listener_callback,
            qos
        )
        self.get_logger().info("Subscriber '/joint_trajectory' criado. Interface pronta.")

    # =========================================================================
    # CALLBACK — recebe trajetória e envia para os motores
    # =========================================================================
    def _listener_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().debug("Mensagem recebida sem pontos de trajetória. Ignorando.")
            return

        point = msg.points[0]
        self.groupSyncWrite.clearParam()
        motores_enviados = 0

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name not in self.joint_map:
                continue

            dxl_id = self.joint_map[joint_name]

            # Lê posição e velocidade (com fallback seguro para 0)
            rads  = point.positions[i]  if i < len(point.positions)  else 0.0
            rad_s = point.velocities[i] if i < len(point.velocities) else 0.0

            # --- Conversão de posição: radianos → ticks (0 a 1023) ---
            rads     = max(-RAD_LIMITE, min(RAD_LIMITE, rads))
            goal_pos = int((rads + RAD_LIMITE) * (TICKS_MAX / RAD_RANGE))
            goal_pos = max(0, min(TICKS_MAX, goal_pos))

            # --- Conversão de velocidade: rad/s → ticks (1 a 1023) ---
            velocidade = int(abs(rad_s) * VEL_TICKS_PER_RAD_S)
            velocidade = max(1, min(TICKS_MAX, velocidade))

            # Escreve velocidade individualmente (sem GroupSyncWrite para velocidade)
            comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_MOVING_SPEED, velocidade
            )
            if comm_result != COMM_SUCCESS:
                erro = (f"[ID {dxl_id}] Falha ao escrever VELOCIDADE: "
                        f"{self.packetHandler.getTxRxResult(comm_result)}")
                self.get_logger().warn(erro)
                self._publicar_erro(erro)
                continue   # pula este motor, não enfileira posição
            if dxl_error != 0:
                erro = (f"[ID {dxl_id}] Erro de hardware na escrita de velocidade: "
                        f"{self.packetHandler.getRxPacketError(dxl_error)} (cod={dxl_error})")
                self.get_logger().warn(erro)
                self._publicar_erro(erro)

            # Enfileira posição no GroupSyncWrite
            param = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]
            if not self.groupSyncWrite.addParam(dxl_id, param):
                erro = f"[ID {dxl_id}] Falha ao enfileirar posição no GroupSyncWrite."
                self.get_logger().warn(erro)
                self._publicar_erro(erro)
                continue

            motores_enviados += 1

        # Envia todas as posições de uma vez
        if motores_enviados > 0:
            comm_result = self.groupSyncWrite.txPacket()
            if comm_result != COMM_SUCCESS:
                erro = (f"Falha no envio do SyncWrite para {motores_enviados} motores: "
                        f"{self.packetHandler.getTxRxResult(comm_result)}")
                self.get_logger().error(erro)
                self._publicar_erro(erro)

    # =========================================================================
    # UTILITÁRIO — publica mensagem de erro no tópico /hardware_errors
    # =========================================================================
    def _publicar_erro(self, mensagem: str):
        msg = String()
        msg.data = mensagem
        self.error_publisher.publish(msg)

    # =========================================================================
    # DESTRUIÇÃO SEGURA
    # =========================================================================
    def destroy_node(self):
        self.get_logger().info("Desligando torque e fechando porta serial...")
        for dxl_id in self.active_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0
            )
        self.portHandler.closePort()
        super().destroy_node()
        self.get_logger().info("Hardware Interface encerrado com segurança.")


# =============================================================================
# ENTRY POINT
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AX12HardwareInterface()
        rclpy.spin(node)
    except RuntimeError as e:
        # Falha na inicialização do hardware (porta, baudrate, etc.)
        print(f"\n[ERRO FATAL] {e}")
        print("O nó não pôde ser iniciado. Corrija o problema e tente novamente.\n")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
