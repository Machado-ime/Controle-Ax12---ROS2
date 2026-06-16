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
# CONFIGURAÇÕES DE HARDWARE
# =============================================================================
BAUDRATE         = 1_000_000
DEVICENAME       = '/dev/ttyACM0'   # Ajuste conforme: ls /dev/tty* na Raspberry Pi
PROTOCOL_VERSION = 1.0

# Tabela de controle do AX-12A — Protocol 1.0
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30   # 2 bytes — inicia o bloco contínuo
ADDR_MOVING_SPEED  = 32   # 2 bytes — imediatamente após goal_position
LEN_SYNC_BLOCK     = 4    # 2 bytes (posição) + 2 bytes (velocidade) = 1 pacote só

# Limites físicos do AX-12A
RAD_LIMITE = 2.618          # ±150° em radianos
RAD_RANGE  = 5.236          # 300° total em radianos
TICKS_MAX  = 1023           # Resolução do encoder

# Conversão de velocidade (Ref: e-Manual AX-12A)
# 1 unidade = 0.111 RPM → 0.111 × (2π / 60) ≈ 0.011625 rad/s por tick
# Inverso: 1 rad/s ÷ 0.011625 ≈ 86.03 ticks
VEL_TICKS_PER_RAD_S = 86.03

# Segurança: tempo máximo sem receber comando antes de desligar o torque
WATCHDOG_TIMEOUT_S = 3.0


class AX12HardwareInterfaceV2(Node): #Nó Hardware Interface

    def __init__(self):
        super().__init__('ax12_hardware_interface_v2')

        # -----------------------------------------------------------------
        # Mapa junta → ID do motor
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

        # Timestamp do último comando recebido (para watchdog)
        self._last_cmd_time = self.get_clock().now()
        self._torque_ativo  = False

        # -----------------------------------------------------------------
        # Publisher de erros (criado antes da porta para logar falhas iniciais)
        # -----------------------------------------------------------------
        self.error_publisher = self.create_publisher(String, '/hardware_errors', 10)

        # -----------------------------------------------------------------
        # 1. Abre a porta serial
        # -----------------------------------------------------------------
        self.portHandler   = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # GroupSyncWrite com bloco de 4 bytes: [goal_pos_L, goal_pos_H, speed_L, speed_H]
        # Envia posição + velocidade em UM único pacote SYNC_WRITE — Ref [1]
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_SYNC_BLOCK
        )

        if not self.portHandler.openPort():
            self._publicar_erro(
                f"FALHA ao abrir porta '{DEVICENAME}'. "
                "Verifique o cabo USB/U2D2 e rode: ls /dev/tty* para confirmar o nome."
            )
            raise RuntimeError(f"Porta serial '{DEVICENAME}' não pôde ser aberta.")

        if not self.portHandler.setBaudRate(BAUDRATE):
            self._publicar_erro(
                f"FALHA ao configurar baudrate {BAUDRATE}. "
                "Confirme que os motores estão configurados com o mesmo baudrate."
            )
            raise RuntimeError(f"Baudrate {BAUDRATE} não pôde ser configurado.")

        self.get_logger().info(f"Porta '{DEVICENAME}' aberta @ {BAUDRATE} bps.")

        # -----------------------------------------------------------------
        # 2. Ping em cada motor para verificar presença — Ref [1], [4]
        # -----------------------------------------------------------------
        motores_ausentes = []
        for dxl_id in self.active_ids:
            model_number, comm_result, dxl_error = self.packetHandler.ping(
                self.portHandler, dxl_id
            )
            if comm_result != COMM_SUCCESS:
                aviso = (f"Motor ID {dxl_id} NÃO RESPONDEU ao ping. "
                         f"({self.packetHandler.getTxRxResult(comm_result)}) "
                         "Verifique alimentação, ID e fiação.")
                self.get_logger().warn(aviso)
                self._publicar_erro(aviso)
                motores_ausentes.append(dxl_id)
            else:
                self.get_logger().info(
                    f"Motor ID {dxl_id} detectado. Modelo: {model_number}."
                )

        # Remove motores ausentes do mapa para não travar na operação
        if motores_ausentes:
            self.joint_map = {
                k: v for k, v in self.joint_map.items()
                if v not in motores_ausentes
            }
            self.active_ids = list(self.joint_map.values())
            self.get_logger().warn(
                f"Prosseguindo sem os motores ausentes: {motores_ausentes}"
            )

        # -----------------------------------------------------------------
        # 3. Liga o torque com delay de proteção da fonte — Ref [2]
        # -----------------------------------------------------------------
        for dxl_id in self.active_ids:
            comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1
            )
            if comm_result != COMM_SUCCESS:
                msg = (f"[ID {dxl_id}] Falha ao ligar torque: "
                       f"{self.packetHandler.getTxRxResult(comm_result)}")
                self.get_logger().error(msg)
                self._publicar_erro(msg)
            elif dxl_error != 0:
                msg = (f"[ID {dxl_id}] Erro de hardware ao ligar torque: "
                       f"{self.packetHandler.getRxPacketError(dxl_error)} "
                       f"(cod={dxl_error:#04x})")
                self.get_logger().warn(msg)
                self._publicar_erro(msg)
            time.sleep(0.05)   # 50 ms por motor para estabilizar a fonte

        self._torque_ativo = True
        self.get_logger().info("Torque LIGADO. Aguardando comandos...")

        # -----------------------------------------------------------------
        # 4. QoS + Subscriber — Ref [2], padrão para actuators em rede Wi-Fi
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

        # -----------------------------------------------------------------
        # 5. Watchdog timer — Ref [2]
        #    Verifica a cada 1 s se o último comando foi há mais de
        #    WATCHDOG_TIMEOUT_S segundos. Se sim, desliga o torque.
        # -----------------------------------------------------------------
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info(
            f"Subscriber '/joint_trajectory' ativo. "
            f"Watchdog configurado para {WATCHDOG_TIMEOUT_S}s."
        )

    # =========================================================================
    # CALLBACK PRINCIPAL — Recebe JointTrajectory e envia para os motores
    # =========================================================================
    def _listener_callback(self, msg: JointTrajectory):
        # Atualiza watchdog
        self._last_cmd_time = self.get_clock().now()

        # Religa torque se o watchdog tiver desligado
        if not self._torque_ativo:
            self.get_logger().info("Comando recebido após watchdog. Religando torque...")
            self._set_torque(True)

        if not msg.points:
            return

        point = msg.points[0]
        self.groupSyncWrite.clearParam()
        motores_enviados = 0

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name not in self.joint_map:
                continue

            dxl_id = self.joint_map[joint_name]

            rads  = point.positions[i]  if i < len(point.positions)  else 0.0
            rad_s = point.velocities[i] if i < len(point.velocities) else 0.0

            # Conversão posição: radianos → ticks (0 a 1023)
            rads     = max(-RAD_LIMITE, min(RAD_LIMITE, rads))
            goal_pos = int((rads + RAD_LIMITE) * (TICKS_MAX / RAD_RANGE))
            goal_pos = max(0, min(TICKS_MAX, goal_pos))

            # Conversão velocidade: rad/s → ticks (1 a 1023)
            velocidade = int(abs(rad_s) * VEL_TICKS_PER_RAD_S)
            velocidade = max(1, min(TICKS_MAX, velocidade))

            # -----------------------------------------------------------------
            # Monta bloco de 4 bytes: [pos_L, pos_H, vel_L, vel_H]
            # Envia posição + velocidade juntos em UM pacote SYNC_WRITE — Ref [1]
            # Isso é mais eficiente que escrever velocidade individualmente e
            # depois fazer o SyncWrite de posição (como na V1).
            # -----------------------------------------------------------------
            param = [
                DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos),   # bytes 0-1: posição
                DXL_LOBYTE(velocidade), DXL_HIBYTE(velocidade) # bytes 2-3: velocidade
            ]

            if not self.groupSyncWrite.addParam(dxl_id, param):
                erro = f"[ID {dxl_id}] Falha ao enfileirar no GroupSyncWrite."
                self.get_logger().warn(erro)
                self._publicar_erro(erro)
                continue

            motores_enviados += 1

        # Dispara UM único pacote para todos os motores
        if motores_enviados > 0:
            comm_result = self.groupSyncWrite.txPacket()
            if comm_result != COMM_SUCCESS:
                erro = (f"Falha no SyncWrite ({motores_enviados} motores): "
                        f"{self.packetHandler.getTxRxResult(comm_result)}")
                self.get_logger().error(erro)
                self._publicar_erro(erro)

    # =========================================================================
    # WATCHDOG — Desliga torque se a conexão cair — Ref [2]
    # =========================================================================
    def _watchdog_callback(self):
        if not self._torque_ativo:
            return

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        if elapsed > WATCHDOG_TIMEOUT_S:
            aviso = (f"Watchdog: nenhum comando nos últimos {elapsed:.1f}s "
                     f"(limite: {WATCHDOG_TIMEOUT_S}s). Desligando torque por segurança.")
            self.get_logger().warn(aviso)
            self._publicar_erro(aviso)
            self._set_torque(False)

    # =========================================================================
    # UTILITÁRIOS
    # =========================================================================
    def _set_torque(self, estado: bool):
        """Liga (True) ou desliga (False) o torque em todos os motores ativos."""
        valor = 1 if estado else 0
        for dxl_id in self.active_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, valor
            )
        self._torque_ativo = estado
        status = "LIGADO" if estado else "DESLIGADO"
        self.get_logger().info(f"Torque {status} em {len(self.active_ids)} motores.")

    def _publicar_erro(self, mensagem: str):
        msg = String()
        msg.data = mensagem
        self.error_publisher.publish(msg)

    # =========================================================================
    # DESTRUIÇÃO SEGURA
    # =========================================================================
    def destroy_node(self):
        self.get_logger().info("Encerrando hardware interface...")
        if self._torque_ativo:
            self._set_torque(False)
        self.portHandler.closePort()
        super().destroy_node()
        self.get_logger().info("Hardware interface encerrado com segurança.")


# =============================================================================
# ENTRY POINT
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AX12HardwareInterfaceV2()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"\n[ERRO FATAL] {e}")
        print("Corrija o problema de hardware e reinicie o nó.\n")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
