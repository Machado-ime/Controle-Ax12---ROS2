import rclpy
from rclpy.node import Node
import math
import time

# Importando a mensagem padrão do ROS 2 de trajetória e a biblioteca de QoS
from trajectory_msgs.msg import JointTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dynamixel_sdk import * 

# --- Configurações do Motor ---
BAUDRATE                = 1000000        
DEVICENAME              = '/dev/ttyACM0' 
PROTOCOL_VERSION        = 1.0

# Endereços de controle do AX-12
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
ADDR_MOVING_SPEED       = 32
LEN_GOAL_POSITION       = 2  

class AX12HardwareInterface(Node):

    def __init__(self):
        super().__init__('ax12_hardware_interface')

        # Mapa das juntas
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

        # 1. Inicializa a porta serial
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Porta aberta com sucesso! Hardware Interface (Somente Escrita) iniciado.')
        else:
            self.get_logger().error('Falha ao abrir a porta!')
            return

        # 2. Liga o Torque apenas dos motores mapeados (Com proteção de corrente)
        for dxl_id in self.active_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            time.sleep(0.05) # <-- VITAL: Dá 50ms para a fonte estabilizar antes de ligar o próximo motor
            
        self.get_logger().info('Torque LIGADO. Pronto para receber e escrever comandos.')

        # --- PERFIL DE REDE (QoS) BLINDADO PARA WI-FI ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 3. Cria o topic
        self.subscription = self.create_subscription(
            JointTrajectory, 
            '/joint_trajectory', 
            self.listener_callback, 
            qos_profile
            )

    # --- FUNÇÃO DE ESCRITA (Recebe Trajetória em Radianos e Rad/s) ---
    def listener_callback(self, msg):
        # Verifica se há pontos na trajetória
        if not msg.points:
            return 

        # Para um controle imediato, pegamos o primeiro ponto (índice 0)
        point = msg.points[0]

        self.groupSyncWrite.clearParam()

        # Itera sobre os nomes das juntas que vieram na mensagem
        for i, joint_name in enumerate(msg.joint_names):
            
            # Se o nome não estiver no dicionário, ignora
            if joint_name not in self.joint_map:
                continue
                
            dxl_id = self.joint_map[joint_name]

            # Pega as posições e velocidades 
            rads = point.positions[i] if i < len(point.positions) else 0.0
            rad_s = point.velocities[i] if i < len(point.velocities) else 0.0

            # --- CONVERSÃO DE POSIÇÃO (Radianos -> 0 a 1023) ---
            limite_rad = 2.618
            rads = max(-limite_rad, min(limite_rad, rads)) 
            goal_pos = int((rads + limite_rad) * (1023.0 / 5.236))
            goal_pos = max(0, min(1023, goal_pos))
            velocidade = int(abs(rad_s) * 86.03)
            velocidade = max(1, min(1023, velocidade))

            # Escreve a velocidade individual e empacota a posição no SyncWrite
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MOVING_SPEED, velocidade)
            param_goal_position = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]
            self.groupSyncWrite.addParam(dxl_id, param_goal_position)

        # Envia todas as posições ao mesmo tempo
        self.groupSyncWrite.txPacket()


    def destroy_node(self):
        for dxl_id in self.active_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AX12HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()