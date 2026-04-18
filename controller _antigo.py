import rclpy
from rclpy.node import Node
import math

# Importando as mensagens padrão do ROS 2 e a biblioteca de QoS
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String # <-- NOVO: Import para a mensagem de erro
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from dynamixel_sdk import * # --- Configurações do Motor ---
BAUDRATE                = 1000000        
DEVICENAME              = '/dev/ttyACM0' 
PROTOCOL_VERSION        = 1.0

# Endereços de controle do AX-12
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
ADDR_MOVING_SPEED       = 32
ADDR_PRESENT_POSITION   = 36 
LEN_GOAL_POSITION       = 2  

class AX12HardwareInterface(Node):

    def __init__(self):
        super().__init__('ax12_hardware_interface')

        # --- O DICIONÁRIO MÁGICO DO ROS ---
        self.joint_map = {
            'tornozelo_1': 1,
            'tornozelo_2': 2,
            'joelho_5': 5,
            'joelho_6': 6,
            'quadril_7': 7,
            'quadril_8': 8
        }
        
        self.active_ids = list(self.joint_map.values())

        # 1. Inicializa a porta serial
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Porta aberta com sucesso! Hardware Interface Padrão ROS iniciado.')
        else:
            self.get_logger().error('Falha ao abrir a porta!')
            return

        # 2. Liga o Torque apenas dos motores mapeados
        for dxl_id in self.active_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info('Torque LIGADO. Pronto para ler e escrever.')

        # --- PERFIL DE REDE (QoS) BLINDADO PARA WI-FI ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 3. Cria o Subscriber (Escuta as posições com QoS Confiável)
        self.subscription = self.create_subscription(
            JointTrajectory, '/joint_trajectory', self.listener_callback, qos_profile)

        # 4. Cria o Publisher (Publica o status dos motores também com QoS Confiável)
        self.publisher_ = self.create_publisher(
            JointState, '/joint_states', qos_profile)

        # --- NOVO: Cria o Publisher para avisos de erro de hardware ---
        self.error_publisher = self.create_publisher(
            String, '/hardware_errors', 10)

        # 5. Timer de leitura a 10 Hz
        self.timer = self.create_timer(0.1, self.read_motors_callback)


    # --- FUNÇÃO DE ESCRITA (Recebe Trajetória em Radianos e Rad/s) ---
    def listener_callback(self, msg):
        if not msg.points:
            return 

        point = msg.points[0]
        self.groupSyncWrite.clearParam()

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name not in self.joint_map:
                continue
                
            dxl_id = self.joint_map[joint_name]

            rads = point.positions[i] if i < len(point.positions) else 0.0
            rad_s = point.velocities[i] if i < len(point.velocities) else 0.0

            # --- CONVERSÃO DE POSIÇÃO (Radianos -> 0 a 1023) ---
            graus = math.degrees(rads)
            graus = max(-150.0, min(150.0, graus)) 
            goal_pos = int((graus + 150.0) * (1023.0 / 300.0))
            goal_pos = max(0, min(1023, goal_pos))

            # --- CONVERSÃO DE VELOCIDADE (Rad/s -> 1 a 1023) ---
            vel_deg_s = abs(math.degrees(rad_s))
            velocidade = int(vel_deg_s / 0.666)
            velocidade = max(1, min(1023, velocidade))

            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MOVING_SPEED, velocidade)
            param_goal_position = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]
            self.groupSyncWrite.addParam(dxl_id, param_goal_position)

        self.groupSyncWrite.txPacket()

    # --- FUNÇÃO DE LEITURA (Lê 0 a 1023 e publica em Radianos) ---
    def read_motors_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for joint_name, dxl_id in self.joint_map.items():
            try:
                dxl_present_pos, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
                
                if dxl_comm_result == COMM_SUCCESS:
                    # --- NOVO BLOCO: Tratamento e Publicação de Erros ---
                    if dxl_error != 0:
                        # Pede ao SDK para traduzir o código do erro (ex: Overload Error)
                        erro_traduzido = self.packetHandler.getRxPacketError(dxl_error)
                        
                        # Monta a string de alerta
                        alerta_txt = f"[{joint_name} - ID {dxl_id}] ERRO DE HARDWARE: {erro_traduzido} (Cod: {dxl_error})"
                        
                        # Mostra no terminal para você ver
                        self.get_logger().warn(alerta_txt)
                        
                        # Publica no tópico '/hardware_errors'
                        msg_erro = String()
                        msg_erro.data = alerta_txt
                        self.error_publisher.publish(msg_erro)

                    # 1. Converte de bruto (0-1023) para graus (-150 a 150)
                    graus_reais = (dxl_present_pos * 300.0 / 1023.0) - 150.0
                    
                    # 2. Converte de graus para radianos (Padrão ROS)
                    rads_reais = math.radians(graus_reais)
                    
                    # Adiciona na mensagem
                    msg.name.append(joint_name)
                    msg.position.append(rads_reais)
                    
            except IndexError:
                self.get_logger().warn(f"Falha de comunicação no ID {dxl_id} (Pacote vazio/corrompido).")
                continue

        # Publica no tópico '/joint_states'
        if msg.name:
            self.publisher_.publish(msg)

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