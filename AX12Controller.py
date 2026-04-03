import rclpy
from rclpy.node import Node
import math

# Importando as mensagens padrão do ROS 2
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

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
        # Mapeia os Nomes que o ROS usa para os IDs físicos dos motores.
        # Adapte os nomes abaixo para bater exatamente com os nomes enviados pela sua marcha!
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

        # 3. Cria o Subscriber (Agora usa JointTrajectory no tópico /joint_trajectory)
        self.subscription = self.create_subscription(
            JointTrajectory, '/joint_trajectory', self.listener_callback, 10)

        # 4. Cria o Publisher (Agora usa JointState no tópico /joint_states)
        self.publisher_ = self.create_publisher(
            JointState, '/joint_states', 10)

        # 5. Timer de leitura a 10 Hz
        self.timer = self.create_timer(0.1, self.read_motors_callback)


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

            # Pega as posições e velocidades (se houver velocidade)
            rads = point.positions[i] if i < len(point.positions) else 0.0
            rad_s = point.velocities[i] if i < len(point.velocities) else 0.0

            # --- CONVERSÃO DE POSIÇÃO (Radianos -> 0 a 1023) ---
            graus = math.degrees(rads)
            graus = max(-150.0, min(150.0, graus)) # Trava de segurança física
            goal_pos = int((graus + 150.0) * (1023.0 / 300.0))
            goal_pos = max(0, min(1023, goal_pos))

            # --- CONVERSÃO DE VELOCIDADE (Rad/s -> 1 a 1023) ---
            # 1 unidade DXL = 0.666 graus/s
            vel_deg_s = abs(math.degrees(rad_s))
            velocidade = int(vel_deg_s / 0.666)
            
            # Garante que envie pelo menos 1 (evita enviar 0, que no AX-12 significa velocidade máxima sem controle)
            velocidade = max(1, min(1023, velocidade))

            # Escreve a velocidade individual e empacota a posição no SyncWrite
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MOVING_SPEED, velocidade)
            param_goal_position = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]
            self.groupSyncWrite.addParam(dxl_id, param_goal_position)

        # Envia todas as posições ao mesmo tempo
        self.groupSyncWrite.txPacket()


    # --- FUNÇÃO DE LEITURA (Lê 0 a 1023 e publica em Radianos) ---
    def read_motors_callback(self):
        msg = JointState()
        
        # Opcional, mas recomendado no ROS: adicionar o timestamp da leitura
        msg.header.stamp = self.get_clock().now().to_msg()

        # Lê a posição de cada motor mapeado
        for joint_name, dxl_id in self.joint_map.items():
            dxl_present_pos, dxl_comm_result, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result == COMM_SUCCESS:
                # 1. Converte de bruto (0-1023) para graus (-150 a 150)
                graus_reais = (dxl_present_pos * 300.0 / 1023.0) - 150.0
                
                # 2. Converte de graus para radianos (Padrão ROS)
                rads_reais = math.radians(graus_reais)
                
                # Adiciona na mensagem
                msg.name.append(joint_name)
                msg.position.append(rads_reais)

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