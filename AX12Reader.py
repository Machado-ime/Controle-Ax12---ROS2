# Importando Ferramentas
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # A mensagem padrão do ROS 2 para leitura de motores
from dynamixel_sdk import *

# --- Configurações do Motor ---
ACTIVE_DXL_IDS          = list(range(1, 19)) 

BAUDRATE                = 1000000        
DEVICENAME              = '/dev/ttyACM0' 
PROTOCOL_VERSION        = 1.0

# Endereços de LEITURA do AX-12
ADDR_PRESENT_POSITION   = 36 # Tamanho: 2 bytes
ADDR_PRESENT_LOAD       = 40 # Tamanho: 2 bytes (Mede a carga/torque atual)

class AX12Reader(Node):

    def __init__(self):
        super().__init__('ax12_reader_node')

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Porta aberta com sucesso para leitura!')
        else:
            self.get_logger().error('Falha ao abrir a porta!')
            return

        # Cria o Publisher usando a mensagem padrão JointState
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Cria um Timer que vai executar a função de leitura a 10Hz (a cada 0.1 segundos)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Inicializa a mensagem padrão
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg() # Marca o tempo exato da leitura
        
        nomes_motores = []
        posicoes_atuais = []
        torques_atuais = []

        # Faz um loop varrendo os 18 motores
        for dxl_id in ACTIVE_DXL_IDS:
            # Lê a Posição Atual
            pos_raw, comm_pos, err_pos = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            
            # Lê o Torque (Carga) Atual
            load_raw, comm_load, err_load = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_LOAD)

            # Converte a posição bruta (0-1023) de volta para a sua lógica de graus (-150 a +150)
            graus = (pos_raw * 300.0 / 1023.0) - 150.0

            # Adiciona os dados nas listas
            nomes_motores.append(f'motor_{dxl_id}')
            posicoes_atuais.append(float(graus))
            torques_atuais.append(float(load_raw))

        # Preenche a mensagem com os vetores criados
        msg.name = nomes_motores
        msg.position = posicoes_atuais
        msg.effort = torques_atuais # No ROS, torque/força é chamado de "effort"

        # Publica o tópico
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.portHandler.closePort()
        self.get_logger().info('Porta de leitura fechada.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AX12Reader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()