# Importando Ferramentas
import rclpy                   # Biblioteca principal do ROS 2 para Python
from rclpy.node import Node    # Classe base para criar um nó
from std_msgs.msg import Int32 # Tipo de mensagem para números inteiros
from dynamixel_sdk import *    # Biblioteca para controlar os motores Dynamixel (AX-12)



# --- Configurações do Motor ---
MY_DXL_ID               = 1              # ID do seu motor
BAUDRATE                = 1000000        # Baudrate padrão (1 Mbps)
DEVICENAME              = '/dev/ttyACM0' # Porta da OpenCR
PROTOCOL_VERSION        = 1.0

# Endereços de controle do AX-12
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30



class AX12Controller(Node):

    def __init__(self):

        super().__init__('ax12_controller_node')              # Nomear o Nó

        # Inicializa a comunicação Serial
        self.portHandler = PortHandler(DEVICENAME)           # Configuração da porta serial
        self.packetHandler = PacketHandler(PROTOCOL_VERSION) # traduzir nossos números para a linguagem de bytes do Protocolo 1.0.
        

        # Tenta abrir a porta e configurar o baudrate
        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Porta aberta e Baudrate configurado com sucesso!')
        else:
            self.get_logger().error('Falha ao abrir a porta! Verifique o cabo e o chmod.')
            return

        # Liga o Torque do motor
        self.packetHandler.write1ByteTxRx(self.portHandler, MY_DXL_ID, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info('Torque do motor LIGADO. Aguardando comandos...')



        # Cria o Subscriber (Tópico): escutar números inteiros
        self.subscription = self.create_subscription(Int32, '/set_position', self.listener_callback, 10)

    def listener_callback(self, msg):

        graus = msg.data
        graus = max(-150, min(150, graus))
        goal_pos = int((graus + 150) * (1023.0 / 300.0))
        goal_pos = max(0, min(1023, goal_pos)) # Limita os valores entre 0 e 1023

        self.get_logger().info(f'Comando recebido: Movendo para {goal_pos}')
        self.packetHandler.write2ByteTxRx(self.portHandler, MY_DXL_ID, ADDR_GOAL_POSITION, goal_pos)

    def destroy_node(self):

        # Desliga o torque e fecha a porta ao encerrar o nó (Ctrl+C)
        self.packetHandler.write1ByteTxRx(self.portHandler, MY_DXL_ID, ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()
        self.get_logger().info('Torque desligado e porta fechada.')
        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)
    node = AX12Controller()

    try:
        rclpy.spin(node) # Mantém o nó rodando infinitamente
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()