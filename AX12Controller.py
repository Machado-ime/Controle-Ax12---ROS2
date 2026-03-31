# Importando Ferramentas
import rclpy                   # Biblioteca principal do ROS 2 para Python
from rclpy.node import Node    # Classe base para criar um nó
from std_msgs.msg import Int32MultiArray # Tipo de msg para arrays/matrizes de inteiros
from dynamixel_sdk import * # Biblioteca para controlar os motores Dynamixel (AX-12)

# --- Configurações do Motor ---
# Lista com os IDs de todos os motores conectados que o nó deve gerenciar o torque
ACTIVE_DXL_IDS          = list(range(1, 19)) # ADICIONE AQUI OS IDs DOS SEUS MOTORES
BROADCAST_ID            = 254                # ID 254 (0xFE) fala com TODOS os motores ao mesmo tempo no Protocolo 1.0
BAUDRATE                = 1000000            # Baudrate padrão (1 Mbps)
DEVICENAME              = '/dev/ttyACM0'     # Porta da OpenCR
PROTOCOL_VERSION        = 1.0

# Endereços de controle do AX-12
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30

class AX12Controller(Node):

    def __init__(self):
        super().__init__('ax12_controller_node')              # Nomear o Nó

        # Inicializa a comunicação Serial
        self.portHandler = PortHandler(DEVICENAME)           # Configuração da porta serial
        self.packetHandler = PacketHandler(PROTOCOL_VERSION) # Traduzir nossos números para a linguagem de bytes do Protocolo 1.0.
        
        # Tenta abrir a porta e configurar o baudrate
        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Porta aberta e Baudrate configurado com sucesso!')
        else:
            self.get_logger().error('Falha ao abrir a porta! Verifique o cabo e o chmod.')
            return

        # Liga o Torque de TODOS os motores listados
        for dxl_id in ACTIVE_DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info(f'Torque LIGADO para os motores {ACTIVE_DXL_IDS}. Aguardando comandos...')

        # Cria o Subscriber usando Int32MultiArray
        self.subscription = self.create_subscription(
            Int32MultiArray, 
            '/set_position', 
            self.listener_callback, 
            10
        )

    def listener_callback(self, msg):
        # msg.data recebe o array achatado. Assumimos o formato: [ID1, Pos1, ID2, Pos2, ...]
        data = msg.data

        # Verifica se o array tem um número par de elementos (garantindo que não falte a posição de um ID)
        if len(data) % 2 != 0:
            self.get_logger().error('Recebido array de tamanho ímpar! Envie pares de [ID, Posição].')
            return

        # Loop pulando de 2 em 2 para pegar os pares (colunas da sua matriz)
        for i in range(0, len(data), 2):
            dxl_id = data[i]
            graus = data[i+1]

            # Limita os valores entre -150 e 150 graus
            graus = max(-150, min(150, graus)) 

            # Converte de graus (-150 a 150) para a escala do Dynamixel (0 a 1023)
            goal_pos = int((graus + 150) * (1023.0 / 300.0))
            goal_pos = max(0, min(1023, goal_pos)) # Limite de segurança

            self.get_logger().info(f'Preparando Motor {dxl_id} para {graus} graus (Passo {goal_pos})')
            
            # --- REG_WRITE: Prepara o comando na memória do motor, mas não executa ainda ---
            self.packetHandler.regWrite2ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, goal_pos)

        # --- ACTION: Quando a matriz acabar, envia o gatilho para todos os motores executarem juntos ---
        self.packetHandler.action(self.portHandler, BROADCAST_ID)
        self.get_logger().info('Comando ACTION (Broadcast) disparado! Motores movendo sincronizadamente.')

    def destroy_node(self):
        # Desliga o torque de todos os motores da lista ao encerrar
        for dxl_id in ACTIVE_DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            
        self.portHandler.closePort()
        self.get_logger().info('Torque desligado e porta fechada.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AX12Controller()

    try:
        rclpy.spin(node)      # Mantém o nó rodando infinitamente
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()