# Importando Ferramentas
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import * 

# --- Configurações do Motor ---
ACTIVE_DXL_IDS          = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18] 
BAUDRATE                = 1000000        
DEVICENAME              = '/dev/ttyACM0' 
PROTOCOL_VERSION        = 1.0

# Endereços de controle do AX-12
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
ADDR_MOVING_SPEED       = 32 # NOVO: Endereço para controlar a velocidade
LEN_GOAL_POSITION       = 2  # Tamanho em bytes da posição (0 a 1023 usa 2 bytes)

class AX12Controller(Node):

    def __init__(self):
        super().__init__('ax12_controller_node')

        # Inicializa a comunicação Serial
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Inicializa o GroupSyncWrite (Configurado para o endereço de Posição Alvo)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Porta aberta e Baudrate configurado com sucesso!')
        else:
            self.get_logger().error('Falha ao abrir a porta!')
            return

        # Liga o Torque de TODOS os motores
        for dxl_id in ACTIVE_DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info(f'Torque LIGADO para os motores {ACTIVE_DXL_IDS}. Aguardando comandos...')

        # Cria o Subscriber
        self.subscription = self.create_subscription(Int32MultiArray, '/set_position', self.listener_callback, 10)

    def listener_callback(self, msg):
        data = msg.data

        # Agora verifica se é múltiplo de 3 (ID, Posição, Velocidade)
        if len(data) % 3 != 0:
            self.get_logger().error('Recebido array inválido! Envie trios de [ID, Posição, Velocidade].')
            return

        # Limpa os parâmetros do SyncWrite da iteração anterior
        self.groupSyncWrite.clearParam()

        # Monta o pacote pulando de 3 em 3
        for i in range(0, len(data), 3):
            dxl_id = data[i]
            graus = data[i+1]
            velocidade = data[i+2]

            # --- Tratamento da Posição ---
            graus = max(-150, min(150, graus)) 
            goal_pos = int((graus + 150) * (1023.0 / 300.0))
            goal_pos = max(0, min(1023, goal_pos)) 

            # --- Tratamento da Velocidade ---
            # 0 = Máxima, 1 a 1023 = Proporcional
            velocidade = max(0, min(1023, velocidade))

            # 1. Escreve a velocidade no motor imediatamente
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MOVING_SPEED, velocidade)

            # 2. A biblioteca Dynamixel exige que os 2 bytes sejam separados em Low e High para o SyncWrite
            param_goal_position = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]

            # 3. Adiciona o motor e sua posição alvo no "caminhão" de dados do SyncWrite
            dxl_addparam_result = self.groupSyncWrite.addParam(dxl_id, param_goal_position)
            if not dxl_addparam_result:
                self.get_logger().error(f'[ID:{dxl_id}] falhou ao adicionar no SyncWrite')

        # --- Envia o pacote SyncWrite de posições para todos os motores de uma vez só ---
        dxl_comm_result = self.groupSyncWrite.txPacket()
        
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Erro de comunicação: {self.packetHandler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('SyncWrite executado! Velocidades ajustadas e Motores movidos em sincronia.')

    def destroy_node(self):
        for dxl_id in ACTIVE_DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            
        self.portHandler.closePort()
        self.get_logger().info('Torque desligado e porta fechada.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AX12Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()