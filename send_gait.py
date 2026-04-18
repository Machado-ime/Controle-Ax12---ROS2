import rclpy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# =====================================================================
# 1. A PONTE DE CONEXÃO (Esconde o Nó ROS nos bastidores)
# =====================================================================
class ConexaoRobo:
    """Cria um nó invisível apenas para enviar comandos ao controlador AX-12."""
    def __init__(self):
        rclpy.init()
        self._node = rclpy.create_node('gait_client_hidden')

        # --- PERFIL DE REDE (QoS) BLINDADO PARA WI-FI ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Fila de tamanho 1 é crucial aqui!
        )

        # LINHA CORRIGIDA: Estava cortada no final
        self._publisher = self._node.create_publisher(JointTrajectory, '/joint_trajectory', qos_profile)

    def enviar_passo(self, nomes_juntas, posicoes_rad, velocidades_rad_s):
        """Monta a mensagem padrão do ROS e envia para o robô."""
        msg = JointTrajectory()
        msg.joint_names = nomes_juntas

        point = JointTrajectoryPoint()
        point.positions = posicoes_rad
        point.velocities = velocidades_rad_s

        msg.points.append(point)
        self._publisher.publish(msg)

        # Garante que o ROS processe o envio rapidamente nos bastidores
        rclpy.spin_once(self._node, timeout_sec=0.01)

    def fechar_conexao(self):
        """Desliga o nó oculto de forma segura."""
        self._node.destroy_node()
        rclpy.shutdown()


# =====================================================================
# 2. O SEU CÓDIGO PURO (Lógica Matemática e Controle)
# =====================================================================
def calcular_velocidade_rad_s(rad_alvo, rad_anterior, passo):
    """Calcula a velocidade angular pura em Rad/s."""
    delta_rad = abs(rad_alvo - rad_anterior)
    return delta_rad / passo

def main():
    robo = ConexaoRobo()

    traj_times = [0.0, 0.2500, 0.5000, 0.7500, 1.0, 1.2500, 1.5000]
    num_points = len(traj_times)

    # =================================================================
    # A MATRIZ DE MOVIMENTO (18 LINHAS x 7 COLUNAS)
    # Linha 0 = Motor 1 ... Linha 17 = Motor 18
    # =================================================================
    matriz_movimento = [
        [-0.7418,  0.0873,  0.2182, -0.7418,  0.5236,  0.5236, -0.7418], 
        [-0.7418,  0.0873,  0.2182, -0.7418,  0.5236,  0.5236, -0.7418], 
       #[ 0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0   ], 
       #[ 0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0   ], 
        [ 1.3963,  1.0472,  0.4363,  0.1745,  0.5236,  0.7854,  1.3963], 
        [ 1.3963,  1.0472,  0.4363,  0.1745,  0.5236,  0.7854,  1.3963], 
        [ 0.0,    -0.7854, -0.6981,  0.4800,  0.4800,  0.3491,  0.0   ], 
        [ 0.0,    -0.7854, -0.6981,  0.4800,  0.4800,  0.3491,  0.0   ], 

    ]


    # Variáveis de tempo
    passo = 1.0  
    pausa = 0.5  
    tempo_total_espera = passo + pausa

    # A ordem exata (Índice 0 = ID 1 ... Índice 17 = ID 18)
    nomes_juntas = [
        'PD_tornozelo_pitch_1', 
        'PE_tornozelo_pitch_2', 
        #'PD_tornozelo_roll_3',  
        #'PE_tornozelo_roll_4',  
        'PD_joelho_pitch_5',    
        'PE_joelho_pitch_6',    
        'PD_quadril_pitch_7',   
        'PE_quadril_pitch_8',   

    ]

    # Pega o primeiro valor de cada linha da matriz para iniciar o histórico
    posicoes_anteriores = [linha[0] for linha in matriz_movimento]

    current_index = 0
    print("Conectado! Iniciando...")

    try:
        while True:
            posicoes_alvo = []
            velocidades_alvo = []

            # Lê linha por linha (i vai de 0 a 17)
            for i in range(8):
                # 1. Lê a célula exata: linha do motor 'i', coluna do passo 'current_index'
                rad_alvo = matriz_movimento[i][current_index]

                # 2. Calcula a velocidade baseada no passo anterior (LINHA CORRIGIDA)
                vel_rad = calcular_velocidade_rad_s(rad_alvo, posicoes_anteriores[i], passo)

                # 3. Guarda nas listas de envio
                posicoes_alvo.append(rad_alvo)
                velocidades_alvo.append(vel_rad)

                # 4. Atualiza o histórico
                posicoes_anteriores[i] = rad_alvo

            # 5. Manda para a Raspberry Pi
            robo.enviar_passo(nomes_juntas, posicoes_alvo, velocidades_alvo)

            # LINHA CORRIGIDA: Estava cortada no final
            print(f"Passo {current_index + 1}/{num_points} enviado | Sleep: {tempo_total_espera}s")

            # 6. Avança a coluna da matriz
            current_index += 1
            if current_index >= num_points:
                current_index = 0  

            # 7. Trava o loop
            time.sleep(tempo_total_espera)

    except KeyboardInterrupt:
        print("\nScript interrompido pelo usuário.")
    finally:
        robo.fechar_conexao()

if __name__ == '__main__':
    main()