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

        # Precisa ser exatamente igual ao do AX12Controller.py

        qos_profile = QoSProfile(

            reliability=ReliabilityPolicy.RELIABLE,

            history=HistoryPolicy.KEEP_LAST,

            depth=10

        )

        

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
    """Calcula a velocidade angular pura em Radianos por Segundo (rad/s)."""
    delta_rad = abs(rad_alvo - rad_anterior)
    return delta_rad / passo



def main():

    # Inicia a conexão com o controlador do robô
    robo = ConexaoRobo()


    # Matrizes de movimento (valores originais em radianos)
    traj_times   = [0.0, 0.2500, 0.5000, 0.7500, 1.0, 1.2500, 1.5000]
    ankle_motion = [-0.7418, 0.0873, 0.2182, -0.7418, 0.5236, 0.5236, -0.7418]
    knee_motion  = [1.3963, 1.0472, 0.4363, 0.1745, 0.5236, 0.7854, 1.3963]
    hip_motion   = [0.0, -0.7854, -0.6981, 0.4800, 0.4800, 0.3491, 0.0]



    num_points = len(traj_times)
    current_index = 0



    # Variáveis de tempo
    passo = 1.0  # Tempo de transição (em segundos)
    pausa = 0.5  # Tempo extra de repouso
    tempo_total_espera = passo + pausa



    # Armazena o histórico do passo anterior

    prev_ankle_rad = ankle_motion[0]
    prev_knee_rad  = knee_motion[0]
    prev_hip_rad   = hip_motion[0]



    # Nomes exatos das juntas (PRECISAM ser idênticos aos cadastrados no controlador)

    nomes_juntas = [

        'tornozelo_1', 'tornozelo_2', 

        'joelho_5', 'joelho_6', 

        'quadril_7', 'quadril_8'

    ]



    print("Conectado ao robô! Iniciando ciclo de marcha... (Pressione Ctrl+C para parar)")



    try:

        while True: # Loop infinito do Python



            # 1. Lê os valores do ponto atual

            ankle_rad = ankle_motion[current_index]

            knee_rad  = knee_motion[current_index]

            hip_rad   = hip_motion[current_index]



            # 2. Calcula as velocidades físicas em rad/s

            vel_ankle = calcular_velocidade_rad_s(ankle_rad, prev_ankle_rad, passo)

            vel_knee  = calcular_velocidade_rad_s(knee_rad, prev_knee_rad, passo)

            vel_hip   = calcular_velocidade_rad_s(hip_rad, prev_hip_rad, passo)



            posicoes_alvo = [ankle_rad, ankle_rad, knee_rad, knee_rad, hip_rad, hip_rad]

            velocidades_alvo = [vel_ankle, vel_ankle, vel_knee, vel_knee, vel_hip, vel_hip]



            # 3. Manda o comando para o robô usando a nossa API simplificada

            robo.enviar_passo(nomes_juntas, posicoes_alvo, velocidades_alvo)



            # Prints de acompanhamento

            print(f"Passo {current_index + 1}/{num_points} enviado! | Sleep: {tempo_total_espera}s")

            print(f"  -> Tornozelos: {ankle_rad:.2f} rad | Joelhos: {knee_rad:.2f} rad | Quadris: {hip_rad:.2f} rad\n")



            # 4. Atualiza as variáveis "anteriores"

            prev_ankle_rad = ankle_rad

            prev_knee_rad  = knee_rad

            prev_hip_rad   = hip_rad



            # 5. Avança o índice da matriz

            current_index += 1

            if current_index >= num_points:

                current_index = 0  



            # 6. Trava o loop

            time.sleep(tempo_total_espera)



    except KeyboardInterrupt:

        print("\nScript interrompido pelo usuário.")

    finally:

        # Desconecta de forma segura

        robo.fechar_conexao()



if __name__ == '__main__':

    main()