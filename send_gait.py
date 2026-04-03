import rclpy
from std_msgs.msg import Int32MultiArray
import time
import math

def calcular_velocidade_dxl(deg_alvo, deg_anterior, passo):
    """
    Calcula a velocidade (Delta Ângulo / Tempo) e converte para a escala do Dynamixel AX-12.
    """
    delta_angulo = abs(deg_alvo - deg_anterior)
    vel_deg_s = delta_angulo / passo
    vel_dxl = int(vel_deg_s / 0.666) # Converte para unidades do Dynamixel (1 unidade = ~0.666 graus/s)
    
    if vel_dxl == 0:
        vel_dxl = 1
        
    return min(1023, vel_dxl) # Garante que não ultrapasse o limite do motor


def main():
    rclpy.init()

    node = rclpy.create_node('simple_gait_publisher')
    publisher = node.create_publisher(Int32MultiArray, '/set_position', 10)

    # Matrizes extraídas da imagem (valores em radianos)
    traj_times   = [0.0, 0.2500, 0.5000, 0.7500, 1.0, 1.2500, 1.5000]
    ankle_motion = [-0.7418, 0.0873, 0.2182, -0.7418, 0.5236, 0.5236, -0.7418]
    knee_motion  = [1.3963, 1.0472, 0.4363, 0.1745, 0.5236, 0.7854, 1.3963]
    hip_motion   = [0.0, -0.7854, -0.6981, 0.4800, 0.4800, 0.3491, 0.0]

    num_points = len(traj_times)
    current_index = 0

    # --- NOVAS VARIÁVEIS DE TEMPO ---
    passo = 0.25  # Tempo de transição (em segundos) que o motor tem para chegar no alvo
    pausa = 0.05  # Tempo extra de repouso na posição antes de iniciar o próximo passo
    tempo_total_espera = passo + pausa

    # Variáveis para armazenar o ângulo do passo anterior (inicializadas com o primeiro ponto da matriz)
    prev_ankle_deg = int(math.degrees(ankle_motion[0]))
    prev_knee_deg  = int(math.degrees(knee_motion[0]))
    prev_hip_deg   = int(math.degrees(hip_motion[0]))

    node.get_logger().info('Iniciando o envio do ciclo de marcha (Pressione Ctrl+C para parar)...')

    try:
        while rclpy.ok():
            # 1. Lê os valores em radianos do ponto atual
            ankle_rad = ankle_motion[current_index]
            knee_rad  = knee_motion[current_index]
            hip_rad   = hip_motion[current_index]

            # 2. Converte de radianos para graus e transforma em inteiro (ÂNGULOS ALVO)
            ankle_deg = int(math.degrees(ankle_rad))
            knee_deg  = int(math.degrees(knee_rad))
            hip_deg   = int(math.degrees(hip_rad))

            # 3. Calcula as velocidades físicas necessárias para os motores baseadas no `passo`
            vel_ankle = calcular_velocidade_dxl(ankle_deg, prev_ankle_deg, passo)
            vel_knee  = calcular_velocidade_dxl(knee_deg, prev_knee_deg, passo)
            vel_hip   = calcular_velocidade_dxl(hip_deg, prev_hip_deg, passo)

            # 4. Monta a mensagem para o NOVO controlador: [ID, Pos, Vel, ID, Pos, Vel...]
            msg = Int32MultiArray()
            msg.data = [
                1, ankle_deg, vel_ankle,  # Tornozelo 1
                2, ankle_deg, vel_ankle,  # Tornozelo 2
                5, knee_deg, vel_knee,    # Joelho 5
                6, knee_deg, vel_knee,    # Joelho 6
                7, hip_deg, vel_hip,      # Quadril 7
                8, hip_deg, vel_hip       # Quadril 8
            ]

            # 5. Publica no tópico
            publisher.publish(msg)
            print(f"Passo {current_index + 1}/{num_points} | Sleep: {tempo_total_espera}s")
            print(f"  -> Tornozelos: {ankle_deg}° (Vel: {vel_ankle}) | Joelhos: {knee_deg}° (Vel: {vel_knee}) | Quadris: {hip_deg}° (Vel: {vel_hip})\n")

            # 6. Atualiza as variáveis de "anterior" para a próxima rodada do loop
            prev_ankle_deg = ankle_deg
            prev_knee_deg  = knee_deg
            prev_hip_deg   = hip_deg

            # 7. Avança o índice da matriz
            current_index += 1
            if current_index >= num_points:
                current_index = 0  # Reinicia o ciclo

            # 8. Trava o loop pelo tempo de transição + a pausa solicitada
            time.sleep(tempo_total_espera)

    except KeyboardInterrupt:
        node.get_logger().info('Script interrompido.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()