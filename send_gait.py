import rclpy
from std_msgs.msg import Int32MultiArray
import time
import math

def main():
    # Inicializa a comunicação do ROS 2
    rclpy.init()

    # Cria o nó de forma direta (sem usar classes)
    node = rclpy.create_node('simple_gait_publisher')
    publisher = node.create_publisher(Int32MultiArray, '/set_position', 10)

    # Matrizes extraídas da sua imagem (valores em radianos)
    traj_times   = [0.0, 0.2500, 0.5000, 0.7500, 1.0, 1.2500, 1.5000]
    ankle_motion = [-0.7418, 0.0873, 0.2182, -0.7418, 0.5236, 0.5236, -0.7418]
    knee_motion  = [1.3963, 1.0472, 0.4363, 0.1745, 0.5236, 0.7854, 1.3963]
    hip_motion   = [0.0, -0.7854, -0.6981, 0.4800, 0.4800, 0.3491, 0.0]

    num_points = len(traj_times)
    current_index = 0

    node.get_logger().info('Iniciando o envio do ciclo de marcha (Pressione Ctrl+C para parar)...')

    try:
        # Loop infinito enviando as posições
        while rclpy.ok():
            # 1. Lê os valores em radianos do ponto atual
            ankle_rad = ankle_motion[current_index]
            knee_rad  = knee_motion[current_index]
            hip_rad   = hip_motion[current_index]

            # 2. Converte de radianos para graus e transforma em inteiro
            ankle_deg = int(math.degrees(ankle_rad))
            knee_deg  = int(math.degrees(knee_rad))
            hip_deg   = int(math.degrees(hip_rad))

            # 3. Monta a mensagem [ID, Pos, ID, Pos...]
            msg = Int32MultiArray()
            msg.data = [
                1, ankle_deg,  # Tornozelo 1
                2, ankle_deg,  # Tornozelo 2
                5, knee_deg,   # Joelho 5
                6, knee_deg,   # Joelho 6
                7, hip_deg,    # Quadril 7
                8, hip_deg     # Quadril 8
            ]

            # 4. Publica no tópico do nó controlador
            publisher.publish(msg)
            print(f"Passo {current_index + 1}/{num_points} enviado! Posições(graus) -> Tornozelo: {ankle_deg}, Joelho: {knee_deg}, Quadril: {hip_deg}")

            # 5. Avança para o próximo ponto da matriz
            current_index += 1
            if current_index >= num_points:
                current_index = 0  # Reinicia o ciclo para continuar andando

            # 6. Espera 0.25 segundos (diferença de tempo entre os pontos da sua matriz)
            time.sleep(0.25)

    except KeyboardInterrupt:
        node.get_logger().info('Script interrompido.')
    finally:
        # Limpa tudo ao fechar
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()