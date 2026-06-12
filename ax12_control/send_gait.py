"""Cliente de marcha para o robô bípede AX-12.

Roda no PC de comando. Publica a sequência de passos (matriz de
movimento) em /joint_trajectory e escuta /hardware_errors para saber
quando o ax12_controller (na Raspberry Pi) está com problemas.

Antes de começar a marcha, espera o controlador aparecer na rede —
sem isso, os primeiros comandos se perdem durante a descoberta do DDS.
"""

import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# =====================================================================
# 1. A PONTE DE CONEXÃO (esconde o nó ROS nos bastidores)
# =====================================================================
class ConexaoRobo:
    """Publica comandos para o ax12_controller e escuta os erros dele."""

    def __init__(self):
        rclpy.init()
        self._node = rclpy.create_node('gait_client')

        # --- QoS dos comandos: PRECISA ser idêntico ao do ax12_controller ---
        # BEST_EFFORT + fila de 1: comando perdido no Wi-Fi é descartado,
        # nunca reenviado atrasado. QoS diferente = os nós nem se conectam!
        qos_comandos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._publisher = self._node.create_publisher(
            JointTrajectory, '/joint_trajectory', qos_comandos)

        # --- Escuta os erros de hardware do controlador ---
        # QoS padrão (RELIABLE), igual ao publisher de erros do controlador:
        # aviso de erro é raro e precisa chegar.
        self.falha_fatal = False
        self._node.create_subscription(
            String, '/hardware_errors', self._erro_callback, 10)

    def _erro_callback(self, msg):
        """Chamado automaticamente quando o controlador publica um erro."""
        print(f'\n[ALERTA DO ROBO] {msg.data}\n')
        # Se o controlador desistiu do hardware, paramos a marcha também:
        # continuar mandando passos para um robô meio-morto derruba ele.
        if 'FALHA FATAL' in msg.data:
            self.falha_fatal = True

    def aguardar_controlador(self):
        """Bloqueia até o ax12_controller aparecer na rede (descoberta DDS).

        get_subscription_count() conta quantos assinantes COMPATÍVEIS o
        publisher encontrou. Enquanto for 0, mandar comando é jogar fora.
        """
        if self._publisher.get_subscription_count() == 0:
            print('Aguardando o ax12_controller aparecer na rede... (Ctrl+C desiste)')
        while self._publisher.get_subscription_count() == 0:
            rclpy.spin_once(self._node, timeout_sec=0.5)
        print('Controlador encontrado!')

    def enviar_passo(self, nomes_juntas, posicoes_rad, velocidades_rad_s, duracao_s):
        """Monta a mensagem padrão do ROS e envia para o robô."""
        msg = JointTrajectory()
        msg.joint_names = nomes_juntas

        point = JointTrajectoryPoint()
        point.positions = posicoes_rad
        point.velocities = velocidades_rad_s
        # time_from_start documenta a duração pretendida do movimento.
        # O controlador atual ignora (usa as velocidades), mas preencher
        # mantém a mensagem completa para ferramentas futuras (RViz, MoveIt).
        point.time_from_start.sec = int(duracao_s)
        point.time_from_start.nanosec = int((duracao_s % 1.0) * 1e9)

        msg.points.append(point)
        self._publisher.publish(msg)

    def esperar(self, segundos):
        """Espera SEM dormir de verdade: continua processando a rede.

        Substitui o time.sleep() — se dormíssemos, os alertas de
        /hardware_errors só seriam lidos depois da pausa.
        """
        fim = time.monotonic() + segundos
        while time.monotonic() < fim and not self.falha_fatal:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def fechar_conexao(self):
        """Desliga o nó de forma segura."""
        self._node.destroy_node()
        rclpy.shutdown()


# =====================================================================
# 2. LÓGICA DA MARCHA (matemática pura, sem ROS)
# =====================================================================
def calcular_velocidade_rad_s(rad_alvo, rad_anterior, passo):
    """Velocidade para cobrir a distância no tempo 'passo' (rad/s).

    Todas as juntas recebem velocidades proporcionais às suas distâncias,
    então todas chegam ao alvo AO MESMO TEMPO (movimento coordenado).
    """
    delta_rad = abs(rad_alvo - rad_anterior)
    return delta_rad / passo


def main():
    robo = ConexaoRobo()

    # =================================================================
    # A MATRIZ DE MOVIMENTO
    # Uma LINHA por junta (mesma ordem de nomes_juntas, abaixo).
    # Uma COLUNA por passo do ciclo de marcha. Valores em radianos.
    # =================================================================
    matriz_movimento = [
        [-0.7418,  0.0873,  0.2182, -0.7418,  0.5236,  0.5236, -0.7418],  # PD_tornozelo
        [-0.7418,  0.0873,  0.2182, -0.7418,  0.5236,  0.5236, -0.7418],  # PE_tornozelo
        [ 1.3963,  1.0472,  0.4363,  0.1745,  0.5236,  0.7854,  1.3963],  # PD_joelho
        [ 1.3963,  1.0472,  0.4363,  0.1745,  0.5236,  0.7854,  1.3963],  # PE_joelho
        [ 0.0,    -0.7854, -0.6981,  0.4800,  0.4800,  0.3491,  0.0   ],  # PD_quadril
        [ 0.0,    -0.7854, -0.6981,  0.4800,  0.4800,  0.3491,  0.0   ],  # PE_quadril
    ]

    # Nomes das juntas: PRECISAM ser idênticos ao joint_map do controlador.
    # Os rolls de tornozelo (roll_3 e roll_4) estão ativos no controlador,
    # mas de propósito NÃO entram na marcha: recebem torque e ficam rígidos
    # na posição em que estiverem. Para movê-los, acrescente o nome aqui e
    # uma linha correspondente na matriz_movimento, na MESMA ordem.
    nomes_juntas = [
        'PD_tornozelo_pitch_1',
        'PE_tornozelo_pitch_2',
        'PD_joelho_pitch_5',
        'PE_joelho_pitch_6',
        'PD_quadril_pitch_7',
        'PE_quadril_pitch_8',
    ]

    # Variáveis de tempo
    passo = 1.0   # duração da transição entre poses (s)
    pausa = 0.5   # repouso extra após cada transição (s)
    tempo_total_espera = passo + pausa

    # --- VALIDAÇÃO: pega erro de digitação ANTES de mexer no robô ---
    if len(matriz_movimento) != len(nomes_juntas):
        print(f'ERRO: a matriz tem {len(matriz_movimento)} linhas, '
              f'mas ha {len(nomes_juntas)} juntas. Corrija antes de rodar.')
        robo.fechar_conexao()
        return

    num_points = len(matriz_movimento[0])
    for n, linha in enumerate(matriz_movimento):
        if len(linha) != num_points:
            print(f'ERRO: a linha {n} da matriz tem {len(linha)} colunas; '
                  f'as outras tem {num_points}. Corrija antes de rodar.')
            robo.fechar_conexao()
            return

    # Histórico: começa na primeira coluna (pose inicial do ciclo)
    posicoes_anteriores = [linha[0] for linha in matriz_movimento]

    current_index = 0

    try:
        # Só começa a marcha quando o controlador estiver ouvindo
        robo.aguardar_controlador()
        print('Iniciando ciclo de marcha... (Ctrl+C para parar)')

        while not robo.falha_fatal:
            posicoes_alvo = []
            velocidades_alvo = []

            # Monta o passo atual: uma linha da matriz por junta
            for i in range(len(nomes_juntas)):
                rad_alvo = matriz_movimento[i][current_index]
                vel_rad = calcular_velocidade_rad_s(
                    rad_alvo, posicoes_anteriores[i], passo)

                posicoes_alvo.append(rad_alvo)
                velocidades_alvo.append(vel_rad)
                posicoes_anteriores[i] = rad_alvo

            # Envia para a Raspberry Pi
            robo.enviar_passo(nomes_juntas, posicoes_alvo, velocidades_alvo, passo)
            print(f'Passo {current_index + 1}/{num_points} enviado | '
                  f'Aguardando {tempo_total_espera}s')

            # Avança a coluna, voltando ao início no fim do ciclo
            current_index = (current_index + 1) % num_points

            # Espera processando a rede (recebe alertas durante a pausa)
            robo.esperar(tempo_total_espera)

        # Só chega aqui se o controlador reportou FALHA FATAL
        print('\nMarcha INTERROMPIDA: o controlador desistiu do hardware. '
              'Verifique o robo e reinicie os dois nos.')

    except KeyboardInterrupt:
        print('\nScript interrompido pelo usuário.')
    finally:
        robo.fechar_conexao()


if __name__ == '__main__':
    main()
