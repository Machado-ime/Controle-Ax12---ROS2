"""Cliente de marcha para o robô bípede AX-12.

Roda no PC de comando. Lê a sequência de passos (matriz de movimento)
de um arquivo YAML, publica passo a passo em /joint_trajectory e escuta
/hardware_errors para saber quando o ax12_controller (na Raspberry Pi)
está com problemas.

As matrizes de marcha moram em arquivos .yaml NA MESMA PASTA deste script.
Para trocar de marcha basta o NOME do arquivo (sem pasta, .yaml opcional):
    ros2 run ax12_control send_gait --ros-args -p matriz:=cin_inve
Sem o parâmetro, usa 'otimizada'. Assim dá para ajustar o movimento SEM
editar este código.

Antes de começar a marcha, espera o controlador aparecer na rede —
sem isso, os primeiros comandos se perdem durante a descoberta do DDS.
"""

import os
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml


# Pasta onde ficam as matrizes (.yaml): a mesma deste script.
PASTA_MATRIZES = os.path.dirname(os.path.abspath(__file__))


def resolver_caminho_matriz(nome):
    """Transforma o NOME de uma matriz no caminho do arquivo .yaml.

    - 'cin_inve'              -> <pasta deste script>/cin_inve.yaml
    - 'cin_inve.yaml'         -> <pasta deste script>/cin_inve.yaml
    - '/outra/pasta/x.yaml'   -> usado como veio (tem pasta no caminho)

    Assim basta jogar os .yaml na mesma pasta do send_gait e pedir só
    pelo nome. Um caminho completo ainda funciona para arquivos de fora.
    """
    nome = str(nome).strip()
    # Já é um caminho (contém pasta)? Respeita como veio.
    if os.path.dirname(nome):
        return nome
    # Nome simples: garante a extensão e prende na pasta deste script.
    if not nome.lower().endswith(('.yaml', '.yml')):
        nome += '.yaml'
    return os.path.join(PASTA_MATRIZES, nome)


# =====================================================================
# 1. A PONTE DE CONEXÃO (esconde o nó ROS nos bastidores)
# =====================================================================
class ConexaoRobo:
    """Publica comandos para o ax12_controller e escuta os erros dele."""

    def __init__(self):
        rclpy.init()
        self._node = rclpy.create_node('gait_client')

        # --- Qual matriz de marcha usar (parâmetro ROS 'matriz') ---
        # Basta o NOME do .yaml, que mora na mesma pasta deste script:
        #   ros2 run ax12_control send_gait --ros-args -p matriz:=cin_inve
        # Sem o parâmetro, usa 'otimizada'.
        self._node.declare_parameter('matriz', 'otimizada')
        nome_matriz = self._node.get_parameter('matriz').value
        self.arquivo_marcha = resolver_caminho_matriz(nome_matriz)

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
def carregar_marcha(caminho):
    """Lê o YAML da marcha e devolve (nomes_juntas, matriz, passo, pausa).

    Toda a validação acontece AQUI, antes de qualquer comando ir ao robô:
    arquivo ausente, campo faltando, tipo errado ou matriz inconsistente
    levantam exceção e o main() aborta sem mexer no hardware.

    Formato esperado (ver otimizada.yaml):
        passo: 1.0
        pausa: 0.5
        nomes_juntas: [<nome>, ...]          # 1 por junta
        matriz_movimento: [[...], ...]        # 1 linha por junta
    """
    with open(caminho, 'r', encoding='utf-8') as f:
        dados = yaml.safe_load(f)

    if not isinstance(dados, dict):
        raise ValueError('o arquivo nao contem um mapa YAML (chave: valor).')

    for chave in ('passo', 'pausa', 'nomes_juntas', 'matriz_movimento'):
        if chave not in dados:
            raise ValueError(f'falta a chave obrigatoria "{chave}".')

    passo = float(dados['passo'])
    pausa = float(dados['pausa'])
    nomes_juntas = dados['nomes_juntas']
    matriz = dados['matriz_movimento']

    if passo <= 0:
        raise ValueError(f'"passo" deve ser maior que zero (veio {passo}).')
    if pausa < 0:
        raise ValueError(f'"pausa" nao pode ser negativa (veio {pausa}).')

    if not isinstance(nomes_juntas, list) or not nomes_juntas:
        raise ValueError('"nomes_juntas" deve ser uma lista nao vazia.')
    if not isinstance(matriz, list) or not matriz:
        raise ValueError('"matriz_movimento" deve ser uma lista nao vazia.')

    # Uma linha por junta
    if len(matriz) != len(nomes_juntas):
        raise ValueError(
            f'a matriz tem {len(matriz)} linhas, mas ha '
            f'{len(nomes_juntas)} juntas em nomes_juntas.')

    # Todas as linhas com o mesmo número de colunas (mesmo nº de passos)
    num_points = len(matriz[0])
    if num_points == 0:
        raise ValueError('a matriz nao tem nenhuma coluna (nenhum passo).')
    for n, linha in enumerate(matriz):
        if len(linha) != num_points:
            raise ValueError(
                f'a linha {n} da matriz tem {len(linha)} colunas; '
                f'as outras tem {num_points}.')

    return nomes_juntas, matriz, passo, pausa


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
    # Carrega a marcha do arquivo YAML (caminho vem do parâmetro 'arquivo').
    # Qualquer problema no arquivo aborta ANTES de mexer no robô.
    # =================================================================
    try:
        nomes_juntas, matriz_movimento, passo, pausa = carregar_marcha(
            robo.arquivo_marcha)
    except FileNotFoundError:
        print(f'ERRO: arquivo de marcha nao encontrado: {robo.arquivo_marcha}')
        robo.fechar_conexao()
        return
    except (yaml.YAMLError, ValueError, TypeError) as e:
        print(f'ERRO no arquivo de marcha ({robo.arquivo_marcha}): {e}')
        robo.fechar_conexao()
        return

    print(f'Marcha carregada de: {robo.arquivo_marcha}')

    # Variáveis de tempo (vindas do arquivo)
    tempo_total_espera = passo + pausa
    num_points = len(matriz_movimento[0])

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
