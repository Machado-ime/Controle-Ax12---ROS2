"""Interface de hardware ROS 2 para servomotores Dynamixel AX-12.

Este nó é o ÚNICO processo que toca o barramento serial dos motores.
Ele assina /joint_trajectory (posições em rad, velocidades em rad/s),
converte para as unidades do motor (ver MODELOS) e escreve tudo num
único pacote SyncWrite. Falhas de hardware são publicadas em
/hardware_errors.

As 10 juntas são AX-12A (Protocolo 1.0) — confirmado lendo o registrador
Model Number (end. 0) de cada ID do barramento. A conversão rad<->unidades
fica parametrizada em MODELOS, e não em constante solta, para o dia em que
um motor de outra resolução for instalado: basta acrescentar a entrada e
apontar a junta para ela nos dois pontos marcados com "modelo = ".

Com `-p ligar_torque:=false` o nó vira SOMENTE-LEITURA: publica a telemetria
mas não escreve nada no barramento — não liga o torque, não o desliga ao
sair e descarta comandos. Serve para espelhar no RViz um robô movido à mão
e para diagnosticar o barramento sem energizar os motores.

A reconexão da porta é disparada por um timer próprio (`intervalo_reconexao`),
independente de haver comandos chegando ou telemetria ligada. A abertura da
porta liga o modo de baixa latência e espera `pausa_pos_abertura` antes do
primeiro pacote — a OpenCR com firmware usb_to_dxl faz o barramento seguir o
baudrate do USB e leva uma iteração do loop() dela para reconfigurar.

Este nó cuida APENAS dos motores. Não há leitura de IMU: a OpenCR aqui é só
a ponte USB-serial, e o ID 200 (que traria o IMU) exige outro firmware.

Referência da tabela de controle:
https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

# pyserial: usado apenas para reconhecer a exceção lançada quando a USB cai
import serial

# ERROS_SERIAL: tudo que significa "a porta morreu debaixo de nós".
#
# O detalhe que custa caro: termios.error NÃO herda de OSError (o MRO é
# error -> Exception), então `except OSError` não o pega. E ele é justamente
# o que sobe quando a USB é arrancada: o SDK chama port.clearPort() antes de
# CADA pacote, que vira ser.flush() -> termios.tcdrain(fd) -> termios.error(5,
# 'Input/output error'). Sem termios.error nesta tupla, arrancar o cabo não
# virava reconexão — derrubava o nó inteiro com traceback.
# serial.SerialException já é subclasse de OSError; fica na tupla por clareza.
try:
    import termios
    _ERROS_TERMIOS = (termios.error,)
except ImportError:            # termios é POSIX-only
    _ERROS_TERMIOS = ()
ERROS_SERIAL = (serial.SerialException, OSError) + _ERROS_TERMIOS

# Importações explícitas (em vez de "import *") para sabermos o que vem do SDK
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_MAKEWORD,
)

# Orçamento de timeout de pacote do SDK. A fórmula é
# port_handler.setPacketTimeout: tx + LATENCY_TIMER*2 + 2 (ms).
#
# Os 16 ms cravados no SDK são herança de adaptadores FTDI, que de fato
# seguram os bytes por esse tempo. Com eles, CADA leitura que falha custa
# ~34 ms: 10 motores mudos gastam 341 ms e estouram o período de 200 ms do
# timer de telemetria, saturando o executor e atrasando os comandos.
#
# Encurtar o orçamento só é seguro quando a latência real for mesmo baixa.
# Por isso o valor NÃO é escolhido aqui: quem decide é _abrir_porta(), depois
# de saber se set_low_latency_mode() funcionou naquela porta. Com 16 ms de
# latência real e 4 ms de orçamento, uma resposta VÁLIDA expiraria e o motor
# seria reportado como mudo — fabricando exatamente o sintoma que este nó
# existe para diagnosticar.
#
# RESSALVA: set_low_latency_mode() retornar sem exceção NÃO prova que a
# latência caiu. No cdc_acm o ioctl é aceito, mas o flag ASYNC_LOW_LATENCY já
# não tem efeito real no kernel moderno — ele pode ter sucesso sendo um no-op.
# A escolha continua segura, só que por outro motivo: o cdc_acm não tem o
# buffering de 16 ms do FTDI (entrega os URBs a cada frame USB, ~1 ms), então
# 4 ms bastam ali de qualquer forma; e o FTDI, que realmente segura 16 ms, é
# justamente o driver que honra o flag. Um adaptador exótico que faça as duas
# coisas (segurar bytes E aceitar o flag sem efeito) cairia nessa brecha.
#
# O valor é escrito no MÓDULO (_dxl_port_handler.LATENCY_TIMER), e não
# importado como constante, porque a fórmula do SDK lê a variável do escopo do
# módulo a cada chamada — importar o valor congelaria o número no import.
import dynamixel_sdk.port_handler as _dxl_port_handler

LATENCY_MS_BAIXA  = 4    # só com set_low_latency_mode ativo (latência real ~1 ms)
LATENCY_MS_PADRAO = 16   # padrão do SDK; seguro em qualquer adaptador

# Balde furado da detecção de falha de leitura: cada falha soma 1, cada
# sucesso subtrai DRENAGEM_POR_SUCESSO. O nível sobe enquanto
#     p_falha > DRENAGEM / (1 + DRENAGEM)
# então a drenagem escolhe a PERDA MÍNIMA DETECTÁVEL:
#     1.00 -> só acusa acima de 50% de perda
#     0.50 -> acima de 33%
#     0.25 -> acima de 20%   <-- escolhido
#     0.10 -> acima de 9%
# Começou em 1.0, e o teste de bancada mostrou o buraco: mexendo no
# conector de 3 pinos o barramento perdeu 3,3% num teste e 19 leituras
# seguidas na pior rajada, e o nó não disse nada. Pior: com 1.0, um
# barramento perdendo 30% dos pacotes — claramente defeituoso — ficaria
# silencioso PARA SEMPRE, porque cada sucesso cancelava uma falha inteira.
# 0.25 mantém ruído pontual sem alarme e acusa degradação real.
DRENAGEM_POR_SUCESSO = 0.25

# Baudrates que o SDK aceita (port_handler.getCFlagBaud). Qualquer outro faz
# setBaudRate() devolver False, o que o nó checa na partida para acusar erro
# de configuração em vez de culpar o cabo.
BAUDRATES_SUPORTADOS = (
    9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000,
    921600, 1000000, 1152000, 2000000, 2500000, 3000000, 3500000, 4000000,
)

# =====================================================================
# Tabela de controle do AX-12 (Protocolo 1.0)
# =====================================================================
PROTOCOL_VERSION        = 1.0
ADDR_TORQUE_ENABLE      = 24   # 1 byte  (0 = desliga, 1 = liga)
ADDR_GOAL_POSITION      = 30   # 2 bytes (0 a 1023 = 0° a 300°)
# Não é lido por nome em lugar nenhum: o SyncWrite alcança este registrador
# escrevendo 4 bytes a partir do 30 (ver LEN_GOAL_POS_E_SPEED logo abaixo).
# Fica aqui porque o mapa da tabela de controle é a referência de quem for
# estender o nó — apagá-lo economizaria uma linha e custaria a documentação.
ADDR_MOVING_SPEED       = 32   # 2 bytes (0 a 1023; 0 = velocidade MÁXIMA!)

# Goal Position (30) e Moving Speed (32) são vizinhos na tabela de controle.
# Escrevendo 4 bytes a partir do endereço 30, enviamos posição E velocidade
# num único pacote SyncWrite: todos os motores partem juntos, já na
# velocidade certa, sem precisar de uma escrita individual por motor.
LEN_GOAL_POS_E_SPEED    = 4

# --- Registradores de telemetria (leitura) ---
# Bloco contíguo 36 a 43: posição(2) + velocidade(2) + carga(2) +
# tensão(1) + temperatura(1) — lemos os 8 bytes numa ÚNICA transação.
ADDR_PRESENT_POSITION   = 36
LEN_BLOCO_TELEMETRIA    = 8

# O AX-12 não tem sensor de torque verdadeiro: o Present Load (end. 40)
# é a estimativa interna do esforço, em % do torque máximo. Convertemos
# para N·m usando o stall torque nominal (aproximação) — ver
# 'torque_max_nm' em MODELOS.

# =====================================================================
# Fatores de conversão (unidades do ROS <-> unidades do motor):
# AX-12 é 0-1023 sobre 300° (curso útil ±150°), Protocolo 1.0.
# =====================================================================
LIMITE_RAD = 2.618   # ±150°, o curso útil do AX-12

MODELOS = {
    'AX12': dict(
        limite_rad=2.618,             # ±150° (curso útil 0-300°)
        max_pos=1023,
        unidades_por_rad_s=86.03,     # rad/s -> unidades (1 un. = 0,111 rpm)
        torque_max_nm=1.5,            # stall torque nominal a 12 V
    ),
}
for _cfg in MODELOS.values():
    _cfg['pos_por_rad'] = _cfg['max_pos'] / (2 * _cfg['limite_rad'])
del _cfg

# Não replique valores de MODELOS em constantes soltas aqui. Existiam
# POS_POR_RAD e UNIDADES_POR_RAD_S "por compatibilidade"; nada as lia (todo
# o código usa modelo['...']) e elas só criavam a chance de divergir do
# dicionário no dia em que um valor fosse ajustado num lugar só.

# NOTA sobre o ID 200 (OpenCR): este nó NÃO fala com a placa como
# dispositivo do barramento — ela é apenas a ponte USB-serial. O ID 200 só
# existe com o firmware estilo OP3 (opencr_dxl_imu_bridge), que expõe uma
# tabela de controle própria com IMU e com o rail de 12 V dos motores. Com o
# firmware usb_to_dxl, que é o gravado nesta placa (confirmado lendo o ID 200
# e não obtendo resposta), nada disso existe: ela só encaminha bytes.
# A leitura de IMU foi removida por não fazer parte desta fase do projeto.


class AX12HardwareInterface(Node):

    def __init__(self):
        super().__init__('ax12_hardware_interface')

        # --- Parâmetros ROS (mude sem editar o código) ---
        # Ex.: ros2 run ax12_control ax12_controller --ros-args -p device:=/dev/ttyUSB0
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('tentativas_abertura', 5)    # tentativas ao iniciar o nó
        # Desiste após N reconexões falhas. O orçamento real é
        # N x intervalo_reconexao, então 300 x 1 s = 5 minutos.
        # Já foi 10 (= 10 segundos), e o teste com hardware mostrou que é curto
        # demais: arrancado o cabo USB, o nó declarou FALHA FATAL em 8,6 s e a
        # porta só reapareceu aos 43 s — quando ele já estava em `desativado`,
        # de onde não sai mais. Religar leva tempo de gente: no WSL exige rodar
        # `usbipd attach` à mão, e numa Raspberry Pi a USB demora a re-enumerar.
        self.declare_parameter('max_falhas_reconexao', 300)
        # Pausa entre abrir a porta e o primeiro pacote. A OpenCR com firmware
        # usb_to_dxl só reconfigura a Serial3 para o baud do USB na iteração
        # seguinte do loop() dela; sem esta margem o primeiro pacote sai no baud
        # errado. Adaptadores "burros" (U2D2, FTDI) aceitam 0.0 sem problema.
        self.declare_parameter('pausa_pos_abertura', 0.2)   # segundos
        # Período do timer que tenta reerguer a porta caída. O orçamento de
        # max_falhas_reconexao passa a ser contado em TEMPO (N tentativas a
        # cada intervalo), e não por comando recebido como era antes.
        self.declare_parameter('intervalo_reconexao', 1.0)  # segundos
        self.declare_parameter('velocidade_padrao', 100)    # usada se a msg vier sem velocities
        self.declare_parameter('taxa_leitura', 5.0)         # Hz da telemetria (0 desliga)
        # Quanto tempo de silêncio ACUMULADO de um motor antes de avisar.
        # Em segundos, e não em ciclos: o limiar era 25 ciclos fixos, cujo
        # significado mudava com taxa_leitura (5 s a 5 Hz, 2,5 s a 10 Hz),
        # deixando o log ambíguo para quem não soubesse a taxa configurada.
        self.declare_parameter('segundos_falha_aviso', 5.0)
        # MODO OBSERVADOR: com ligar_torque:=false o nó vira somente-leitura —
        # não liga o torque ao iniciar, não religa ao reconectar, não desliga
        # ao sair e DESCARTA comandos de /joint_trajectory. Serve para espelhar
        # no RViz um robô movido à mão, e para diagnosticar barramento sem
        # energizar os motores. O descarte de comandos não é preciosismo: o
        # AX-12 aceita Goal Position com torque desligado e guarda o valor,
        # então um comando recebido agora viraria um salto brusco no instante
        # em que alguém ligasse o torque depois.
        self.declare_parameter('ligar_torque', True)

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        self.velocidade_padrao = self.get_parameter('velocidade_padrao').value
        self.ligar_torque = self.get_parameter('ligar_torque').value

        # --- VALIDAÇÃO DOS PARÂMETROS NUMÉRICOS ---
        # Nenhum destes dá "erro de configuração" quando recebe valor bobo: dá
        # traceback, 100% de CPU, ou — pior — silêncio com mensagem enganosa.
        # Corrigimos para o padrão avisando, em vez de abortar: é um nó de
        # hardware, e subir com valor são é melhor que não subir.
        self.pausa_pos_abertura = self.get_parameter('pausa_pos_abertura').value
        if self.pausa_pos_abertura < 0:
            self.get_logger().warn(
                f'pausa_pos_abertura negativa ({self.pausa_pos_abertura}); usando 0.0. '
                'time.sleep() nao aceita negativo e o ValueError derrubaria o no.')
            self.pausa_pos_abertura = 0.0

        self.intervalo_reconexao = self.get_parameter('intervalo_reconexao').value
        if self.intervalo_reconexao <= 0:
            self.get_logger().warn(
                f'intervalo_reconexao invalido ({self.intervalo_reconexao}); usando 1.0 s. '
                'Periodo zero e aceito pelo rclpy e faz o executor girar a 100% de CPU.')
            self.intervalo_reconexao = 1.0

        # Com 0 ou negativo, range(1, n+1) fica VAZIO: o laço de abertura não
        # roda nenhuma vez e o nó morre dizendo "nao foi possivel abrir a
        # porta" sem nunca ter tentado — uma mensagem que afirma algo falso.
        self.tentativas_abertura = self.get_parameter('tentativas_abertura').value
        if self.tentativas_abertura < 1:
            self.get_logger().warn(
                f'tentativas_abertura invalido ({self.tentativas_abertura}); usando 1. '
                'Zero ou negativo faria o no desistir sem tentar abrir a porta.')
            self.tentativas_abertura = 1

        # Com 0 ou negativo, a PRIMEIRA falha de reconexão já satisfaz
        # `falhas >= max` e desativa o nó em definitivo.
        self.max_falhas_reconexao = self.get_parameter('max_falhas_reconexao').value
        if self.max_falhas_reconexao < 1:
            self.get_logger().warn(
                f'max_falhas_reconexao invalido ({self.max_falhas_reconexao}); usando 1. '
                'Zero ou negativo desativaria o no na primeira falha de reconexao.')
            self.max_falhas_reconexao = 1

        # Baudrate: o SDK só aceita valores tabelados, e um fora da lista faz
        # setBaudRate() devolver False — indistinguível, para quem lê o log, de
        # cabo solto ou porta errada. Sem esta checagem, digitar um baudrate
        # inválido produzia cinco "Falha ao abrir a porta" seguidos, culpando o
        # hardware por um erro de digitação.
        if self.baudrate not in BAUDRATES_SUPORTADOS:
            raise RuntimeError(
                f'baudrate {self.baudrate} nao e suportado pelo Dynamixel SDK. '
                f'Valores aceitos: {", ".join(str(b) for b in BAUDRATES_SUPORTADOS)}.')

        self._avisou_observador = False   # o aviso de comando descartado sai uma vez só
        self._avisou_baixa_latencia = False  # o aviso de latência sai uma vez por sessão

        # Mapa das juntas (nome ROS -> ID do motor no barramento).
        # Nomes seguem a convenção do URDF (adam.urdf): {lado}_{movimento}_{segmento}_{N}.
        # Os motores foram regravados para que o ID no barramento SEJA o sufixo N
        # do nome: pd_picht_tornozelo_3 é o ID 3, pe_roll_quadril_10 é o ID 10.
        # Antes os dois números eram diferentes (o 3 era o motor de ID 12), o que
        # já custou horas de diagnóstico. Ao trocar um motor, regrave o ID dele
        # para casar com o sufixo em vez de editar este mapa.
        self.joint_map = {
            'pd_picht_tornozelo_3': 3,
            'pe_picht_tornozelo_4': 4,
            'pd_roll_tornozelo_1': 1,
            'pe_roll_tornozelo_2': 2,    # recebem torque e seguram a posição
            'pd_picht_joelho_5': 5,
            'pe_picht_joelho_6': 6,
            'pd_picht_quadril_7': 7,
            'pe_pich_quadril_8': 8,
            'pd_roll_quadril_9': 9,      # quadril roll: novo, sem medição ainda
            'pe_roll_quadril_10': 10,    # (fora da marcha; segura posição)
            # Juntas ainda sem ID no barramento (braços, pescoço): os sufixos
            # 11 a 16 do URDF (ombros e cotovelos) ficam reservados para elas,
            # e a mesma regra vale — grave no motor o ID igual ao sufixo.
        }
        self.active_ids = list(self.joint_map.values())

        # Limites de posição por junta (rad) medidos no hardware e convertidos via:
        #   rad = (grau_AX12 - 150) * pi/180
        # onde grau_AX12 é a posição na escala 0-300° do AX-12.
        # PD (direito) e PE (esquerdo) são espelhados: os limites de PE são
        # (lo, hi) direto da medição; PD recebe (-hi, -lo) para refletir a montagem.
        self.joint_limits = {
            'pd_picht_tornozelo_3': (-1.4661,     0.5585),  # espelho de PE
            'pe_picht_tornozelo_4': (-0.5585,     1.4661),  # (118°,234°) → (-32°,+84°)
            'pd_roll_tornozelo_1':  (-0.8727,     0.5934),  # espelho de PE
            'pe_roll_tornozelo_2':  (-0.5934,     0.8727),  # (116°,200°) → (-34°,+50°)
            'pd_picht_joelho_5':    (0.0,         LIMITE_RAD),  # (150°,300°) → (0°,+150°)
            'pe_picht_joelho_6':    (-LIMITE_RAD, 0.0),         # espelho de PD
            'pd_picht_quadril_7':   (-LIMITE_RAD, LIMITE_RAD),  # sem medição ainda
            'pe_pich_quadril_8':    (-LIMITE_RAD, LIMITE_RAD),
            'pd_roll_quadril_9':    (-LIMITE_RAD, LIMITE_RAD),  # sem medição ainda
            'pe_roll_quadril_10':   (-LIMITE_RAD, LIMITE_RAD),
        }

        # Juntas cujo motor está montado com o eixo INVERTIDO em relação ao
        # URDF: o mesmo comando +θ gira o modelo para um lado e o motor real
        # para o outro (URDF leva o pé à frente, motor leva atrás). Corrigimos
        # trocando o sinal APENAS na fronteira rad<->unidades do motor (na
        # escrita e na leitura), mantendo todo o resto — joint_limits, marcha,
        # /joint_states, RViz — na convenção do URDF. É o mesmo que um
        # SystemInterface do ros2_control faz com um flag de direção por junta.
        # Os limites em joint_limits JÁ estão na convenção do URDF (medidos no
        # motor e negados), então o clamp continua correto após a inversão.
        # São as 4 juntas de PITCH de tornozelo e quadril (verificado no
        # hardware); joelhos e rolls não são invertidos.
        self.juntas_invertidas = {
            'pd_picht_tornozelo_3',
            'pe_picht_tornozelo_4',
            'pd_picht_quadril_7',
            'pe_pich_quadril_8',
            # Rolls de quadril: verificados no robô, giram ao contrário do
            # URDF. Os limites deles são simétricos (±LIMITE_RAD), então o
            # clamp continua correto após a troca de sinal. Os SINAIS_ROLL do
            # medir_roll/controle_pe foram ajustados junto, para o slider
            # continuar movendo o robô no mesmo sentido físico de antes.
            'pd_roll_quadril_9',
            'pe_roll_quadril_10',
        }

        # --- Estado da conexão serial ---
        self.port_ok = False          # a porta está aberta e funcionando?
        self.falhas_reconexao = 0     # reconexões falhas consecutivas
        self.desativado = False       # True = desistimos do hardware (falha fatal)

        # --- Publisher de erros de hardware ---
        # QoS RELIABLE (padrão): aviso de erro é raro e PRECISA chegar,
        # ao contrário dos comandos de marcha, que são frequentes e descartáveis.
        self.error_publisher = self.create_publisher(String, '/hardware_errors', 10)

        # --- Publishers de telemetria ---
        # /joint_states usa o perfil padrão de sensores (BEST_EFFORT):
        # telemetria perdida não deve ser reenviada atrasada.
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', qos_profile_sensor_data)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # Memória da leitura: último erro visto por motor (para alertar só
        # quando MUDA, e não 5x por segundo) e falhas de leitura seguidas
        self._ultimo_erro_lido = {}
        self._falhas_leitura = {}   # dxl_id -> nível do balde (ver ler_motores_callback)
        self._avisou_leitura = set()  # dxl_ids que já geraram aviso de falha
        self._clampado = {}        # junta -> estava clampada na última mensagem?

        # --- Objetos do Dynamixel SDK ---
        self.portHandler = PortHandler(self.device)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_GOAL_POS_E_SPEED)

        # 1. Abre a porta serial (com várias tentativas: a USB pode demorar
        #    a enumerar logo após o boot da Raspberry Pi)
        if not self._abrir_porta_com_tentativas():
            # Sem hardware o nó não tem função: aborta a criação.
            # O main() captura este erro e encerra de forma limpa.
            raise RuntimeError(f'Nao foi possivel abrir a porta {self.device}.')

        # 2. Liga o torque dos motores mapeados (ou só confere quem responde,
        #    no modo observador)
        if self.ligar_torque:
            self._ligar_torque()
            self.get_logger().info('Torque LIGADO. Pronto para receber e escrever comandos.')
        else:
            self._verificar_presenca()
            self.get_logger().info(
                'MODO OBSERVADOR (ligar_torque:=false): torque intocado, comandos '
                'descartados. Só telemetria — mova o robô à mão e veja no RViz.')

        # --- PERFIL DE REDE (QoS) BLINDADO PARA WI-FI ---
        # BEST_EFFORT + fila de 1: comando perdido é descartado, nunca
        # reenviado atrasado (comando velho é pior que comando perdido).
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 3. Assina o tópico de trajetórias
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.listener_callback,
            qos_profile
        )

        # O limiar do balde é definido em SEGUNDOS e convertido para nível
        # usando a taxa de cada leitor. Como uma falha soma exatamente 1, o
        # nível equivale a "ciclos de silêncio total", e nível/taxa é o tempo
        # de silêncio equivalente — o que torna a mensagem legível sem que o
        # leitor precise saber a taxa configurada.
        segundos_aviso = self.get_parameter('segundos_falha_aviso').value
        if segundos_aviso <= 0:
            self.get_logger().warn(
                f'segundos_falha_aviso invalido ({segundos_aviso}); usando 5.0.')
            segundos_aviso = 5.0

        # 4. Timer da telemetria (taxa_leitura = 0 desliga a leitura)
        taxa = self.get_parameter('taxa_leitura').value
        self.taxa_leitura = taxa
        self._limiar_leitura = max(1, round(segundos_aviso * taxa)) if taxa > 0 else 1
        if taxa > 0:
            self.create_timer(1.0 / taxa, self.ler_motores_callback)

        # 5. Timer da reconexão — roda SEMPRE, e é o único gatilho dela.
        #    Antes a reconexão só acontecia dentro do listener_callback, o que
        #    a deixava inalcançável em dois casos reais: no modo observador
        #    (que retorna antes de chegar lá) e quando ninguém está publicando
        #    em /joint_trajectory. Nos dois, a porta caía e o nó ficava vivo
        #    porém mudo para sempre, sem nem chegar à mensagem de falha fatal.
        self.create_timer(self.intervalo_reconexao, self._reconectar_callback)

    # =================================================================
    # FUNÇÕES DE APOIO (conexão, torque e avisos de erro)
    # =================================================================

    def _avisar_erro(self, texto):
        """Mostra o aviso no terminal E publica em /hardware_errors."""
        self.get_logger().warn(texto)
        if not rclpy.ok():
            return      # encerrando: publicar aqui levantaria RCLError
        msg = String()
        msg.data = texto
        self.error_publisher.publish(msg)

    def _abrir_porta(self):
        """Uma única tentativa de abrir a porta. Retorna True se conseguiu.

        Três cuidados que o caminho ingênuo (openPort + setBaudRate) não tem:

        1. `openPort()` do SDK É literalmente `setBaudRate(baudrate_guardado)`,
           e `setupPort()` fecha a porta antes de reabrir. Encadear os dois
           fazia DOIS ciclos abre-fecha-abre seguidos num dispositivo CDC ACM,
           sem ganho nenhum. Uma chamada a setBaudRate já abre no baud certo.
        2. Modo de baixa latência: sem ele o kernel pode segurar os bytes
           recebidos por até 16 ms. É AQUI que o orçamento de timeout do SDK é
           escolhido, pelo resultado real do ioctl (ver LATENCY_MS_* no topo).
        3. Pausa antes do primeiro pacote: a OpenCR com firmware usb_to_dxl faz
           o barramento SEGUIR o baudrate do USB, mas só reconfigura a Serial3
           na iteração seguinte do loop() dela. Sem a pausa, os primeiros
           pacotes saem no baud errado e ninguém responde.
        """
        try:
            # setBaudRate() já abre a porta (setupPort) no baud pedido.
            if not self.portHandler.setBaudRate(self.baudrate):
                self.port_ok = False
                return False

            # O orçamento de timeout ACOMPANHA o resultado real. Encurtá-lo sem
            # a baixa latência faria respostas válidas expirarem (ver o bloco
            # de LATENCY_MS_* no topo do arquivo).
            try:
                self.portHandler.ser.set_low_latency_mode(True)
            except (OSError, ValueError, AttributeError):
                # Nem todo driver/porta suporta; não é motivo para falhar aqui.
                # O aviso sai uma vez por sessão: numa porta que não suporta o
                # ioctl, ele repetiria a cada reconexão sem nada de novo a dizer.
                _dxl_port_handler.LATENCY_TIMER = LATENCY_MS_PADRAO
                if not self._avisou_baixa_latencia:
                    self._avisou_baixa_latencia = True
                    self.get_logger().warn(
                        'Modo de baixa latencia indisponivel nesta porta. Mantendo o '
                        f'timeout padrao de {LATENCY_MS_PADRAO} ms por pacote: a telemetria '
                        'fica lenta quando o barramento estiver ruim, mas resposta boa '
                        'nao corre risco de expirar.')
            else:
                _dxl_port_handler.LATENCY_TIMER = LATENCY_MS_BAIXA

            time.sleep(self.pausa_pos_abertura)
            self.portHandler.ser.reset_input_buffer()

            self.port_ok = True
            return True
        except ERROS_SERIAL:
            pass  # porta inexistente/ocupada: tratado como falha normal
        self.port_ok = False
        return False

    def _abrir_porta_com_tentativas(self):
        """Tenta abrir a porta N vezes antes de desistir (N = parâmetro ROS)."""
        tentativas = self.tentativas_abertura
        for tentativa in range(1, tentativas + 1):
            if self._abrir_porta():
                self.get_logger().info(
                    f'Porta {self.device} aberta com sucesso (tentativa {tentativa}).')
                return True
            self.get_logger().warn(
                f'Falha ao abrir {self.device} (tentativa {tentativa}/{tentativas}). '
                'Tentando de novo em 2 s...')
            time.sleep(2.0)
        return False

    def _ligar_torque(self):
        """Liga o torque motor a motor, reportando o resultado de CADA um.

        Quem respondeu ganha confirmação no log; quem não respondeu vira
        aviso em /hardware_errors. No final, um resumo X/N conectados.
        """
        conectados = 0
        for joint_name, dxl_id in self.joint_map.items():
            try:
                result, error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            except ERROS_SERIAL:
                self._porta_caiu('ao ligar o torque')
                return
            if result != COMM_SUCCESS:
                self._avisar_erro(
                    f'Motor ID {dxl_id} ({joint_name}) NAO respondeu ao ligar torque: '
                    f'{self.packetHandler.getTxRxResult(result)}')
            elif error != 0:
                # Respondeu (está no barramento), mas reclamando de algo
                # (ex.: sobrecarga, tensão fora da faixa)
                conectados += 1
                self._avisar_erro(
                    f'Motor ID {dxl_id} ({joint_name}) conectou, MAS reportou erro: '
                    f'{self.packetHandler.getRxPacketError(error)}')
            else:
                conectados += 1
                self.get_logger().info(
                    f'Motor ID {dxl_id} ({joint_name}): conectado, torque LIGADO.')
            # VITAL: 50 ms para a fonte estabilizar antes de ligar o próximo
            time.sleep(0.05)

        self._resumo_presenca(conectados, 'conectados')

    def _verificar_presenca(self):
        """Modo observador: confere quem responde SEM escrever em registrador.

        Faz o papel do resumo X/N do _ligar_torque, mas lendo Present Position
        em vez de escrever Torque Enable — assim o nó nunca toca no estado dos
        motores quando ligar_torque:=false.
        """
        conectados = 0
        for joint_name, dxl_id in self.joint_map.items():
            try:
                _, result, _ = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            except ERROS_SERIAL:
                self._porta_caiu('ao verificar a presenca dos motores')
                return
            if result != COMM_SUCCESS:
                self._avisar_erro(
                    f'Motor ID {dxl_id} ({joint_name}) NAO respondeu: '
                    f'{self.packetHandler.getTxRxResult(result)}')
            else:
                conectados += 1
                self.get_logger().info(
                    f'Motor ID {dxl_id} ({joint_name}): presente (torque intocado).')
            time.sleep(0.05)   # mesmo espaçamento do _ligar_torque

        self._resumo_presenca(conectados, 'presentes')

    def _resumo_presenca(self, conectados, adjetivo):
        """Resume quem respondeu, separando falha de barramento de falha de motor.

        A distinção importa porque o conselho é oposto: NENHUM motor
        respondendo quase nunca é ID errado — é energia ou cabo, já que a
        OpenCR enumera no USB e a porta abre normalmente mesmo sem os 12 V
        chegarem aos motores. Antes as duas situações davam a mesma mensagem,
        mandando conferir IDs justamente no caso em que o ID não é o problema.
        """
        total = len(self.joint_map)
        if conectados == total:
            self.get_logger().info(f'Todos os {total} motores {adjetivo}.')
        elif conectados == 0:
            self._avisar_erro(
                f'NENHUM dos {total} motores respondeu. Isso quase nunca e ID errado: '
                'a porta serial abriu, entao o adaptador esta vivo — o que falta e '
                'energia ou cabo. Verifique (1) a fonte 12 V ligada no jack da placa, '
                '(2) a chave de power da placa, (3) o cabo de 3 pinos ate o primeiro '
                'motor e a corrente de cabos entre eles. A OpenCR enumera no USB '
                'mesmo sem os 12 V chegarem aos motores.')
        else:
            self._avisar_erro(
                f'Apenas {conectados}/{total} motores responderam. Como parte do '
                'barramento respondeu, suspeite dos ausentes em si: ID regravado, '
                'cabo solto a partir de um ponto da corrente, ou motor queimado.')

    def _avisar_clamp(self, joint_name, cmd_rad, rads, low, high):
        """Avisa sobre o clamp SÓ na transição (entrou/saiu do limite).

        Era o único aviso do arquivo sem memória de estado, ao lado do
        _ultimo_erro_lido e do contador de ciclos da leitura. Uma junta
        parada fora do limite publicava um aviso por mensagem recebida E
        por junta — inundando /hardware_errors na cadência de quem estivesse
        comandando, justamente quando o tópico precisa estar legível.
        """
        fora = abs(rads - cmd_rad) > 1e-4
        if fora == self._clampado.get(joint_name, False):
            return
        self._clampado[joint_name] = fora
        if fora:
            self._avisar_erro(
                f'{joint_name}: {cmd_rad:.3f} rad fora do limite '
                f'[{low:.3f}, {high:.3f}] — clampado para {rads:.3f} rad.')
        else:
            self.get_logger().info(
                f'{joint_name}: comando voltou para dentro do limite.')

    def _porta_caiu(self, contexto):
        """Marca a porta como caída e avisa. A reconexão acontece no callback."""
        self.port_ok = False
        self._avisar_erro(
            f'PORTA SERIAL CAIU {contexto}! Reconexao automatica a cada '
            f'{self.intervalo_reconexao:g} s.')

    def _reconectar_callback(self):
        """Gatilho periódico da reconexão (timer). Silencioso se a porta está viva."""
        if self.desativado or self.port_ok:
            return
        self._tentar_reconectar()

    def _tentar_reconectar(self):
        """Tenta reerguer a conexão. Após max_falhas_reconexao, desiste de vez."""
        try:
            self.portHandler.closePort()
        except ERROS_SERIAL:
            pass  # a porta já estava morta; só queríamos liberar o descritor

        if self._abrir_porta():
            self.falhas_reconexao = 0
            if self.ligar_torque:
                self._avisar_erro('Porta serial RECONECTADA. Religando o torque dos motores.')
                # Se os motores perderam energia no evento, voltaram com torque OFF
                self._ligar_torque()
            else:
                self._avisar_erro('Porta serial RECONECTADA (modo observador: '
                                  'torque intocado).')
                self._verificar_presenca()
            return

        self.falhas_reconexao += 1
        if self.falhas_reconexao >= self.max_falhas_reconexao:
            self.desativado = True
            self._avisar_erro(
                f'FALHA FATAL: {self.falhas_reconexao} reconexoes falharam. '
                'Desativando a escrita nos motores. Verifique o cabo USB e reinicie o no.')

    # =================================================================
    # FUNÇÃO DE ESCRITA (recebe trajetória em radianos e rad/s)
    # =================================================================

    def listener_callback(self, msg):
        # Falha fatal anterior: ignora tudo até o nó ser reiniciado
        if self.desativado:
            return

        # Modo observador: nao escreve NADA no barramento. Descartar aqui (e
        # nao so deixar de ligar o torque) evita deixar um Goal Position
        # engatilhado, que viraria um salto quando alguem ligasse o torque.
        if not self.ligar_torque:
            if not self._avisou_observador:
                self._avisou_observador = True
                self._avisar_erro(
                    'Comando em /joint_trajectory DESCARTADO: no em modo '
                    'observador (ligar_torque:=false). Reinicie sem esse '
                    'parametro para mover os motores.')
            return

        # Porta caída: só descarta. Quem reergue a conexão é o timer
        # (_reconectar_callback), que roda mesmo sem comando chegando.
        if not self.port_ok:
            return

        # Trajetória vazia não comanda nada
        if not msg.points:
            return

        # Para controle imediato, usamos apenas o primeiro ponto
        point = msg.points[0]

        # --- VALIDAÇÃO: mensagem malformada é DESCARTADA, nunca "consertada" ---
        # (o código antigo assumia 0.0 rad para posição faltante, o que mandava
        # o motor para o centro sem ninguém pedir)
        if len(point.positions) < len(msg.joint_names):
            self._avisar_erro(
                f'Mensagem descartada: {len(msg.joint_names)} juntas, '
                f'mas so {len(point.positions)} posicoes.')
            return

        # Velocidades são opcionais no JointTrajectory; se não vierem,
        # usamos a velocidade padrão (parâmetro) em vez de quase-zero.
        tem_velocidades = len(point.velocities) >= len(msg.joint_names)

        self.groupSyncWrite.clearParam()
        juntas_no_pacote = 0

        for i, joint_name in enumerate(msg.joint_names):
            # Junta que este controlador não conhece: ignora
            if joint_name not in self.joint_map:
                continue
            dxl_id = self.joint_map[joint_name]
            modelo = MODELOS['AX12']   # todas as juntas são AX-12

            cmd_rad = point.positions[i]

            # NaN/inf NUNCA podem chegar ao motor. O clamp abaixo é feito por
            # comparação, e toda comparação com NaN é falsa: min(high, nan)
            # devolve high, então um NaN viraria silenciosamente o LIMITE
            # SUPERIOR da junta — e o aviso de clamp também não dispararia,
            # porque abs(high - nan) > 1e-4 é falso. Ou seja, uma IK que
            # divergisse mandaria a junta ao batente sem deixar rastro.
            if not math.isfinite(cmd_rad):
                self._avisar_erro(
                    f'{joint_name}: posicao invalida ({cmd_rad}) — junta ignorada.')
                continue

            # --- CLAMP POR JUNTA (limites mecânicos do URDF) ---
            low, high = self.joint_limits.get(
                joint_name, (-modelo['limite_rad'], modelo['limite_rad']))
            rads = max(low, min(high, cmd_rad))
            self._avisar_clamp(joint_name, cmd_rad, rads, low, high)

            # Motor com eixo invertido: passa da convenção do URDF para a do
            # motor trocando o sinal (o clamp acima já usou os limites do URDF).
            rads_motor = -rads if joint_name in self.juntas_invertidas else rads

            # --- CONVERSÃO DE POSIÇÃO (rad -> unidades do modelo do motor) ---
            goal_pos = round((rads_motor + modelo['limite_rad']) * modelo['pos_por_rad'])
            goal_pos = max(0, min(modelo['max_pos'], goal_pos))

            # --- CONVERSÃO DE VELOCIDADE (rad/s -> 1 a 1023) ---
            # Mínimo 1, porque 0 significa "velocidade máxima" no motor!
            # O isfinite vale aqui também: um NaN em velocities viraria
            # ValueError no round(), derrubando o callback inteiro.
            if tem_velocidades and math.isfinite(point.velocities[i]):
                velocidade = round(abs(point.velocities[i]) * modelo['unidades_por_rad_s'])
            else:
                velocidade = self.velocidade_padrao
            velocidade = max(1, min(1023, velocidade))

            # --- EMPACOTA posição (2 bytes) + velocidade (2 bytes) juntas ---
            param = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos),
                     DXL_LOBYTE(velocidade), DXL_HIBYTE(velocidade)]
            if self.groupSyncWrite.addParam(dxl_id, param):
                juntas_no_pacote += 1
            else:
                self._avisar_erro(f'addParam falhou para o motor ID {dxl_id} (ID repetido?).')

        # Nenhuma junta reconhecida nesta mensagem: não há o que enviar.
        # Sem esta saída, o txPacket() de um pacote vazio devolve
        # COMM_NOT_AVAILABLE, cuja string é "Protocol does not support this
        # function!" — publicada em /hardware_errors como se fosse falha de
        # hardware, quando na verdade a mensagem só não era para este nó.
        if juntas_no_pacote == 0:
            return

        # --- ENVIA TUDO num único pacote broadcast ---
        try:
            result = self.groupSyncWrite.txPacket()
        except ERROS_SERIAL:
            self._porta_caiu('durante o envio de comando')
            return

        if result != COMM_SUCCESS:
            self._avisar_erro(
                f'Falha no SyncWrite: {self.packetHandler.getTxRxResult(result)}')

    # =================================================================
    # FUNÇÃO DE LEITURA (telemetria: posição, torque, tensão, temperatura)
    # =================================================================

    def ler_motores_callback(self):
        """Lê o bloco de telemetria de cada motor e publica nos tópicos.

        Roda no mesmo thread dos comandos (o rclpy executa um callback de
        cada vez), então leitura e escrita nunca disputam a serial.
        """
        if self.desativado or not self.port_ok:
            return

        agora = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = agora
        diag = DiagnosticArray()
        diag.header.stamp = agora

        for joint_name, dxl_id in self.joint_map.items():
            # UMA transação traz os 8 bytes: pos(2) vel(2) carga(2) V(1) °C(1)
            try:
                dados, result, error = self.packetHandler.readTxRx(
                    self.portHandler, dxl_id,
                    ADDR_PRESENT_POSITION, LEN_BLOCO_TELEMETRIA)
            except ERROS_SERIAL:
                self._porta_caiu('durante a leitura de telemetria')
                return

            # --- BALDE FURADO por motor: falha enche 1, sucesso escoa 1 ---
            # O contador antigo ZERAVA a cada sucesso e avisava em "== 25"
            # exato. Isso deixava passar justamente o defeito mais comum, o
            # cabo mal crimpado: um motor que falha 24 vezes, responde uma e
            # volta a falhar nunca chegava a 25, então nunca avisava. E um
            # motor morto de vez avisava UMA única vez, para sempre.
            # Com o balde, 24 falhas + 1 acerto param em 23 e a rajada
            # seguinte cruza o limiar; motor saudável fica no piso zero.
            if result != COMM_SUCCESS:
                nivel = self._falhas_leitura.get(dxl_id, 0.0) + 1.0
                self._falhas_leitura[dxl_id] = nivel
                # Avisa ao cruzar o limiar e a cada múltiplo dele enquanto durar.
                # Com o balde, o nível só sobe se a perda superar 20% — ruído
                # pontual é drenado e não gera alarme.
                anterior = nivel - 1.0
                if int(nivel // self._limiar_leitura) > int(anterior // self._limiar_leitura):
                    self._avisou_leitura.add(dxl_id)
                    segundos = nivel / self.taxa_leitura if self.taxa_leitura else 0.0
                    self._avisar_erro(
                        f'Motor ID {dxl_id} ({joint_name}) falhando na leitura: '
                        f'{segundos:.1f} s de silencio acumulado '
                        f'(nivel {nivel:.0f}; falha soma 1, acerto desconta '
                        f'{DRENAGEM_POR_SUCESSO}).')
                continue
            nivel = max(0.0, self._falhas_leitura.get(dxl_id, 0.0) - DRENAGEM_POR_SUCESSO)
            self._falhas_leitura[dxl_id] = nivel
            if nivel == 0.0 and dxl_id in self._avisou_leitura:
                self._avisou_leitura.discard(dxl_id)
                self.get_logger().info(
                    f'Motor ID {dxl_id} ({joint_name}) voltou a responder de forma estavel.')
            modelo = MODELOS['AX12']   # todas as juntas são AX-12

            # --- Conversões (inverso das fórmulas de escrita) ---
            pos_raw = DXL_MAKEWORD(dados[0], dados[1])
            vel_raw = DXL_MAKEWORD(dados[2], dados[3])
            carga_raw = DXL_MAKEWORD(dados[4], dados[5])
            tensao = dados[6] / 10.0        # ex.: 119 -> 11,9 V
            temperatura = float(dados[7])   # já vem em °C

            pos_rad = (pos_raw / modelo['pos_por_rad']) - modelo['limite_rad']

            # Velocidade e carga usam 10 bits + bit de direção (>=1024 = horário)
            vel_rad_s = (vel_raw & 0x3FF) / modelo['unidades_por_rad_s']
            if vel_raw >= 1024:
                vel_rad_s = -vel_rad_s

            # Motor com eixo invertido: traz posição e velocidade de volta para
            # a convenção do URDF (inverso da troca de sinal feita na escrita),
            # para o /joint_states e o RViz baterem com o modelo.
            if joint_name in self.juntas_invertidas:
                pos_rad = -pos_rad
                vel_rad_s = -vel_rad_s

            # --- CARGA: só tem significado com o torque LIGADO ---
            # Medido no hardware: com o torque desligado e o motor parado e
            # livre, o Present Load lê 864 CONSTANTE (84,5%) nas 200 leituras
            # de um teste — não é ruído nem carga, é lixo estável que o
            # registrador guarda. Publicar isso como N·m seria inventar
            # medição: o /joint_states anunciaria 1,27 N·m num motor solto, e
            # o /diagnostics ficaria em WARN eterno por "perto do limite de
            # torque" (o limiar é 80%).
            # A convenção do ROS para campo indisponível é NaN, não um número.
            if self.ligar_torque:
                carga_pct = (carga_raw & 0x3FF) / 10.23   # % do torque máximo
                if carga_raw >= 1024:
                    carga_pct = -carga_pct
                torque_nm = carga_pct / 100.0 * modelo['torque_max_nm']  # estimativa!
            else:
                carga_pct = torque_nm = float('nan')

            # --- Monta o /joint_states (padrão ROS: rad, rad/s, N·m) ---
            js.name.append(joint_name)
            js.position.append(pos_rad)
            js.velocity.append(vel_rad_s)
            js.effort.append(torque_nm)

            # --- Flags de erro do motor (overload, tensão, temperatura...) ---
            # Alerta apenas quando o erro MUDA, para não inundar o tópico
            if error != self._ultimo_erro_lido.get(dxl_id, 0):
                self._ultimo_erro_lido[dxl_id] = error
                if error != 0:
                    self._avisar_erro(
                        f'Motor ID {dxl_id} ({joint_name}) com erro de hardware: '
                        f'{self.packetHandler.getRxPacketError(error)}')
                else:
                    self.get_logger().info(
                        f'Motor ID {dxl_id} ({joint_name}): erro de hardware limpou.')

            # --- Monta o /diagnostics (tensão, temperatura, torque, erro) ---
            status = DiagnosticStatus()
            status.name = f'ax12/{joint_name}'
            status.hardware_id = str(dxl_id)
            # math.isnan no lugar de `>= 80.0` direto: com o torque desligado
            # carga_pct é NaN, toda comparação com NaN é falsa, e o nível cairia
            # em OK por acidente. Melhor dizer explicitamente que só a
            # temperatura é avaliável nesse caso.
            carga_valida = not math.isnan(carga_pct)
            if error != 0:
                status.level = DiagnosticStatus.ERROR
                status.message = self.packetHandler.getRxPacketError(error)
            elif temperatura >= 65.0:
                status.level = DiagnosticStatus.WARN
                status.message = 'Temperatura perto do limite'
            elif carga_valida and abs(carga_pct) >= 80.0:
                status.level = DiagnosticStatus.WARN
                status.message = 'Torque perto do limite'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK' if carga_valida else 'OK (torque desligado: carga nao medida)'
            status.values = [
                KeyValue(key='angulo_graus', value=f'{math.degrees(pos_rad):.1f}'),
                KeyValue(key='torque_pct',
                         value=f'{carga_pct:.1f}' if carga_valida else 'n/d'),
                KeyValue(key='torque_nm_estimado',
                         value=f'{torque_nm:.2f}' if carga_valida else 'n/d'),
                KeyValue(key='tensao_v', value=f'{tensao:.1f}'),
                KeyValue(key='temperatura_c', value=f'{temperatura:.0f}'),
            ]
            diag.status.append(status)

        # rclpy.ok(): o SIGINT invalida o contexto do rclpy ANTES de o executor
        # terminar o callback em curso. Sem esta guarda, o publish levantava
        # RCLError('publisher's context is invalid'), que escapava do main e
        # virava traceback com codigo de saida 1 a cada Ctrl+C.
        if js.name and rclpy.ok():
            self.joint_state_publisher.publish(js)
            self.diagnostics_publisher.publish(diag)

    # =================================================================
    # ENCERRAMENTO SEGURO
    # =================================================================

    def destroy_node(self):
        # Modo observador: o nó não ligou o torque, então também não o desliga —
        # sai deixando o barramento exatamente como encontrou.
        if not self.ligar_torque:
            if self.port_ok:
                try:
                    self.portHandler.closePort()
                except ERROS_SERIAL:
                    pass
            super().destroy_node()
            return

        # Só toca na serial se ela estiver viva (evita traceback no Ctrl+C
        # quando a porta nunca abriu ou caiu no meio da operação)
        if self.port_ok:
            try:
                for dxl_id in self.active_ids:
                    self.packetHandler.write1ByteTxRx(
                        self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.portHandler.closePort()
            except ERROS_SERIAL:
                self.get_logger().warn('Porta indisponivel no encerramento; torque nao desligado.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AX12HardwareInterface()
    except RuntimeError as e:
        # Porta não abriu nem com as tentativas: encerra limpo, sem traceback
        print(f'ERRO: {e}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass            # Ctrl+C
    except ExternalShutdownException:
        pass            # SIGTERM: como systemd, docker e o ros2 launch encerram
    except Exception:
        # Rede de segurança para a corrida de encerramento: o SIGINT invalida
        # o contexto do rclpy enquanto um callback ainda está no meio de um
        # publish, e o RCLError resultante escapava daqui virando traceback
        # com código de saída 1. As guardas `rclpy.ok()` nos publishes fecham
        # quase toda a janela, mas ela é inerentemente uma corrida.
        # O teste é preciso: contexto morto = encerrando, engole; contexto
        # vivo = erro de verdade, repassa. (RCLError só existe no módulo
        # privado _rclpy_pybind11, então não dá para capturá-lo pelo tipo
        # sem depender de API interna.)
        if rclpy.ok():
            raise
    finally:
        # destroy_node() é quem desliga o torque, então precisa rodar em
        # qualquer caminho de saída. Já o shutdown() pode ter acontecido
        # sozinho (é o que levanta ExternalShutdownException) — chamá-lo de
        # novo lançava RCLError e enterrava o traceback do encerramento.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
