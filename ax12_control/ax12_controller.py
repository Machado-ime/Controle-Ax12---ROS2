"""Interface de hardware ROS 2 para servomotores Dynamixel AX-12.

Este nó é o ÚNICO processo que toca o barramento serial dos motores.
Ele assina /joint_trajectory (posições em rad, velocidades em rad/s),
converte para as unidades do AX-12 e escreve tudo num único pacote
SyncWrite. Falhas de hardware são publicadas em /hardware_errors.

Referência da tabela de controle do AX-12:
https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

# pyserial: usado apenas para reconhecer a exceção lançada quando a USB cai
import serial

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

# =====================================================================
# Tabela de controle do AX-12 (Protocolo 1.0)
# =====================================================================
PROTOCOL_VERSION        = 1.0
ADDR_TORQUE_ENABLE      = 24   # 1 byte  (0 = desliga, 1 = liga)
ADDR_GOAL_POSITION      = 30   # 2 bytes (0 a 1023 = 0° a 300°)
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

# O AX-12 NÃO tem sensor de torque verdadeiro: o Present Load (end. 40)
# é a estimativa interna do esforço, em % do torque máximo. Convertemos
# para N·m usando o stall torque nominal (~1,5 N·m a 12 V) — aproximação.
TORQUE_MAX_NM           = 1.5

# =====================================================================
# Fatores de conversão (unidades do ROS <-> unidades do AX-12)
# =====================================================================
LIMITE_RAD          = 2.618                       # ±150° (curso útil do AX-12)
POS_POR_RAD         = 1023.0 / (2 * LIMITE_RAD)   # rad -> unidades de posição
UNIDADES_POR_RAD_S  = 86.03                       # rad/s -> unidades (1 un. = 0,111 rpm)


class AX12HardwareInterface(Node):

    def __init__(self):
        super().__init__('ax12_hardware_interface')

        # --- Parâmetros ROS (mude sem editar o código) ---
        # Ex.: ros2 run ax12_control ax12_controller --ros-args -p device:=/dev/ttyUSB0
        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('tentativas_abertura', 5)    # tentativas ao iniciar o nó
        self.declare_parameter('max_falhas_reconexao', 10)  # desiste após N reconexões falhas
        self.declare_parameter('velocidade_padrao', 100)    # usada se a msg vier sem velocities
        self.declare_parameter('taxa_leitura', 5.0)         # Hz da telemetria (0 desliga)

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        self.velocidade_padrao = self.get_parameter('velocidade_padrao').value
        self.max_falhas_reconexao = self.get_parameter('max_falhas_reconexao').value

        # Mapa das juntas (nome ROS -> ID do motor no barramento)
        # Convenção: {lado}_{segmento}_{movimento}_{N} onde N é sequencial 1-8.
        # O ID que vale é sempre o número à direita; NÃO confundir com o sufixo N.
        self.joint_map = {
            'PD_tornozelo_pitch_1': 12,
            'PE_tornozelo_pitch_2': 17,
            'PD_tornozelo_roll_3': 13,   # ativos, mas fora da marcha atual:
            'PE_tornozelo_roll_4': 18,   # recebem torque e seguram a posição
            'PD_joelho_pitch_5': 11,
            'PE_joelho_pitch_6': 16,
            'PD_quadril_pitch_7': 10,
            'PE_quadril_pitch_8': 15,
            # Juntas ainda sem ID no barramento atual (quadril roll, braços,
            # pescoço): adicione aqui quando forem ligadas — cuidado para
            # NÃO repetir um ID já usado acima (ID duplicado = dois nomes
            # comandando o mesmo motor físico).
        }
        self.active_ids = list(self.joint_map.values())

        # Limites de posição por junta (rad) medidos no hardware e convertidos via:
        #   rad = (grau_AX12 - 150) * pi/180
        # onde grau_AX12 é a posição na escala 0-300° do AX-12.
        # PD (direito) e PE (esquerdo) são espelhados: os limites de PE são
        # (lo, hi) direto da medição; PD recebe (-hi, -lo) para refletir a montagem.
        self.joint_limits = {
            'PD_tornozelo_pitch_1': (-1.4661,     0.5585),  # espelho de PE
            'PE_tornozelo_pitch_2': (-0.5585,     1.4661),  # (118°,234°) → (-32°,+84°)
            'PD_tornozelo_roll_3':  (-0.8727,     0.5934),  # espelho de PE
            'PE_tornozelo_roll_4':  (-0.5934,     0.8727),  # (116°,200°) → (-34°,+50°)
            'PD_joelho_pitch_5':    (0.0,         LIMITE_RAD),  # (150°,300°) → (0°,+150°)
            'PE_joelho_pitch_6':    (-LIMITE_RAD, 0.0),         # espelho de PD
            'PD_quadril_pitch_7':   (-LIMITE_RAD, LIMITE_RAD),  # sem medição ainda
            'PE_quadril_pitch_8':   (-LIMITE_RAD, LIMITE_RAD),
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
        self._falhas_leitura = {}

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

        # 2. Liga o torque dos motores mapeados
        self._ligar_torque()
        self.get_logger().info('Torque LIGADO. Pronto para receber e escrever comandos.')

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

        # 4. Timer da telemetria (taxa_leitura = 0 desliga a leitura)
        taxa = self.get_parameter('taxa_leitura').value
        if taxa > 0:
            self.create_timer(1.0 / taxa, self.ler_motores_callback)

    # =================================================================
    # FUNÇÕES DE APOIO (conexão, torque e avisos de erro)
    # =================================================================

    def _avisar_erro(self, texto):
        """Mostra o aviso no terminal E publica em /hardware_errors."""
        self.get_logger().warn(texto)
        msg = String()
        msg.data = texto
        self.error_publisher.publish(msg)

    def _abrir_porta(self):
        """Uma única tentativa de abrir a porta. Retorna True se conseguiu."""
        try:
            if self.portHandler.openPort() and self.portHandler.setBaudRate(self.baudrate):
                self.port_ok = True
                return True
        except (serial.SerialException, OSError):
            pass  # porta inexistente/ocupada: tratado como falha normal
        self.port_ok = False
        return False

    def _abrir_porta_com_tentativas(self):
        """Tenta abrir a porta N vezes antes de desistir (N = parâmetro ROS)."""
        tentativas = self.get_parameter('tentativas_abertura').value
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
            except (serial.SerialException, OSError):
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

        # --- Resumo final: tudo certo é info; motor faltando é erro publicado ---
        total = len(self.joint_map)
        if conectados == total:
            self.get_logger().info(f'Todos os {total} motores conectados.')
        else:
            self._avisar_erro(
                f'Apenas {conectados}/{total} motores responderam! '
                'Verifique cabo, energia e IDs dos ausentes.')

    def _porta_caiu(self, contexto):
        """Marca a porta como caída e avisa. A reconexão acontece no callback."""
        self.port_ok = False
        self._avisar_erro(
            f'PORTA SERIAL CAIU {contexto}! Tentando reconectar a cada comando recebido...')

    def _tentar_reconectar(self):
        """Tenta reerguer a conexão. Após max_falhas_reconexao, desiste de vez."""
        try:
            self.portHandler.closePort()
        except (serial.SerialException, OSError):
            pass  # a porta já estava morta; só queríamos liberar o descritor

        if self._abrir_porta():
            self.falhas_reconexao = 0
            self._avisar_erro('Porta serial RECONECTADA. Religando o torque dos motores.')
            # Se os motores perderam energia no evento, voltaram com torque OFF
            self._ligar_torque()
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

        # Porta caída: usa a chegada de cada comando como gatilho de reconexão
        if not self.port_ok:
            self._tentar_reconectar()
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

        for i, joint_name in enumerate(msg.joint_names):
            # Junta que este controlador não conhece: ignora
            if joint_name not in self.joint_map:
                continue
            dxl_id = self.joint_map[joint_name]

            # --- CLAMP POR JUNTA (limites mecânicos do URDF) ---
            low, high = self.joint_limits.get(joint_name, (-LIMITE_RAD, LIMITE_RAD))
            cmd_rad = point.positions[i]
            rads = max(low, min(high, cmd_rad))
            if abs(rads - cmd_rad) > 1e-4:
                self._avisar_erro(
                    f'{joint_name}: {cmd_rad:.3f} rad fora do limite '
                    f'[{low:.3f}, {high:.3f}] — clampado para {rads:.3f} rad.')

            # --- CONVERSÃO DE POSIÇÃO (rad -> 0 a 1023) ---
            goal_pos = round((rads + LIMITE_RAD) * POS_POR_RAD)
            goal_pos = max(0, min(1023, goal_pos))

            # --- CONVERSÃO DE VELOCIDADE (rad/s -> 1 a 1023) ---
            # Mínimo 1, porque 0 significa "velocidade máxima" no AX-12!
            if tem_velocidades:
                velocidade = round(abs(point.velocities[i]) * UNIDADES_POR_RAD_S)
            else:
                velocidade = self.velocidade_padrao
            velocidade = max(1, min(1023, velocidade))

            # --- EMPACOTA posição (2 bytes) + velocidade (2 bytes) juntas ---
            param = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos),
                     DXL_LOBYTE(velocidade), DXL_HIBYTE(velocidade)]
            if not self.groupSyncWrite.addParam(dxl_id, param):
                self._avisar_erro(f'addParam falhou para o motor ID {dxl_id} (ID repetido?).')

        # --- ENVIA TUDO num único pacote broadcast ---
        try:
            result = self.groupSyncWrite.txPacket()
        except (serial.SerialException, OSError):
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
            except (serial.SerialException, OSError):
                self._porta_caiu('durante a leitura de telemetria')
                return

            if result != COMM_SUCCESS:
                # Motor mudo nesta leitura: avisa só se virar rotina (~5 s)
                falhas = self._falhas_leitura.get(dxl_id, 0) + 1
                self._falhas_leitura[dxl_id] = falhas
                if falhas == 25:
                    self._avisar_erro(
                        f'Motor ID {dxl_id} ({joint_name}) sem responder '
                        f'a leitura ha {falhas} ciclos seguidos.')
                continue
            self._falhas_leitura[dxl_id] = 0

            # --- Conversões (inverso das fórmulas de escrita) ---
            pos_raw = DXL_MAKEWORD(dados[0], dados[1])
            vel_raw = DXL_MAKEWORD(dados[2], dados[3])
            carga_raw = DXL_MAKEWORD(dados[4], dados[5])
            tensao = dados[6] / 10.0        # ex.: 119 -> 11,9 V
            temperatura = float(dados[7])   # já vem em °C

            pos_rad = (pos_raw / POS_POR_RAD) - LIMITE_RAD

            # Velocidade e carga usam 10 bits + bit de direção (>=1024 = horário)
            vel_rad_s = (vel_raw & 0x3FF) / UNIDADES_POR_RAD_S
            if vel_raw >= 1024:
                vel_rad_s = -vel_rad_s

            carga_pct = (carga_raw & 0x3FF) / 10.23   # % do torque máximo
            if carga_raw >= 1024:
                carga_pct = -carga_pct
            torque_nm = carga_pct / 100.0 * TORQUE_MAX_NM  # estimativa!

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
            if error != 0:
                status.level = DiagnosticStatus.ERROR
                status.message = self.packetHandler.getRxPacketError(error)
            elif temperatura >= 65.0 or abs(carga_pct) >= 80.0:
                status.level = DiagnosticStatus.WARN
                status.message = 'Perto do limite (temperatura ou torque)'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'
            status.values = [
                KeyValue(key='angulo_graus', value=f'{math.degrees(pos_rad):.1f}'),
                KeyValue(key='torque_pct', value=f'{carga_pct:.1f}'),
                KeyValue(key='torque_nm_estimado', value=f'{torque_nm:.2f}'),
                KeyValue(key='tensao_v', value=f'{tensao:.1f}'),
                KeyValue(key='temperatura_c', value=f'{temperatura:.0f}'),
            ]
            diag.status.append(status)

        if js.name:
            self.joint_state_publisher.publish(js)
            self.diagnostics_publisher.publish(diag)

    # =================================================================
    # ENCERRAMENTO SEGURO
    # =================================================================

    def destroy_node(self):
        # Só toca na serial se ela estiver viva (evita traceback no Ctrl+C
        # quando a porta nunca abriu ou caiu no meio da operação)
        if self.port_ok:
            try:
                for dxl_id in self.active_ids:
                    self.packetHandler.write1ByteTxRx(
                        self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.portHandler.closePort()
            except (serial.SerialException, OSError):
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
