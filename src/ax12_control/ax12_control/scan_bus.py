"""Varredura SOMENTE-LEITURA do barramento Dynamixel (ferramenta de bancada).

Responde três perguntas antes de qualquer nó ROS subir:

  1. A porta abre, e o modo de baixa latência funciona nela?
  2. Quais IDs existem de fato no barramento, e em qual baudrate?
  3. Qual firmware está na OpenCR — usb_to_dxl (ponte burra) ou o estilo OP3
     (que responde no ID 200 e expõe o IMU)?

Sem isso, um nó que não acha motores deixa você sem saber se o problema é o
código ou o barramento. Aqui não há ROS no meio: é pyserial + Dynamixel SDK.

NUNCA ESCREVE NADA. Só instruções READ, que não alteram registrador nenhum,
não ligam torque e não movem motor. Pode rodar com o robô montado.

Uso:
    ros2 run ax12_control scan_bus
    ros2 run ax12_control scan_bus --device /dev/ttyUSB0
    ros2 run ax12_control scan_bus --ids 1-20 --baudrates 1000000

Ou direto, sem o ROS instalado:
    python3 scan_bus.py --device /dev/ttyACM0
"""

import argparse
import sys
import time

import serial

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# termios.error NÃO herda de OSError, e é o que sobe quando a USB é arrancada
# no meio de uma transação (o SDK faz tcdrain antes de cada pacote). Mesma
# armadilha que já derrubou o ax12_controller — ver ERROS_SERIAL lá.
try:
    import termios
    _ERROS_TERMIOS = (termios.error,)
except ImportError:
    _ERROS_TERMIOS = ()
ERROS_SERIAL = (serial.SerialException, OSError) + _ERROS_TERMIOS

PROTOCOL_VERSION = 1.0

ADDR_MODEL_NUMBER = 0    # 2 bytes, presente em TODO Dynamixel
ADDR_TELEMETRIA = 36     # pos(2) vel(2) carga(2) tensao(1) temp(1)
LEN_TELEMETRIA = 8

# OpenCR com firmware estilo OP3 responde aqui; com usb_to_dxl, não existe
OPENCR_ID = 200
ADDR_OPENCR_BLOCO = 30
LEN_OPENCR_BLOCO = 20

# Número do modelo -> nome. Fonte: tabelas de controle da ROBOTIS.
MODELOS = {
    12: 'AX-12A', 18: 'AX-18A', 300: 'AX-12W',
    29: 'MX-28', 310: 'MX-64', 320: 'MX-106',
    350: 'XL-320', 1020: 'XM430-W350', 1030: 'XM430-W210',
}

# 1 Mbps primeiro: é o do projeto. Os outros pegam um motor que tenha sido
# regravado com baudrate diferente e ficado "invisível".
BAUDRATES_PADRAO = [1000000, 57600, 115200, 500000, 2000000, 9600]

LIMITE_RAD = 2.618
POS_POR_RAD = 1023 / (2 * LIMITE_RAD)


def faixa_de_ids(texto):
    """Converte '1-20' ou '1,2,9' numa lista de IDs."""
    ids = []
    for parte in texto.split(','):
        parte = parte.strip()
        if '-' in parte:
            ini, fim = parte.split('-')
            ids.extend(range(int(ini), int(fim) + 1))
        else:
            ids.append(int(parte))
    # 254 é broadcast e 253+ não são endereçáveis: o SDK recusa de saída
    return [i for i in ids if 0 <= i < 253]


def abrir(device, baudrate, pausa):
    """Abre a porta com os mesmos cuidados do ax12_controller.

    Devolve (portHandler, baixa_latencia_ok) ou (None, None) se não abriu.
    """
    port = PortHandler(device)
    try:
        # setBaudRate() já abre a porta; chamar openPort() antes só provocaria
        # um ciclo abre-fecha-abre extra num dispositivo CDC ACM.
        if not port.setBaudRate(baudrate):
            return None, None

        try:
            port.ser.set_low_latency_mode(True)
            baixa_latencia = True
        except (OSError, ValueError, AttributeError):
            baixa_latencia = False

        # A OpenCR com usb_to_dxl faz o barramento SEGUIR o baud do USB, mas só
        # reconfigura a Serial3 na iteração seguinte do loop() dela. Sem esta
        # pausa, os primeiros pacotes saem no baud errado e ninguém responde.
        time.sleep(pausa)
        port.ser.reset_input_buffer()
        return port, baixa_latencia
    except ERROS_SERIAL as e:
        print(f'  ERRO ao abrir {device} @ {baudrate}: {e}')
        return None, None


def varrer(port, packet, ids):
    """Pergunta o Model Number de cada ID. Devolve [(id, nome_do_modelo), ...]."""
    achados = []
    for dxl_id in ids:
        try:
            dados, result, _ = packet.readTxRx(port, dxl_id, ADDR_MODEL_NUMBER, 2)
        except ERROS_SERIAL as e:
            print(f'  PORTA CAIU durante a varredura (ID {dxl_id}): {e}')
            return achados, False
        if result == COMM_SUCCESS:
            num = dados[0] | (dados[1] << 8)
            achados.append((dxl_id, MODELOS.get(num, f'desconhecido (model={num})')))
    return achados, True


def detalhar(port, packet, dxl_id):
    """Lê o bloco de telemetria de um motor encontrado. Só leitura."""
    try:
        dados, result, error = packet.readTxRx(port, dxl_id, ADDR_TELEMETRIA, LEN_TELEMETRIA)
    except ERROS_SERIAL:
        return None
    if result != COMM_SUCCESS:
        return None
    pos_raw = dados[0] | (dados[1] << 8)
    return {
        'pos_unidades': pos_raw,
        'pos_graus': (pos_raw / POS_POR_RAD - LIMITE_RAD) * 57.2957795,
        'tensao_v': dados[6] / 10.0,
        'temp_c': dados[7],
        'flag_erro': error,
    }


def testar_opencr(port, packet):
    """O ID 200 responde? Isso identifica o firmware da placa."""
    try:
        _, result, _ = packet.readTxRx(port, OPENCR_ID, ADDR_OPENCR_BLOCO, LEN_OPENCR_BLOCO)
    except ERROS_SERIAL:
        return None
    return result == COMM_SUCCESS


def main(args=None):
    ap = argparse.ArgumentParser(
        description='Varredura somente-leitura do barramento Dynamixel.')
    ap.add_argument('--device', default='/dev/ttyACM0')
    ap.add_argument('--ids', default='0-30',
                    help="IDs a testar, ex.: '0-30' ou '1,2,9' (padrao: 0-30)")
    ap.add_argument('--baudrates', default=None,
                    help='lista separada por virgula; padrao testa os comuns')
    ap.add_argument('--pausa', type=float, default=0.2,
                    help='segundos entre abrir a porta e o 1o pacote (OpenCR)')
    # ros2 run injeta --ros-args; ignoramos o que não é nosso
    opts, _ = ap.parse_known_args(args if args is not None else sys.argv[1:])

    ids = faixa_de_ids(opts.ids)
    bauds = ([int(b) for b in opts.baudrates.split(',')]
             if opts.baudrates else BAUDRATES_PADRAO)

    print('=' * 68)
    print(f'VARREDURA DO BARRAMENTO — {opts.device}')
    print(f'IDs {opts.ids} | baudrates: {", ".join(str(b) for b in bauds)}')
    print('Somente leitura: nada e escrito, nenhum torque e ligado.')
    print('=' * 68)

    packet = PacketHandler(PROTOCOL_VERSION)
    total_achados = 0
    latencia_relatada = False
    abriu_alguma_vez = False   # separa "porta morta" de "barramento vazio"

    for baud in bauds:
        port, baixa_latencia = abrir(opts.device, baud, opts.pausa)
        if port is None:
            print(f'\n[baud {baud}] nao foi possivel abrir a porta.')
            continue
        abriu_alguma_vez = True

        # Só vale relatar uma vez: é propriedade da porta, não do baudrate.
        if not latencia_relatada:
            latencia_relatada = True
            print(f'\nPorta aberta. Modo de baixa latencia: '
                  f'{"ATIVO" if baixa_latencia else "INDISPONIVEL"}')
            if baixa_latencia:
                print('  -> o ax12_controller vai usar orcamento de 4 ms por pacote.')
            else:
                print('  -> o ax12_controller vai manter 16 ms por pacote (mais lento')
                print('     com barramento ruim, mas resposta boa nao expira).')

        achados, ok = varrer(port, packet, ids)
        print(f'\n[baud {baud}] {len(achados)} dispositivo(s)')
        for dxl_id, modelo in achados:
            info = detalhar(port, packet, dxl_id)
            if info:
                erro = f" ERRO=0x{info['flag_erro']:02x}" if info['flag_erro'] else ''
                print(f'    ID {dxl_id:>3}  {modelo:<22} '
                      f"pos={info['pos_unidades']:>4} ({info['pos_graus']:+6.1f}deg)  "
                      f"{info['tensao_v']:.1f}V  {info['temp_c']:>2}C{erro}")
            else:
                print(f'    ID {dxl_id:>3}  {modelo:<22} (telemetria nao respondeu)')
        total_achados += len(achados)

        if ok and baud == bauds[0]:
            resp = testar_opencr(port, packet)
            print(f'\n  OpenCR no ID {OPENCR_ID}: '
                  f'{"RESPONDE" if resp else "mudo"}')
            if resp:
                print('    -> firmware estilo OP3: o IMU esta disponivel.')
                print('       Pode usar o ax12_controller com -p taxa_imu:=50.0')
            else:
                print('    -> firmware usb_to_dxl (ponte USB-serial pura), ou placa')
                print('       ausente. O ID 200 NUNCA responde com esse firmware, por')
                print('       construcao — mantenha taxa_imu:=0.0 (o padrao).')

        try:
            port.closePort()
        except ERROS_SERIAL:
            pass

    print('\n' + '=' * 68)
    if not abriu_alguma_vez:
        # Caso MUITO diferente do barramento vazio: aqui nem o adaptador
        # apareceu, entao falar de fonte 12 V ou cabo de motor seria mandar
        # o usuario procurar no lugar errado.
        print(f'A PORTA {opts.device} NAO ABRIU EM NENHUM BAUDRATE.')
        print()
        print('O problema esta antes do barramento — nada foi perguntado a')
        print('motor nenhum. Verifique:')
        print('  1. o dispositivo existe?   ls /dev/ttyACM* /dev/ttyUSB*')
        print('  2. no WSL, foi anexado?    usbipd attach --wsl --busid <BUSID>')
        print('  3. outro processo esta com a porta?  (o ax12_controller rodando?)')
        print('  4. permissao:  sudo usermod -aG dialout $USER  (requer relogin)')
    elif total_achados == 0:
        print('NENHUM dispositivo respondeu em nenhum baudrate.')
        print()
        print('A porta ABRIU, entao o adaptador USB esta vivo — o que falta esta')
        print('depois dele. Quase sempre nesta ordem:')
        print('  1. fonte de 12 V ligada no jack da placa (o USB alimenta so a logica)')
        print('  2. chave de power da placa ligada')
        print('  3. cabo de 3 pinos da placa ate o primeiro motor')
        print('  4. corrente de cabos entre motores (um solto derruba os seguintes)')
    else:
        print(f'{total_achados} dispositivo(s) no total.')
        print('Compare os IDs acima com o joint_map do ax12_controller:')
        print('  ID 1=pd_roll_tornozelo_1   ID 2=pe_roll_tornozelo_2')
        print('  ID 3=pd_picht_tornozelo_3  ID 4=pe_picht_tornozelo_4')
        print('  ID 5=pd_picht_joelho_5     ID 6=pe_picht_joelho_6')
        print('  ID 7=pd_picht_quadril_7    ID 8=pe_pich_quadril_8')
        print('  ID 9=pd_roll_quadril_9     ID 10=pe_roll_quadril_10')
    print('=' * 68)


if __name__ == '__main__':
    main()
