/*
 *  opencr_dxl_imu_bridge — ponte USB<->DXL + IMU como dispositivo Dynamixel
 *
 *  Substitui o exemplo oficial "usb_to_dxl" mantendo 100% do comportamento
 *  de ponte (o PC continua falando Dynamixel SDK direto com os motores) e
 *  ADICIONA o proprio OpenCR como um dispositivo no barramento, no padrao
 *  oficial do ROBOTIS OP3 (firmware "opencr_op3"): ID 200, tabela de
 *  controle com IMU (gyro/acc crus + roll/pitch/yaw filtrados), tensao,
 *  botoes, LEDs e chave de energia DXL.
 *
 *  Diferenca para o opencr_op3 oficial: aquele fala SO Protocolo 2.0
 *  (motores XM430); aqui o no local responde em PROTOCOLO 1.0, porque o
 *  barramento deste robo e AX-12/MX-28(1.0). O layout da tabela e as
 *  escalas sao os MESMOS do OP3:
 *    - Gyro_X/Y/Z: int16 cru do sensor (fator 2000/32800 -> graus/s)
 *    - Acc_X/Y/Z : int16 cru do sensor (fator 2/32768 -> g)
 *    - Roll/Pitch/Yaw: int16 = graus * 10 (0,1 grau por unidade)
 *
 *  Funcionamento da ponte: os bytes do USB sao analisados por uma maquina
 *  de estados do Protocolo 1.0. Pacote enderecado ao ID 200 (com checksum
 *  valido) e respondido localmente e NAO vai ao barramento; qualquer outro
 *  byte/pacote (incluindo broadcast 0xFE, pacotes 2.0 do Wizard e lixo) e
 *  repassado ao barramento exatamente como chegou — em caso de duvida
 *  (timeout ou header invalido) o buffer e descarregado cru, garantindo
 *  que a ponte NUNCA segura trafego que nao e dela.
 *
 *  Ao ligar: IMU.begin() calibra o giroscopio (~3 s). MANTENHA O ROBO
 *  PARADO nesse periodo. Recalibracao: escrever 1 no endereco 50.
 *
 *  Referencias oficiais (ROBOTIS-GIT/OpenCR):
 *    examples/10. Etc/usb_to_dxl            (ponte original)
 *    libraries/OP3/examples/opencr_op3      (tabela ID 200 do OP3)
 *    examples/09. IMU/*                     (classe cIMU)
 *  PC (OP3): ROBOTIS-OP3/open_cr_module (fatores de conversao) e
 *  op3_manager/config/OP3.robot (sensor | ID 200 | OPEN-CR).
 */

#include <DynamixelSDK.h>
#include <IMU.h>

#define CMD_PORT              Serial      // USB (PC)
#define DXL_PORT              Serial3     // barramento TTL
#define DXL_BAUD              1000000

#define DXL_LED_RX            BDPIN_LED_USER_1
#define DXL_LED_TX            BDPIN_LED_USER_2

#define DXL_TX_BUFFER_LENGTH  1024

// ===================================================================
// No local (estilo opencr_op3, Protocolo 1.0)
// ===================================================================
#define NODE_ID               200
#define NODE_MODEL_NUMBER     0x7400   // mesmo model number do OP3
#define NODE_FW_VERSION       0x01

// Instrucoes do Protocolo 1.0
#define INST_PING             0x01
#define INST_READ             0x02
#define INST_WRITE            0x03

// Bits de erro do status packet (Protocolo 1.0)
#define ERR_NONE              0x00
#define ERR_RANGE             0x08
#define ERR_INSTRUCTION       0x40

// Tabela de controle — MESMO layout do opencr_op3 (dxl_node_op3.h)
typedef struct
{
  uint16_t Model_Number;                  // 0
  uint8_t  Firmware_Version;              // 2
  uint8_t  ID;                            // 3
  uint8_t  Baud;                          // 4
  uint8_t  Return_Delay_Time;             // 5
  uint8_t  Dummy1[10];                    // 6
  uint8_t  Status_Return_Level;           // 16
  uint8_t  Dummy2[1];                     // 17
  int16_t  Roll_Offset;                   // 18
  int16_t  Pitch_Offset;                  // 20
  int16_t  Yaw_Offset;                    // 22
  uint8_t  Dynamixel_Power;               // 24  (RW: 0=desliga, 1=liga o rail DXL)
  uint8_t  LED;                           // 25  (RW: bits 0-3 = LEDs de usuario)
  uint16_t LED_RGB;                       // 26
  uint16_t Buzzer;                        // 28
  uint8_t  Button;                        // 30  (bit0=SW1, bit1=SW2)
  uint8_t  Voltage;                       // 31  (0,1 V por unidade, ex.: 119 = 11,9 V)
  int16_t  Gyro_X;                        // 32
  int16_t  Gyro_Y;                        // 34
  int16_t  Gyro_Z;                        // 36
  int16_t  Acc_X;                         // 38
  int16_t  Acc_Y;                         // 40
  int16_t  Acc_Z;                         // 42
  int16_t  Roll;                          // 44  (graus * 10)
  int16_t  Pitch;                         // 46  (graus * 10)
  int16_t  Yaw;                           // 48  (graus * 10)
  uint8_t  IMU_Control;                   // 50  (RW: !=0 recalibra o giroscopio)
} __attribute__((packed)) mem_tabela_t;

typedef union
{
  mem_tabela_t reg;
  uint8_t      bytes[sizeof(mem_tabela_t)];
} tabela_t;

static tabela_t tabela;
#define TABELA_TAM  ((uint16_t)sizeof(mem_tabela_t))

static cIMU IMU;

// ===================================================================
// Parser do Protocolo 1.0 (direcao USB -> DXL)
// ===================================================================
// Pacote 1.0: FF FF ID LEN INST PARAM... CHK  (LEN = n_params + 2)
#define PKT_BUF_TAM       300     // LEN max 255 -> pacote max 259 bytes
#define PKT_TIMEOUT_US    2000   // gap entre bytes de um pacote parcial

enum { ST_FF1, ST_FF2, ST_ID, ST_LEN, ST_BODY };

static uint8_t  pkt_buf[PKT_BUF_TAM];
static uint16_t pkt_idx = 0;
static uint8_t  pkt_state = ST_FF1;
static uint8_t  pkt_id = 0;
static uint8_t  pkt_len = 0;
static uint16_t pkt_restante = 0;
static uint32_t pkt_ultimo_byte_us = 0;

uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];

static int rx_led_count = 0;
static int tx_led_count = 0;
static uint32_t rx_led_update_time;
static uint32_t tx_led_update_time;
static uint32_t imu_update_time = 0;
static uint32_t lento_update_time = 0;


void setup()
{
  CMD_PORT.begin(115200);
  DXL_PORT.begin(DXL_BAUD);

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  pinMode(DXL_LED_RX, OUTPUT);
  pinMode(DXL_LED_TX, OUTPUT);
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
  pinMode(BDPIN_LED_USER_3, OUTPUT);
  pinMode(BDPIN_LED_USER_4, OUTPUT);

  digitalWrite(DXL_LED_TX, HIGH);   // LEDs de usuario sao ativos em LOW
  digitalWrite(DXL_LED_RX, HIGH);

  drv_dxl_tx_enable(FALSE);

  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);   // liga o rail dos motores

  // --- tabela de controle ---
  memset(tabela.bytes, 0, TABELA_TAM);
  tabela.reg.Model_Number        = NODE_MODEL_NUMBER;
  tabela.reg.Firmware_Version    = NODE_FW_VERSION;
  tabela.reg.ID                  = NODE_ID;
  tabela.reg.Baud                = 1;      // informativo (1 = 1 Mbps na escala AX)
  tabela.reg.Return_Delay_Time   = 0;
  tabela.reg.Status_Return_Level = 2;
  tabela.reg.Dynamixel_Power     = 1;

  // Calibra o giroscopio no boot (~3 s) — robo PARADO.
  IMU.begin();
}

void loop()
{
  update_usb_para_dxl();
  update_dxl_para_usb();
  update_led();
  update_imu();
  update_regs_lentos();

  // Segue o baudrate pedido pelo PC na CDC (comportamento do usb_to_dxl:
  // permite o Wizard escanear em outros baudrates atraves da ponte)
  if (CMD_PORT.getBaudRate() != DXL_PORT.getBaudRate())
  {
    DXL_PORT.begin(CMD_PORT.getBaudRate());
  }
}


// ===================================================================
// Ponte USB -> DXL com interceptacao do ID 200
// ===================================================================

void encaminhar_para_dxl(const uint8_t *dados, uint16_t tam)
{
  if (tam == 0) return;
  drv_dxl_tx_enable(TRUE);
  DXL_PORT.write(dados, tam);
  DXL_PORT.flush();
  drv_dxl_tx_enable(FALSE);
  tx_led_count = 3;
}

void parser_reset(void)
{
  pkt_idx = 0;
  pkt_state = ST_FF1;
}

// Descarrega o buffer parcial cru no barramento (header invalido/timeout):
// a ponte nunca retem bytes que nao formaram um pacote 1.0 valido.
void parser_flush(void)
{
  encaminhar_para_dxl(pkt_buf, pkt_idx);
  parser_reset();
}

void update_usb_para_dxl(void)
{
  while (CMD_PORT.available() > 0)
  {
    uint8_t b = (uint8_t)CMD_PORT.read();
    pkt_ultimo_byte_us = micros();

    switch (pkt_state)
    {
      case ST_FF1:
        if (b == 0xFF) { pkt_buf[pkt_idx++] = b; pkt_state = ST_FF2; }
        else           { encaminhar_para_dxl(&b, 1); }
        break;

      case ST_FF2:
        if (b == 0xFF) { pkt_buf[pkt_idx++] = b; pkt_state = ST_ID; }
        else           { pkt_buf[pkt_idx++] = b; parser_flush(); }
        break;

      case ST_ID:
        if (b == 0xFF)
        {
          // FF FF FF...: encaminha um FF e mantem a janela FF FF
          uint8_t ff = 0xFF;
          encaminhar_para_dxl(&ff, 1);
        }
        else
        {
          pkt_id = b;
          pkt_buf[pkt_idx++] = b;
          pkt_state = ST_LEN;
        }
        break;

      case ST_LEN:
        // LEN valido no 1.0: 2..253. Fora disso (ex.: header 2.0
        // "FF FF FD 00") nao e pacote 1.0 -> descarrega cru.
        if (b < 2)
        {
          pkt_buf[pkt_idx++] = b;
          parser_flush();
        }
        else
        {
          pkt_len = b;
          pkt_buf[pkt_idx++] = b;
          pkt_restante = b;          // INST + params + checksum
          pkt_state = ST_BODY;
        }
        break;

      case ST_BODY:
        pkt_buf[pkt_idx++] = b;
        if (--pkt_restante == 0)
        {
          despachar_pacote();
          parser_reset();
        }
        break;
    }

    if (pkt_idx >= PKT_BUF_TAM - 1)
    {
      parser_flush();   // nunca deveria acontecer; protecao de overflow
    }
  }

  // Pacote parcial parado ha muito tempo: nao era um pacote — descarrega.
  if (pkt_idx > 0 && (micros() - pkt_ultimo_byte_us) > PKT_TIMEOUT_US)
  {
    parser_flush();
  }
}

void despachar_pacote(void)
{
  // So consome o pacote se for para o ID 200 E o checksum fechar;
  // qualquer outra coisa (inclusive broadcast 0xFE) vai para o barramento.
  if (pkt_id != NODE_ID)
  {
    encaminhar_para_dxl(pkt_buf, pkt_idx);
    return;
  }

  uint8_t chk = 0;
  for (uint16_t i = 2; i < pkt_idx - 1; i++) chk += pkt_buf[i];
  chk = ~chk;
  if (chk != pkt_buf[pkt_idx - 1])
  {
    encaminhar_para_dxl(pkt_buf, pkt_idx);   // corrompido: nao e nosso
    return;
  }

  uint8_t inst = pkt_buf[4];
  const uint8_t *params = &pkt_buf[5];
  uint8_t n_params = pkt_len - 2;

  switch (inst)
  {
    case INST_PING:
      responder_status(ERR_NONE, NULL, 0);
      break;

    case INST_READ:
      if (n_params < 2 || params[0] + params[1] > TABELA_TAM)
      {
        responder_status(ERR_RANGE, NULL, 0);
      }
      else
      {
        responder_status(ERR_NONE, &tabela.bytes[params[0]], params[1]);
      }
      break;

    case INST_WRITE:
      if (n_params < 2)
      {
        responder_status(ERR_RANGE, NULL, 0);
      }
      else
      {
        responder_status(aplicar_escrita(params[0], &params[1], n_params - 1),
                         NULL, 0);
      }
      break;

    default:
      responder_status(ERR_INSTRUCTION, NULL, 0);
      break;
  }

  rx_led_count = 3;
}

uint8_t aplicar_escrita(uint8_t addr, const uint8_t *dados, uint8_t tam)
{
  if ((uint16_t)addr + tam > TABELA_TAM) return ERR_RANGE;

  for (uint8_t i = 0; i < tam; i++)
  {
    uint8_t a = addr + i;
    uint8_t v = dados[i];

    switch (a)
    {
      case 24:   // Dynamixel_Power: liga/desliga o rail 12V dos motores
        tabela.reg.Dynamixel_Power = (v != 0);
        digitalWrite(BDPIN_DXL_PWR_EN, (v != 0) ? HIGH : LOW);
        break;

      case 25:   // LED: bits 0-3 -> LEDs de usuario (ativos em LOW)
        tabela.reg.LED = v;
        digitalWrite(BDPIN_LED_USER_1, (v & 0x01) ? LOW : HIGH);
        digitalWrite(BDPIN_LED_USER_2, (v & 0x02) ? LOW : HIGH);
        digitalWrite(BDPIN_LED_USER_3, (v & 0x04) ? LOW : HIGH);
        digitalWrite(BDPIN_LED_USER_4, (v & 0x08) ? LOW : HIGH);
        break;

      case 50:   // IMU_Control: qualquer valor != 0 recalibra o giroscopio
        if (v != 0)
        {
          tabela.reg.IMU_Control = v;
          IMU.SEN.gyro_cali_start();
        }
        break;

      default:   // demais enderecos sao somente leitura
        return ERR_RANGE;
    }
  }
  return ERR_NONE;
}

void responder_status(uint8_t erro, const uint8_t *dados, uint8_t tam)
{
  uint8_t resp[6 + 255];
  uint8_t len = tam + 2;
  uint8_t chk = 0;

  resp[0] = 0xFF;
  resp[1] = 0xFF;
  resp[2] = NODE_ID;
  resp[3] = len;
  resp[4] = erro;
  for (uint8_t i = 0; i < tam; i++) resp[5 + i] = dados[i];

  for (uint16_t i = 2; i < (uint16_t)(5 + tam); i++) chk += resp[i];
  resp[5 + tam] = ~chk;

  CMD_PORT.write(resp, 6 + tam);
}


// ===================================================================
// Ponte DXL -> USB (identica ao usb_to_dxl original)
// ===================================================================

void update_dxl_para_usb(void)
{
  int length = DXL_PORT.available();
  if (length > 0)
  {
    if (length > DXL_TX_BUFFER_LENGTH) length = DXL_TX_BUFFER_LENGTH;
    for (int i = 0; i < length; i++)
    {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);
    rx_led_count = 3;
  }
}


// ===================================================================
// Atualizacao dos registradores (IMU rapido; tensao/botao lentos)
// ===================================================================

void update_imu(void)
{
  // Prioriza a ponte: so atualiza o IMU quando o USB esta ocioso, mas
  // garante pelo menos uma atualizacao a cada 20 ms.
  if (CMD_PORT.available() > 0 && (millis() - imu_update_time) < 20) return;
  if ((millis() - imu_update_time) < 2) return;
  imu_update_time = millis();

  IMU.update();

  tabela.reg.Gyro_X = IMU.SEN.gyroADC[0];
  tabela.reg.Gyro_Y = IMU.SEN.gyroADC[1];
  tabela.reg.Gyro_Z = IMU.SEN.gyroADC[2];
  tabela.reg.Acc_X  = IMU.SEN.accRAW[0];
  tabela.reg.Acc_Y  = IMU.SEN.accRAW[1];
  tabela.reg.Acc_Z  = IMU.SEN.accRAW[2];
  tabela.reg.Roll   = (int16_t)(IMU.rpy[0] * 10.0);
  tabela.reg.Pitch  = (int16_t)(IMU.rpy[1] * 10.0);
  tabela.reg.Yaw    = (int16_t)(IMU.rpy[2] * 10.0);

  // Calibracao pedida via end. 50: limpa o flag quando terminar
  if (tabela.reg.IMU_Control != 0 && IMU.SEN.gyro_cali_get_done() == true)
  {
    tabela.reg.IMU_Control = 0;
  }
}

void update_regs_lentos(void)
{
  if ((millis() - lento_update_time) < 50) return;
  lento_update_time = millis();

  tabela.reg.Voltage = (uint8_t)(getPowerInVoltage() * 10.0);
  tabela.reg.Button  = (digitalRead(BDPIN_PUSH_SW_1) ? 0x01 : 0x00)
                     | (digitalRead(BDPIN_PUSH_SW_2) ? 0x02 : 0x00);
}


// ===================================================================
// LEDs de atividade (identico ao usb_to_dxl original)
// ===================================================================

void update_led(void)
{
  if ((millis() - tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();
    if (tx_led_count)
    {
      digitalWrite(DXL_LED_TX, !digitalRead(DXL_LED_TX));
      tx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_TX, HIGH);
    }
  }

  if ((millis() - rx_led_update_time) > 50)
  {
    rx_led_update_time = millis();
    if (rx_led_count)
    {
      digitalWrite(DXL_LED_RX, !digitalRead(DXL_LED_RX));
      rx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_RX, HIGH);
    }
  }
}
