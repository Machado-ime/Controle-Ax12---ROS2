# Arquitetura do sistema

Documentação técnica do pacote `ax12_control`. Para instalar e rodar, veja o [README](../README.md).

---

## Visão do sistema

Dois nós ROS 2 se comunicam via rede (DDS/Wi-Fi):

```
send_gait (PC)                           ax12_controller (Raspberry Pi)
──────────────────────────────           ──────────────────────────────
Lê *.yaml                                Único processo no barramento serial
Monta JointTrajectory                    Converte rad → unidades AX-12
Publica /joint_trajectory ──────────────▶ SyncWrite (4 bytes/motor)
                                         Lê telemetria (8 bytes/motor)
ax12_monitor (PC)                        Publica /joint_states
Exibe tabela ao vivo  ◀────────────────── Publica /diagnostics
                      ◀────────────────── Publica /hardware_errors
```

### Tópicos

| Tópico | Tipo | QoS | Direção |
|---|---|---|---|
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | BEST_EFFORT / KEEP_LAST / depth 1 | PC → Pi |
| `/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT (sensor) | Pi → PC |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RELIABLE | Pi → PC |
| `/hardware_errors` | `std_msgs/String` | RELIABLE | Pi → PC |

O QoS `BEST_EFFORT/depth=1` nos comandos é deliberado para Wi-Fi: um comando perdido é descartado, nunca reenviado atrasado (comando velho chegando fora de hora é pior que comando perdido).

---

## `ax12_controller.py` — interface de hardware

É o nó `ax12_hardware_interface`, o único processo que toca o barramento serial. Roda na Raspberry Pi.

### Fluxo de inicialização

1. **Abre a porta serial** com N tentativas (padrão 5, 2 s entre cada) — útil porque a USB pode demorar a enumerar no boot da Pi.
2. **Liga o torque** motor a motor, com 50 ms entre cada (escalona o pico de corrente). Confirma qual respondeu (`COMM_SUCCESS`) e publica no `/hardware_errors` quem não respondeu.
3. **Assina `/joint_trajectory`** e inicia o timer de telemetria.

### Escrita (comandos de posição)

Para cada mensagem recebida:
- Descarta mensagens malformadas (mais juntas do que posições) com aviso.
- Converte posição: `goal = (rad + 2,618) × 1023 / 5,236`, saturado em 0–1023.
- Converte velocidade: `vel = |rad/s| × 86,03`, saturado em 1–1023 (mínimo 1, pois 0 = velocidade máxima no AX-12).
- Envia posição + velocidade de todos os motores num **único pacote `GroupSyncWrite`** de 4 bytes (endereços 30–33 são contíguos na tabela de controle).

### Leitura (telemetria)

Timer a 5 Hz (configurável). Por motor, uma única transação de 8 bytes (registradores 36–43) traz:

| Bytes | Campo | Publicado em |
|---|---|---|
| 0–1 | Posição (0–1023 → ±2,618 rad) | `/joint_states.position` |
| 2–3 | Velocidade (0–1023 + bit de direção) | `/joint_states.velocity` |
| 4–5 | Present Load (10 bits + bit de direção, % do torque máx) | `/joint_states.effort` (N·m estimado) |
| 6 | Tensão (× 0,1 V) | `/diagnostics` |
| 7 | Temperatura (°C) | `/diagnostics` |

O "torque" é o **Present Load** — estimativa interna do AX-12 em % do torque máximo. O valor em N·m é aproximado a partir do stall torque nominal (~1,5 N·m a 12 V). É o mesmo sinal que dispara a proteção de overload.

### Mapa de juntas

Os nomes seguem a convenção do URDF (`adam.urdf`): `{lado}_{movimento}_{segmento}_{N}`. O sufixo N é o ID de projeto no URDF e **não** é o ID físico do motor no barramento.

| Junta | ID no barramento |
|---|---|
| `pd_picht_tornozelo_3` | 12 |
| `pe_picht_tornozelo_4` | 17 |
| `pd_roll_tornozelo_1`  | 13 |
| `pe_roll_tornozelo_2`  | 18 |
| `pd_picht_joelho_5`    | 11 |
| `pe_picht_joelho_6`    | 16 |
| `pd_picht_quadril_7`   | 10 |
| `pe_pich_quadril_8`    | 15 |
| `pd_roll_quadril_9`    | 9  |
| `pe_roll_quadril_10`   | 14 |

Para ativar uma junta nova (braços, pescoço): adicione ao `joint_map` em `ax12_controller.py` sem repetir ID. Se ela deve se mover na marcha, acrescente o nome em `nomes_juntas` e uma linha na `matriz_movimento` do YAML, na mesma posição.

### Tolerância a falhas

- USB cai em operação → reconecta a cada comando recebido (religa o torque). Após 10 falhas consecutivas, desativa a escrita com aviso fatal em `/hardware_errors`.
- Encerramento (`Ctrl+C`) → desliga o torque de todos os motores e fecha a porta.

### Parâmetros ROS disponíveis

```bash
ros2 run ax12_control ax12_controller --ros-args \
  -p device:=/dev/ttyUSB0 \     # porta serial (padrão /dev/ttyACM0)
  -p baudrate:=1000000 \        # baud (padrão 1 Mbps)
  -p tentativas_abertura:=5 \   # tentativas ao iniciar
  -p max_falhas_reconexao:=10 \ # desiste após N falhas
  -p velocidade_padrao:=100 \   # usado se a msg vier sem velocities
  -p taxa_leitura:=5.0          # Hz da telemetria (0 desliga)
```

---

## `send_gait.py` — gerador de marcha

Roda no PC de comando. A marcha não está hardcoded: vem de um arquivo `.yaml` lido em tempo de execução. Trocar ou ajustar o movimento não exige editar nem recompilar código.

### Componentes

**`resolver_caminho_matriz(nome)`** — transforma um nome simples (`cin_inve`) no caminho `<pasta do script>/cin_inve.yaml`. Um caminho com diretório é passado sem alteração (para arquivos externos ao pacote).

**`carregar_marcha(caminho)`** — lê e valida o YAML antes de qualquer comando ir ao robô. Aborta com mensagem de erro se:
- alguma chave obrigatória estiver faltando (`passo`, `pausa`, `nomes_juntas`, `matriz_movimento`),
- `passo ≤ 0` ou `pausa < 0`,
- nº de linhas da matriz ≠ nº de juntas,
- linhas com tamanhos diferentes entre si.

**`ConexaoRobo`** — encapsula o nó ROS. Cria o publisher de `/joint_trajectory`, assina `/hardware_errors` (alertas aparecem no terminal) e seta `falha_fatal = True` quando o controlador publica `FALHA FATAL`.

**`main()`**:
1. Carrega a marcha — erro aqui aborta antes de conectar ao robô.
2. Aguarda o `ax12_controller` aparecer na rede (`get_subscription_count() > 0`).
3. Em loop: lê a coluna atual da matriz, calcula `velocidade = |Δposição| / passo` para cada junta (todas chegam ao alvo ao mesmo tempo), publica e espera `passo + pausa` segundos **processando a rede** (alertas chegam durante a pausa).
4. Se o controlador publicar `FALHA FATAL`, a marcha para sozinha.

---

## `ax12_monitor.py` — painel de telemetria

Roda no PC de comando (em outro terminal). Exibe uma tabela atualizada 2×/s no terminal: ângulo, velocidade, torque (% e N·m estimado), tensão, temperatura e status de cada motor, mais os últimos alertas de `/hardware_errors`. Funciona por SSH, sem interface gráfica.

É a ferramenta para flagrar um overload em formação: a coluna de torque sobe e o status muda para `ATENCAO` antes de o motor entrar em proteção.

Para gráficos ao longo do tempo: **PlotJuggler** (`ros2 run plotjuggler plotjuggler` → aba *Streaming* → assine `/joint_states`) ou `rqt_plot /joint_states/position[0]`.

---

## Marchas (arquivos `.yaml`)

Cada arquivo descreve um ciclo completo. Mora em `src/ax12_control/ax12_control/` e é instalado junto com o pacote (`package_data` em `setup.py`). Formato:

```yaml
passo: 1.0      # duração de cada transição (segundos)
pausa: 0.5      # repouso extra após cada transição (segundos)

nomes_juntas:             # nomes do joint_map — ordem importa
  - pd_picht_tornozelo_3
  - pe_picht_tornozelo_4
  - ...

matriz_movimento:         # 1 linha por junta (mesma ordem)
  - [pos1, pos2, ...]     # 1 coluna por passo, em RADIANOS
  - ...
```

### Marchas disponíveis

| Arquivo | Juntas | Descrição |
|---|---|---|
| `otimizada.yaml` (padrão) | 6 — pitches de tornozelo, joelho e quadril | Ajustada manualmente. Os rolls de tornozelo (`pd_roll_tornozelo_1`/`pe_roll_tornozelo_2`) ficam de fora: recebem torque mas não são comandados. |
| `cin_inve.yaml` | 8 — inclui rolls de tornozelo | Gerada da cinemática inversa dos pés (`angulos.mat`). Rolls comandados em 0 rad (centro). |

### Selecionar a marcha

```bash
ros2 run ax12_control send_gait --ros-args -p matriz:=cin_inve
```

### Criar uma marcha nova

1. Copie um `.yaml` existente para `src/ax12_control/ax12_control/<nome>.yaml`.
2. Ajuste `nomes_juntas`, `matriz_movimento`, `passo` e `pausa`.
3. Recompile: `colcon build --packages-select ax12_control`.
4. Use com `-p matriz:=<nome>`.

> A pasta `src/matrizes-de-movimento/` guarda as mesmas matrizes como referência/origem, incluindo `otimizacao.h` — um header C de um protótipo antigo com 18 motores, não usado por este pacote.

---

## Visualização sem hardware (`visualizar_marcha.py`, `passo_slider.py`)

Para ver a marcha no RViz sem Raspberry Pi nem motores ligados:

```bash
ros2 launch ax12_control visualizar_marcha.launch.py matriz:=otimizada
```

Sobe três nós:

| Nó | Função |
|---|---|
| `robot_state_publisher` | TF a partir do URDF do pacote `adam` (lido automaticamente — não precisa passar `urdf:=`). |
| `visualizar_marcha` | Publica `/joint_states` direto do YAML da marcha, sem `ros2_control`. Juntas do URDF ausentes na matriz são publicadas em 0 rad. |
| `passo_slider` | Janela Qt com slider + botões ◀▶ que publica o índice da etapa em `/passo_marcha`. |

### Modos do `visualizar_marcha`

- **Manual** (`passo_s:=0.0`, padrão) — espera mensagens `Int32` em `/passo_marcha` (é o que o `passo_slider` envia). Também aceita comando direto, sem o slider: `ros2 topic pub --once /passo_marcha std_msgs/msg/Int32 "data: N"`.
- **Automático** (ex.: `passo_s:=0.5`) — timer avança a etapa sozinho a cada N segundos.

### `gait_bridge.py` — ponte para o `ros2_control` (Caso 2: MoveIt2/mock)

Liga o `send_gait` (publica em `/joint_trajectory`, QoS BEST_EFFORT) aos `JointTrajectoryController`s do pacote `adam` (`/perna_direita_controller/joint_trajectory` e `/perna_esquerda_controller/joint_trajectory`, QoS RELIABLE — exigido pelo controller). Sem o bridge os dois lados nunca se conectam, mesmo com os nomes de junta certos, porque o QoS é incompatível.

---

## Detalhes técnicos

| Item | Detalhe |
|---|---|
| Protocolo | Dynamixel Protocol 1.0 |
| Mensagem de comando | `trajectory_msgs/JointTrajectory` — posições em rad, velocidades em rad/s |
| Escrita | `GroupSyncWrite` em 4 bytes por motor (endereços 30–33: Goal Position + Moving Speed) |
| Leitura | `readTxRx` em 8 bytes por motor (endereços 36–43: posição, velocidade, carga, tensão, temperatura) |
| Conversão de posição | `goal = (rad + 2,618) × 1023 / 5,236` — faixa ±150° = ±2,618 rad |
| Conversão de velocidade | `vel = \|rad/s\| × 86,03` — mínimo 1 (0 = velocidade máxima no AX-12) |
| Taxa de telemetria | 5 Hz por padrão; 10 motores × 5 Hz = 50 transações/s (confortável a 1 Mbps) |
