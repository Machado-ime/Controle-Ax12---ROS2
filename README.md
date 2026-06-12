# Controle AX-12 — ROS 2

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12%2B-yellow)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Pacote ROS 2 (`ax12_control`) para controle dos servomotores **Dynamixel AX-12** de um robô bípede. O sistema é distribuído em duas máquinas que se comunicam pela rede (DDS/Wi-Fi):

- **Raspberry Pi** (conectada aos motores via USB): roda o nó `ax12_controller`, que recebe trajetórias e escreve nos motores.
- **PC de comando**: roda o `send_gait`, que publica a sequência de marcha.

```
   PC de comando                              Raspberry Pi
┌─────────────────┐    /joint_trajectory   ┌──────────────────┐    USB/serial     ┌──────────────┐
│    send_gait    │ ─────────────────────▶ │  ax12_controller │ ────────────────▶ │ Motores AX-12│
└─────────────────┘   (JointTrajectory,    └──────────────────┘  1 Mbps, Proto 1.0└──────────────┘
                       QoS BEST_EFFORT,
                          depth = 1)
```

## Sumário

- [Estrutura do repositório](#estrutura-do-repositório)
- [Pré-requisitos](#pré-requisitos)
- [Instalação](#instalação)
- [Como rodar](#como-rodar)
- [Os códigos explicados](#os-códigos-explicados)
- [Detalhes técnicos](#detalhes-técnicos)
- [Solução de problemas](#solução-de-problemas)
- [Licença](#licença)

## Estrutura do repositório

```
Controle-Ax12---ROS2/
├── package.xml              # Manifesto ROS 2 (nome do pacote e dependências)
├── setup.py                 # Instalador: define os executáveis do ros2 run
├── setup.cfg                # Destino dos executáveis no install/
├── resource/
│   └── ax12_control         # Marcador do ament index (não editar)
├── ax12_control/            # Código-fonte (módulo Python)
│   ├── __init__.py
│   ├── ax12_controller.py   # Nó de interface de hardware (roda na Raspberry Pi)
│   └── send_gait.py         # Cliente de marcha (roda no PC de comando)
├── docs/
│   └── bizuario_ros.md      # Cola de comandos úteis do ROS 2 para diagnóstico
├── legacy/                  # Versões antigas (referência histórica, fora do build)
│   ├── controller_antigo.py # Controlador com leitura de posição e tópico de erros
│   └── send_antigo.py       # Cliente antigo com listas separadas por articulação
├── LICENSE
└── README.md
```

## Pré-requisitos

| Requisito | Versão | Onde baixar / instalar |
|---|---|---|
| Ubuntu | 24.04 (Noble) | [releases.ubuntu.com/24.04](https://releases.ubuntu.com/24.04/) |
| ROS 2 | Jazzy | [Guia oficial de instalação](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| colcon | — | `sudo apt install python3-colcon-common-extensions` |
| Dynamixel SDK | 3.x | [Manual da ROBOTIS](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) — comando abaixo |
| git | — | `sudo apt install git` |

**Instalação resumida dos pré-requisitos** (após instalar o Ubuntu 24.04):

```bash
# 1. ROS 2 Jazzy (siga o guia oficial; resumo dos comandos principais)
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# Adiciona o repositório apt oficial do ROS 2 (método atual, via ros2-apt-source)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install -y ros-jazzy-ros-base

# 2. Ferramenta de build
sudo apt install -y python3-colcon-common-extensions

# 3. Dynamixel SDK (necessário apenas na Raspberry Pi, que fala com os motores)
sudo apt install -y ros-jazzy-dynamixel-sdk
# alternativa, caso o pacote apt não exista para sua plataforma:
# pip3 install dynamixel-sdk
```

> [!NOTE]
> Os passos acima devem ser feitos **nas duas máquinas** (Raspberry Pi e PC de comando), exceto o Dynamixel SDK, que só é obrigatório na Raspberry Pi.

## Instalação

Os comandos abaixo criam o workspace `ax12_control_ws`, clonam o repositório, compilam o pacote e deixam o ambiente configurado **permanentemente** (via `~/.bashrc`):

```bash
# 1. Cria o workspace e clona o repositório dentro de src/
mkdir -p ~/ax12_control_ws/src
cd ~/ax12_control_ws/src
git clone https://github.com/Machado-ime/Controle-Ax12---ROS2.git

# 2. Compila o pacote
cd ~/ax12_control_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ax12_control

# 3. Torna o ambiente permanente (ROS 2 + workspace em todo terminal novo)
echo "source /opt/ros/jazzy/setup.bash"              >> ~/.bashrc
echo "source ~/ax12_control_ws/install/setup.bash"   >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0"                        >> ~/.bashrc
source ~/.bashrc
```

> [!IMPORTANT]
> O `ROS_DOMAIN_ID` precisa ser **o mesmo** nas duas máquinas para que elas se enxerguem na rede. O valor `0` é o padrão; se houver outros robôs/ROS na mesma rede, escolha outro número (0–101) e use-o nas duas máquinas.

Para verificar a instalação:

```bash
ros2 pkg executables ax12_control
# saída esperada:
# ax12_control ax12_controller
# ax12_control send_gait
```

## Como rodar

**1. Na Raspberry Pi** (motores ligados na porta `/dev/ttyACM0` e fonte de alimentação conectada):

```bash
ros2 run ax12_control ax12_controller
```

Saída esperada: `Porta aberta com sucesso!` seguido de `Torque LIGADO`. O nó fica aguardando trajetórias.

**2. No PC de comando:**

```bash
ros2 run ax12_control send_gait
```

O robô começa a executar o ciclo de marcha em loop. `Ctrl+C` em qualquer um dos lados encerra com segurança (o controlador desliga o torque dos motores ao sair).

## Os códigos explicados

### `ax12_control/ax12_controller.py` — interface de hardware

É o nó ROS 2 `ax12_hardware_interface`, o único processo que toca o barramento serial dos motores. O fluxo dele:

1. **Mapa de juntas** (`joint_map`): associa o nome de cada junta ao ID do motor no barramento. Atualmente 8 juntas ativas — 6 comandadas pela marcha e 2 rolls de tornozelo travados (ver tabela abaixo).
2. **Abertura da serial com tentativas**: porta e baudrate são **parâmetros ROS** (padrão `/dev/ttyACM0`, 1 Mbps); o nó tenta abrir 5 vezes antes de desistir (útil quando a USB demora a enumerar no boot da Pi).
3. **Liga o torque** de cada motor com pausa de 50 ms entre eles, conferindo se cada um respondeu (`COMM_SUCCESS`).
4. **Assina `/joint_trajectory`** (`trajectory_msgs/JointTrajectory`) e, a cada mensagem:
   - valida a mensagem (malformada = descartada com aviso);
   - converte posição de radianos (±2,618 rad = ±150°) para a escala 0–1023 do AX-12;
   - converte velocidade de rad/s para a escala 1–1023 (fator 86,03);
   - envia **posição + velocidade de todos os motores num único pacote** `GroupSyncWrite` de 4 bytes (endereços 30–33 são contíguos na tabela de controle do AX-12).
5. **Telemetria** (timer, padrão 5 Hz): lê de cada motor, numa única transação de 8 bytes (registradores 36–43), posição, velocidade, **torque** (Present Load), tensão e temperatura. Publica em `/joint_states` (`sensor_msgs/JointState`: rad, rad/s, N·m estimado) e `/diagnostics` (`diagnostic_msgs/DiagnosticArray`: torque %, tensão, temperatura, nível OK/WARN/ERROR). Flags de erro do motor (ex.: **overload**) viram alerta imediato em `/hardware_errors` — fecha o ponto cego de erros durante o movimento.
6. **Tolerância a falhas**: se a USB cair em operação, o nó tenta reconectar a cada comando recebido (religando o torque) e, após 10 falhas, desativa a escrita com aviso fatal. Todos os eventos são publicados no tópico **`/hardware_errors`** (`std_msgs/String`, QoS RELIABLE) — monitore com `ros2 topic echo /hardware_errors`.
7. **No encerramento** (`Ctrl+C`): desliga o torque de todos os motores e fecha a porta.

Parâmetros disponíveis (`--ros-args -p nome:=valor`): `device`, `baudrate`, `tentativas_abertura`, `max_falhas_reconexao`, `velocidade_padrao`, `taxa_leitura` (Hz da telemetria; `0` desliga a leitura). Exemplo:

```bash
ros2 run ax12_control ax12_controller --ros-args -p device:=/dev/ttyUSB0
```

| Junta                  | ID do motor | Comandada pela marcha? |
|------------------------|-------------|------------------------|
| `PD_tornozelo_pitch_1` | 18          | Sim |
| `PE_tornozelo_pitch_2` | 13          | Sim |
| `PD_tornozelo_roll_3`  | 17          | Não — torque ligado, posição travada |
| `PE_tornozelo_roll_4`  | 12          | Não — torque ligado, posição travada |
| `PD_joelho_pitch_5`    | 16          | Sim |
| `PE_joelho_pitch_6`    | 11          | Sim |
| `PD_quadril_pitch_7`   | 15          | Sim |
| `PE_quadril_pitch_8`   | 10          | Sim |

> [!WARNING]
> O sufixo numérico do **nome** da junta é histórico e **não corresponde ao ID real** do motor (ex.: `PD_tornozelo_pitch_1` é o motor de ID **18**). O ID que vale é o da tabela acima / do `joint_map`.

As demais juntas (quadril roll, braços e pescoço) ainda não têm ID no barramento atual. Para ativar uma junta nova: adicione-a ao `joint_map` do `ax12_controller.py` (sem repetir ID!) e, se ela deve se mover na marcha, acrescente o nome em `nomes_juntas` e uma linha na `matriz_movimento` do `send_gait.py`, na mesma ordem.

### `ax12_control/send_gait.py` — gerador de marcha

Script cliente que publica a sequência de passos. Dois blocos:

- **`ConexaoRobo`**: encapsula o nó ROS — cria o publisher de `/joint_trajectory` com o mesmo QoS do controlador, **assina `/hardware_errors`** (alertas do controlador aparecem no terminal) e expõe `enviar_passo(...)`, `aguardar_controlador()` e `esperar(...)`.
- **`main()`**: contém a **matriz de movimento**, onde cada **linha é uma junta** (na mesma ordem de `nomes_juntas`) e cada **coluna é um passo** da trajetória (7 pontos). O fluxo:
  1. valida a matriz (nº de linhas = nº de juntas, todas as linhas com o mesmo nº de colunas);
  2. **espera o `ax12_controller` aparecer na rede** antes do primeiro passo (sem isso, os primeiros comandos se perdem na descoberta do DDS);
  3. em loop: lê a coluna atual, calcula a velocidade de cada junta como `|Δposição| / passo` (todas chegam ao alvo juntas), publica e espera `passo + pausa` segundos **processando a rede** (alertas chegam mesmo durante a pausa);
  4. se o controlador publicar `FALHA FATAL` em `/hardware_errors`, a marcha **para sozinha** — não adianta mandar passos para um robô sem hardware.

Para alterar a marcha, edite a `matriz_movimento` (valores em radianos) e/ou os tempos `passo` e `pausa`.

### `ax12_control/ax12_monitor.py` — painel de telemetria

Visualizador que roda no PC de comando (em outro terminal, junto do `send_gait`):

```bash
ros2 run ax12_control ax12_monitor
```

Mostra uma tabela atualizada 2×/s no terminal — ângulo, velocidade, torque (% e N·m), tensão, temperatura e status de cada motor — mais os últimos alertas de `/hardware_errors`. Não exige interface gráfica (funciona por SSH). É a ferramenta para flagrar um overload em formação: a coluna de torque sobe e o status muda para `ATENCAO` antes de o motor entrar em proteção.

### `legacy/` — versões antigas (não compiladas)

- **`controller_antigo.py`**: versão anterior do controlador que, além de escrever, **lia** a posição real dos motores a 10 Hz, publicava em `/joint_states` e reportava erros de hardware (ex.: sobrecarga) no tópico `/hardware_errors`. Usava QoS `RELIABLE/depth=10`, que se mostrou problemático no Wi-Fi.
- **`send_antigo.py`**: cliente correspondente, com três listas separadas (tornozelo/joelho/quadril) em vez da matriz por junta.

Esses arquivos ficam fora do módulo `ax12_control/` de propósito: não são instalados pelo build e servem só de referência (ex.: para reintroduzir a leitura de posição no futuro).

### `docs/bizuario_ros.md` — diagnóstico

Cola de comandos do ROS 2 (`ros2 node list`, `ros2 topic info`, `ros2 topic echo`, …) e glossário de QoS, útil para depurar a comunicação entre as máquinas.

## Telemetria e visualização

Com o controlador rodando, três formas de ver os dados dos motores (todas no PC, via rede):

| Ferramenta | Para quê | Como |
|---|---|---|
| `ax12_monitor` (deste pacote) | Tabela ao vivo no terminal, sem GUI | `ros2 run ax12_control ax12_monitor` |
| **PlotJuggler** (padrão da indústria) | Curvas ao longo do tempo (posição, torque...) | `sudo apt install ros-$ROS_DISTRO-plotjuggler-ros` e `ros2 run plotjuggler plotjuggler` → aba *Streaming* → assine `/joint_states` |
| **rqt_plot** | Gráfico rápido de um campo | `ros2 run rqt_plot rqt_plot /joint_states/position[0]` |

> [!NOTE]
> O "torque" lido é o **Present Load** do AX-12: uma estimativa interna do esforço em % do torque máximo — o AX-12 não tem sensor de torque real. O valor em N·m publicado no campo `effort` de `/joint_states` é aproximado a partir do stall torque nominal (~1,5 N·m a 12 V). É exatamente esse sinal que dispara a proteção de overload, então é o número certo para vigiar.

A leitura usa o barramento junto com os comandos (8 motores × 5 Hz = 40 transações/s, tranquilo a 1 Mbps). Para desligá-la e dedicar o barramento só à escrita: `--ros-args -p taxa_leitura:=0.0`.

## Detalhes técnicos

- **Mensagem**: `trajectory_msgs/JointTrajectory` — posições em **radianos**, velocidades em **rad/s**.
- **QoS**: `BEST_EFFORT`, `KEEP_LAST`, `depth=1` nos dois lados (**precisa ser igual**, ou o subscriber não recebe nada). Escolha deliberada para Wi-Fi: um comando perdido é descartado em vez de reenviado atrasado — comando velho chegando fora de hora é pior que comando perdido.
- **Conversões** (feitas no controlador):
  - Posição: `goal = (rad + 2,618) × 1023 / 5,236`, saturada em 0–1023.
  - Velocidade: `vel = |rad/s| × 86,03`, saturada em 1–1023 (0 significaria "velocidade máxima" no AX-12, por isso o mínimo é 1).

## Solução de problemas

| Sintoma | Causa provável | Solução |
|---|---|---|
| `Falha ao abrir a porta!` | Porta errada ou sem permissão | Confira com `ls /dev/ttyACM*`; ajuste `DEVICENAME` no código. Permissão: `sudo usermod -aG dialout $USER` e relogue |
| `send_gait` roda mas o robô não se mexe | Máquinas em domínios diferentes ou QoS incompatível | Confirme `echo $ROS_DOMAIN_ID` igual nos dois lados; `ros2 topic info /joint_trajectory -v` deve listar 1 publisher e 1 subscription |
| `Package 'ax12_control' not found` | Workspace não carregado | `source ~/ax12_control_ws/install/setup.bash` (confira se está no `~/.bashrc`) |
| Motores desligam ao ligar o torque | Fonte insuficiente para o pico de corrente | Verifique a fonte; o código já escalona o torque com 50 ms entre motores |
| Nós não se enxergam no Wi-Fi | Firewall ou multicast bloqueado | Teste com `ros2 multicast receive/send`; libere o firewall (`sudo ufw allow in proto udp`) |

Mais comandos de diagnóstico em [docs/bizuario_ros.md](docs/bizuario_ros.md).

## Licença

Distribuído sob a licença MIT — veja [LICENSE](LICENSE).
