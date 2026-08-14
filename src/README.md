# src/ — pacotes ROS 2 do Adam

Esta pasta é um mini-workspace: 3 pacotes ROS 2 lado a lado, prontos para `colcon build` a
partir da raiz do repositório (veja o [README principal](../README.md)).

## Organograma

```
src/
├── ax12_control/                 # pacote: controle dos motores AX-12
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── ax12_control
│   ├── launch/
│   │   ├── visualizar_marcha.launch.py
│   │   ├── controle_manual.launch.py
│   │   ├── marcha_manual.launch.py
│   │   ├── medir_roll.launch.py
│   │   └── controle_pe.launch.py
│   ├── ax12_control/             # módulo Python (mesmo nome = convenção ament_python)
│   │   ├── __init__.py
│   │   ├── ax12_controller.py
│   │   ├── send_gait.py
│   │   ├── ax12_monitor.py
│   │   ├── visualizar_marcha.py
│   │   ├── passo_slider.py
│   │   ├── controle_manual.py
│   │   ├── marcha_manual.py
│   │   ├── marcha_continua.py
│   │   ├── medir_roll.py
│   │   ├── controle_pe.py
│   │   ├── gait_bridge.py
│   │   ├── adam.rviz
│   │   ├── otimizada.yaml
│   │   ├── cin_inve.yaml
│   │   ├── cin_inve_roll.yaml
│   │   ├── matriz_zmp.yaml
│   │   ├── cin_inve_2.yaml
│   │   ├── teste_equilibrio.yaml
│   │   ├── marcha_pe.yaml
│   │   ├── marcha_avanco.yaml
│   │   └── marcha_avanco_suave.yaml
│   ├── scripts/                  # utilitários offline (NÃO são nós ROS, rodam com `python3`)
│   │   ├── cabecalho.txt         # cabeçalho colado pelo gerar_mega_matriz.py
│   │   ├── converter_matriz_lugar.py
│   │   ├── gerar_marcha_pe.py
│   │   ├── gerar_mega_matriz.py
│   │   ├── teste_motores.py
│   │   └── validacao_ik/         # bateria de validação FK/IK do controle_pe
│   │       ├── altura_quadril.py
│   │       ├── ik_harness.py
│   │       ├── teste_x_trim.py
│   │       └── verificar_x_pes.py
│   └── firmware/                 # firmware embarcado (Arduino IDE — NÃO é pacote ROS)
│       ├── opencr_dxl_imu_bridge/
│       │   └── opencr_dxl_imu_bridge.ino
│       └── opencr_hurocup/       # 32 arquivos — port do firmware STM32 do HuroCup p/ OpenCR
│
├── adam_urdf/                    # pacote: URDF, meshes e launch files do robô
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   ├── adam_fixed.urdf       # fonte de verdade (origens visuais corrigidas à mão)
│   │   ├── gen_xacro.py          # gera adam.urdf.xacro a partir do adam_fixed.urdf
│   │   ├── adam.urdf.xacro       # gerado — não editar à mão
│   │   ├── adam.ros2_control.xacro
│   │   ├── adam.urdf             # versão antiga, sem as correções de origem
│   │   └── adam.csv              # export de referência (BOM/juntas), não usado em runtime
│   ├── meshes/                   # 16 arquivos .STL (pernas e braços)
│   ├── config/
│   │   ├── adam.rviz
│   │   ├── ros2_controllers.yaml
│   │   └── joint_names_adam.yaml
│   └── launch/
│       ├── display.launch.py     # visualizar URDF no RViz (com/sem sliders)
│       ├── mock.launch.py        # digital twin: ros2_control mock + RViz
│       ├── gazebo.launch.py      # Gazebo (legado, não mantido)
│       └── display.launch        # launch ROS1 (legado, não mantido)
│
└── adam_moveit_config/            # pacote: configuração MoveIt2 (gerado pelo Setup Assistant)
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/
    │   ├── adam.srdf
    │   ├── kinematics.yaml
    │   ├── joint_limits.yaml
    │   └── moveit_controllers.yaml
    └── launch/
        ├── demo.launch.py        # digital twin completo: mock + MoveIt2 + RViz
        └── move_group.launch.py  # só o move_group (assume mock.launch.py já rodando)
```

## Cada pacote em detalhe

### `ax12_control/` — controle dos motores

O único pacote com código próprio (Python). Tudo que fala com o hardware ou gera/visualiza
marcha mora aqui. Documentação aprofundada: [docs/arquitetura.md](../docs/arquitetura.md).

| Arquivo | Função |
|---|---|
| `ax12_controller.py` | Nó de hardware — único processo que toca o barramento serial (roda na Raspberry Pi) |
| `send_gait.py` | Lê uma marcha (`.yaml`) e publica `/joint_trajectory` (roda no PC de comando) |
| `ax12_monitor.py` | Painel de telemetria ao vivo no terminal (ângulo, torque, tensão, temperatura) |
| `visualizar_marcha.py` | Publica `/joint_states` direto do YAML, sem `ros2_control` — visualização sem hardware |
| `passo_slider.py` | Janela Qt com slider/botões para escolher manualmente a etapa da marcha no RViz |
| `controle_manual.py` | Janela Qt com um slider por junta — jog manual dos motores reais via `/joint_trajectory`, com o RViz espelhando a posição real (telemetria do `ax12_controller`) |
| `marcha_manual.py` | Janela Qt com slider/botões para escolher a coluna da matriz de marcha — o robô real vai à pose da etapa escolhida e o RViz espelha a posição real (une `visualizar_marcha` + `controle_manual`) |
| `marcha_continua.py` | Roda uma marcha em ciclo contínuo no robô real: ao iniciar vai pra coluna 1 e espera o play, depois repete o ciclo (coluna 2, 3, ..., volta pra 1) — reaproveita `ConexaoRobo`/`carregar_marcha` do `send_gait.py` |
| `medir_roll.py` | Janela Qt com UM slider que comanda as 4 juntas de roll juntas (pitchs fixos na coluna 1 da matriz) — mede no robô real o ângulo de roll necessário para transferir o peso entre as pernas |
| `controle_pe.py` | Janela Qt com IK cartesiana do pé: 5 sliders (roll central, X da passada em oposição dir/esq, Z de cada pé, trim de quadril) resolvidos por Newton sobre a FK exata do URDF, sempre com o pé paralelo ao chão. Botão "Exportar coluna" imprime os 10 ângulos prontos para colar numa matriz YAML |
| `gait_bridge.py` | Ponte entre `send_gait` (QoS BEST_EFFORT) e os `JointTrajectoryController` do `adam_urdf`/MoveIt2 (QoS RELIABLE) |
| `otimizada.yaml`, `cin_inve.yaml`, `cin_inve_roll.yaml`, `matriz_zmp.yaml`, `cin_inve_2.yaml` | Marchas prontas — detalhes de cada uma em [docs/arquitetura.md](../../docs/arquitetura.md#marchas-disponíveis) |
| `teste_equilibrio.yaml` | Balanço NO LUGAR (8 etapas) gerado pela IK do `controle_pe`: X=0 em todas as colunas, só transferência de peso lateral (roll) + levantar cada pé, em colunas separadas |
| `marcha_pe.yaml` | Marcha (4 etapas) gerada pela IK do `controle_pe`: roll e levantamento do pé de balanço acontecem JUNTOS na mesma coluna, trim de quadril −9° |
| `marcha_avanco.yaml` | Marcha (8 etapas) — mesmos parâmetros da `teste_equilibrio` (roll ±16°, trim −7°) adicionando avanço X=±10mm |
| `marcha_avanco_suave.yaml` | A `marcha_avanco` expandida em "mega matriz" por rampa cosseno entre colunas (gerada por `scripts/gerar_mega_matriz.py`) — movimento contínuo em vez de 8 saltos discretos |
| `adam.rviz` | Config do RViz usada por `visualizar_marcha.launch.py` |
| `package.xml` / `setup.py` / `setup.cfg` / `resource/` | Metadados do pacote (dependências, `console_scripts`, instalação) |

### `ax12_control/scripts/` — utilitários offline (sem ROS)

Rodam com `python3` puro — não são nós, não usam `ros2 run` — mas dependem do pacote
**instalado** (`colcon build` + `source install/setup.bash`), porque importam
`ax12_control.controle_pe`/`send_gait` para reaproveitar a mesma FK/IK e os mesmos
carregadores de matriz usados no robô real.

| Arquivo | Função |
|---|---|
| `gerar_mega_matriz.py` | Expande a `marcha_avanco` numa "mega matriz" (`marcha_avanco_suave`) por rampa cosseno entre colunas consecutivas |
| `gerar_marcha_pe.py` | Gera a `marcha_pe.yaml` resolvendo cada coluna pela mesma IK do `controle_pe.aplicar()` |
| `converter_matriz_lugar.py` | Converte uma tabela em graus (gerada externamente) para radianos e valida contra os limites de `controle_pe.py` |
| `teste_motores.py` | PING em cada motor esperado do projeto, direto por serial + `dynamixel_sdk` (sem ROS) — diagnóstico rápido de quais motores respondem |
| `validacao_ik/ik_harness.py` | Valida a IK do `controle_pe` contra a FK exata do URDF: reproduz a coluna 1, varre a grade completa dos sliders (erro < 1mm, pé plano, dentro dos limites) |
| `validacao_ik/teste_x_trim.py` | Valida a combinação X (oposição dir/esq) + trim de quadril em toda a faixa dos sliders |
| `validacao_ik/verificar_x_pes.py` | Audita uma matriz já gerada: imprime o X (via FK) do tornozelo de cada perna em cada coluna, pra conferir a simetria dir/esq |
| `validacao_ik/altura_quadril.py` | Calcula a altura do quadril acima do pé na postura base — a referência física por trás do Z=0 do `controle_pe` |

### `ax12_control/firmware/` — firmware embarcado (não é pacote ROS)

Sketches Arduino, compilados e gravados pela Arduino IDE (placa **OpenCR**) — não passam
pelo `colcon`/`ros2`.

| Pasta | Função |
|---|---|
| `opencr_dxl_imu_bridge/` | Bridge USB↔DXL transparente (compatível com o DYNAMIXEL Wizard) que também expõe a IMU da OpenCR como um dispositivo Protocol 1.0 no endereço 200, no estilo do firmware `opencr_op3` da ROBOTIS |
| `opencr_hurocup/` | Port completo (32 arquivos) do firmware STM32 do projeto HuroCup para a OpenCR (STM32F746): mesma lógica pura do original (stream de posições das juntas, estabilização por IMU, soft-start, safety), camadas de hardware (IMU, bateria, half-duplex DXL, bootflag, systick) reescritas para o core Arduino da OpenCR. **Nunca compilado** — falta validar na Arduino IDE antes de gravar |

### `adam_urdf/` — modelo do robô

Descrição física do Adam: URDF, malhas 3D e os launch files que o colocam no RViz ou no
`ros2_control`. Sem código Python de nó próprio — é um pacote `ament_cmake` de dados.

| Item | Função |
|---|---|
| `adam_fixed.urdf` | Fonte de verdade da geometria (origens visuais corrigidas manualmente) |
| `gen_xacro.py` | Regenera `adam.urdf.xacro` a partir do `adam_fixed.urdf`, injetando limites de junta — **rode este script para editar limites, nunca edite o `.xacro` direto** |
| `adam.urdf.xacro` | Versão com `<ros2_control>` (gerada — usada por `mock.launch.py` e pelo MoveIt2) |
| `meshes/*.STL` | As 16 peças do robô (pernas e braços) referenciadas pelo URDF via `package://adam_urdf/meshes/...` |
| `display.launch.py` | RViz com o modelo: `use_gui_sliders:=true` (padrão) para sliders manuais, `:=false` para espelhar o robô real via `/joint_states` da rede |
| `mock.launch.py` | Digital twin: `ros2_control` com `mock_components/GenericSystem` + RViz, sem MoveIt2 |
| `gazebo.launch.py`, `display.launch` | Legados (Gazebo / ROS1), não mantidos |

### `adam_moveit_config/` — planejamento de movimento

Gerado pelo MoveIt Setup Assistant a partir do `adam_urdf`. Configura os planning groups
(`perna_direita`, `perna_esquerda`), o SRDF e a ponte com os controllers do `ros2_control`.

| Item | Função |
|---|---|
| `adam.srdf` | Planning groups, colisões permitidas, poses nomeadas |
| `kinematics.yaml`, `joint_limits.yaml` | Configuração do solver de IK e limites usados pelo MoveIt2 |
| `moveit_controllers.yaml` | Liga o MoveIt2 aos `JointTrajectoryController`s definidos em `adam_urdf/config/ros2_controllers.yaml` |
| `demo.launch.py` | Sobe tudo: mock hardware + `move_group` + RViz com o plugin MotionPlanning |
| `move_group.launch.py` | Só o `move_group`, para quando `mock.launch.py` (do `adam_urdf`) já está rodando |

## Comandos para rodar cada código

Antes de qualquer comando, builde e sourcie a partir da raiz do repositório:

```bash
colcon build
source install/setup.bash
```

**`ax12_control` — nós (`ros2 run`):**

```bash
ros2 run ax12_control ax12_controller   # Raspberry Pi — liga o torque e fala com os motores
ros2 run ax12_control send_gait         # PC de comando — envia uma marcha
ros2 run ax12_control ax12_monitor      # PC de comando — telemetria no terminal
ros2 run ax12_control gait_bridge       # PC de comando — ponte para ros2_control/MoveIt2
ros2 run ax12_control marcha_continua --ros-args -p matriz:=marcha_pe   # ciclo continuo no robo real
```

**`ax12_control` — launch (visualização sem hardware):**

```bash
ros2 launch ax12_control visualizar_marcha.launch.py matriz:=cin_inve   # ou otimizada
```

**`ax12_control` — launch (jog manual com hardware real + RViz, tudo numa máquina só):**

```bash
ros2 launch ax12_control controle_manual.launch.py
ros2 launch ax12_control controle_manual.launch.py device:=/dev/ttyUSB0 velocidade:=0.5
```

**`ax12_control` — launch (marcha por matriz no robô real: escolhe a coluna e o robô vai):**

```bash
ros2 launch ax12_control marcha_manual.launch.py                    # matriz otimizada
ros2 launch ax12_control marcha_manual.launch.py matriz:=cin_inve
```

**`ax12_control` — launch (medir roll: 1 slider comanda as 4 juntas de roll no robô real + RViz):**

```bash
ros2 launch ax12_control medir_roll.launch.py
ros2 launch ax12_control medir_roll.launch.py matriz:=matriz_zmp velocidade:=0.3
```

**`ax12_control` — launch (IK cartesiana do pé: roll + X/Z por perna no robô real + RViz):**

```bash
ros2 launch ax12_control controle_pe.launch.py
ros2 launch ax12_control controle_pe.launch.py matriz:=matriz_zmp velocidade:=0.3
```

**`ax12_control/scripts/` — utilitários offline (sem `ros2 run`; precisam do workspace buildado e sourced):**

```bash
python3 src/ax12_control/scripts/gerar_mega_matriz.py
python3 src/ax12_control/scripts/gerar_marcha_pe.py
python3 src/ax12_control/scripts/converter_matriz_lugar.py
python3 src/ax12_control/scripts/teste_motores.py /dev/ttyACM0
python3 src/ax12_control/scripts/validacao_ik/ik_harness.py
python3 src/ax12_control/scripts/validacao_ik/teste_x_trim.py
python3 src/ax12_control/scripts/validacao_ik/verificar_x_pes.py marcha_avanco
python3 src/ax12_control/scripts/validacao_ik/altura_quadril.py
```

**`ax12_control/firmware/` — não usa `colcon`/`ros2`:**

Abrir o `.ino` (`opencr_hurocup/opencr_hurocup.ino` ou `opencr_dxl_imu_bridge/opencr_dxl_imu_bridge.ino`)
na Arduino IDE com o board package da OpenCR instalado, selecionar a placa OpenCR e usar
Verify/Upload. Não há comando de linha de comando validado nesta sessão (`arduino-cli` não
estava disponível no PATH).

**`adam_urdf` — launch:**

```bash
ros2 launch adam_urdf display.launch.py                        # sliders manuais
ros2 launch adam_urdf display.launch.py use_gui_sliders:=false  # espelha o robô real
ros2 launch adam_urdf mock.launch.py                            # digital twin: ros2_control mock
```

**`adam_moveit_config` — launch:**

```bash
ros2 launch adam_moveit_config demo.launch.py        # digital twin completo + MoveIt2
ros2 launch adam_moveit_config move_group.launch.py  # só o move_group (mock.launch.py já rodando)
```

## Clonar e buildar — mesmo padrão em qualquer máquina

O repositório já é o workspace: tem `src/` na raiz, então não precisa criar uma pasta de
workspace separada nem symlink. A receita é a mesma em qualquer máquina — PC de comando ou
Raspberry Pi (a Pi também roda `adam_urdf`, via `controle_manual.launch.py`, jog manual com
RViz espelhando o robô real):

```bash
git clone https://github.com/Machado-ime/Controle-Ax12---ROS2.git ~/dev/Controle-Ax12---ROS2
cd ~/dev/Controle-Ax12---ROS2
colcon build
echo "source ~/dev/Controle-Ax12---ROS2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Para atualizar depois:

```bash
cd ~/dev/Controle-Ax12---ROS2
git pull
colcon build
```
