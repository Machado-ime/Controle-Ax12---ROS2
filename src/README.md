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
│   │   └── marcha_manual.launch.py
│   └── ax12_control/             # módulo Python (mesmo nome = convenção ament_python)
│       ├── __init__.py
│       ├── ax12_controller.py
│       ├── send_gait.py
│       ├── ax12_monitor.py
│       ├── visualizar_marcha.py
│       ├── passo_slider.py
│       ├── controle_manual.py
│       ├── marcha_manual.py
│       ├── gait_bridge.py
│       ├── adam.rviz
│       ├── otimizada.yaml
│       ├── cin_inve.yaml
│       ├── cin_inve_roll.yaml
│       ├── matriz_zmp.yaml
│       └── cin_inve_2.yaml
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
| `gait_bridge.py` | Ponte entre `send_gait` (QoS BEST_EFFORT) e os `JointTrajectoryController` do `adam_urdf`/MoveIt2 (QoS RELIABLE) |
| `otimizada.yaml`, `cin_inve.yaml`, `cin_inve_roll.yaml`, `matriz_zmp.yaml`, `cin_inve_2.yaml` | Marchas prontas — detalhes de cada uma em [docs/arquitetura.md](../../docs/arquitetura.md#marchas-disponíveis) |
| `adam.rviz` | Config do RViz usada por `visualizar_marcha.launch.py` |
| `package.xml` / `setup.py` / `setup.cfg` / `resource/` | Metadados do pacote (dependências, `console_scripts`, instalação) |

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
