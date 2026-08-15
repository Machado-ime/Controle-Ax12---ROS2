# src/ — pacotes ROS 2 do Adam

Esta pasta é um mini-workspace: 2 pacotes ROS 2 lado a lado, prontos para `colcon build` a
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
│   └── ax12_control/             # módulo Python (mesmo nome = convenção ament_python)
│       ├── __init__.py
│       ├── ax12_controller.py
│       ├── send_gait.py
│       ├── ax12_monitor.py
│       ├── visualizar_marcha.py
│       ├── passo_slider.py
│       ├── controle_manual.py
│       ├── marcha_manual.py
│       ├── marcha_continua.py
│       ├── medir_roll.py
│       ├── controle_pe.py
│       ├── gait_bridge.py
│       └── adam.rviz             # nenhuma marcha .yaml vem no repositório (ver docs/arquitetura.md)
│
└── adam_urdf/                    # pacote: URDF, meshes e launch files do robô
    ├── package.xml
    ├── CMakeLists.txt
    ├── urdf/
    │   ├── adam_fixed.urdf       # fonte de verdade (origens visuais corrigidas à mão)
    │   ├── gen_xacro.py          # gera adam.urdf.xacro a partir do adam_fixed.urdf
    │   ├── adam.urdf.xacro       # gerado — não editar à mão
    │   ├── adam.ros2_control.xacro
    │   ├── adam.urdf             # versão antiga, sem as correções de origem
    │   └── adam.csv              # export de referência (BOM/juntas), não usado em runtime
    ├── meshes/                   # 16 arquivos .STL (pernas e braços)
    ├── config/
    │   ├── adam.rviz
    │   ├── ros2_controllers.yaml
    │   └── joint_names_adam.yaml
    └── launch/
        ├── display.launch.py     # visualizar URDF no RViz (com/sem sliders)
        ├── mock.launch.py        # digital twin: ros2_control mock + RViz
        ├── gazebo.launch.py      # Gazebo (legado, não mantido)
        └── display.launch        # launch ROS1 (legado, não mantido)
```

## Cada pacote em detalhe

### `ax12_control/` — controle dos motores

O único pacote com código próprio (Python). Tudo que fala com o hardware ou gera/visualiza
marcha mora aqui. Documentação aprofundada: [docs/arquitetura.md](../docs/arquitetura.md).

> Nenhuma matriz de marcha (`.yaml`) vem pronta no repositório — foram removidas. Veja "Criar
> uma marcha nova" em [docs/arquitetura.md](../docs/arquitetura.md#criar-uma-marcha-nova).

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
| `gait_bridge.py` | Ponte entre `send_gait` (QoS BEST_EFFORT) e os `JointTrajectoryController` do `adam_urdf` (QoS RELIABLE) |
| `adam.rviz` | Config do RViz usada por `visualizar_marcha.launch.py` |
| `package.xml` / `setup.py` / `setup.cfg` / `resource/` | Metadados do pacote (dependências, `console_scripts`, instalação) |

### `adam_urdf/` — modelo do robô

Descrição física do Adam: URDF, malhas 3D e os launch files que o colocam no RViz ou no
`ros2_control`. Sem código Python de nó próprio — é um pacote `ament_cmake` de dados.

| Item | Função |
|---|---|
| `adam_fixed.urdf` | Fonte de verdade da geometria (origens visuais corrigidas manualmente) |
| `gen_xacro.py` | Regenera `adam.urdf.xacro` a partir do `adam_fixed.urdf`, injetando limites de junta — **rode este script para editar limites, nunca edite o `.xacro` direto** |
| `adam.urdf.xacro` | Versão com `<ros2_control>` (gerada — usada por `mock.launch.py`) |
| `meshes/*.STL` | As 16 peças do robô (pernas e braços) referenciadas pelo URDF via `package://adam_urdf/meshes/...` |
| `display.launch.py` | RViz com o modelo: `use_gui_sliders:=true` (padrão) para sliders manuais, `:=false` para espelhar o robô real via `/joint_states` da rede |
| `mock.launch.py` | Digital twin: `ros2_control` com `mock_components/GenericSystem` + RViz — sobe sozinho o `controller_manager` e os `JointTrajectoryController`s de cada perna |
| `gazebo.launch.py`, `display.launch` | Legados (Gazebo / ROS1), não mantidos |

## Comandos para rodar cada código

Antes de qualquer comando, builde e sourcie a partir da raiz do repositório:

```bash
colcon build
source install/setup.bash
```

> Os comandos abaixo usam `matriz:=<nome>` como placeholder — nenhuma marcha vem pronta no
> repositório (ver nota acima). `<nome>` deve apontar para um `.yaml` seu, criado seguindo
> "Criar uma marcha nova" em [docs/arquitetura.md](../docs/arquitetura.md#criar-uma-marcha-nova).

**`ax12_control` — nós (`ros2 run`):**

```bash
ros2 run ax12_control ax12_controller   # Raspberry Pi — liga o torque e fala com os motores
ros2 run ax12_control send_gait --ros-args -p matriz:=<nome>   # PC de comando — envia uma marcha
ros2 run ax12_control ax12_monitor      # PC de comando — telemetria no terminal
ros2 run ax12_control gait_bridge       # PC de comando — ponte para ros2_control
ros2 run ax12_control marcha_continua --ros-args -p matriz:=<nome>   # ciclo continuo no robo real
```

**`ax12_control` — launch (visualização sem hardware):**

```bash
ros2 launch ax12_control visualizar_marcha.launch.py matriz:=<nome>
```

**`ax12_control` — launch (jog manual com hardware real + RViz, tudo numa máquina só):**

```bash
ros2 launch ax12_control controle_manual.launch.py
ros2 launch ax12_control controle_manual.launch.py device:=/dev/ttyUSB0 velocidade:=0.5
```

**`ax12_control` — launch (marcha por matriz no robô real: escolhe a coluna e o robô vai):**

```bash
ros2 launch ax12_control marcha_manual.launch.py matriz:=<nome>
```

**`ax12_control` — launch (medir roll: 1 slider comanda as 4 juntas de roll no robô real + RViz):**

```bash
ros2 launch ax12_control medir_roll.launch.py matriz:=<nome> velocidade:=0.3
```

**`ax12_control` — launch (IK cartesiana do pé: roll + X/Z por perna no robô real + RViz):**

```bash
ros2 launch ax12_control controle_pe.launch.py matriz:=<nome> velocidade:=0.3
```

**`adam_urdf` — launch:**

```bash
ros2 launch adam_urdf display.launch.py                        # sliders manuais
ros2 launch adam_urdf display.launch.py use_gui_sliders:=false  # espelha o robô real
ros2 launch adam_urdf mock.launch.py                            # digital twin: ros2_control mock
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
