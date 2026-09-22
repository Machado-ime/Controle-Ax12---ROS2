# src/ — pacotes ROS 2 do Adam

Esta pasta é um mini-workspace: 2 pacotes ROS 2 lado a lado, prontos para `colcon build` a
partir da raiz do repositório (veja o [README principal](../README.md)).

A divisão segue a convenção do ecossistema ROS 2: `_description` guarda só o modelo físico
(dados estáticos), e o driver fica num pacote próprio. Um terceiro pacote, `adam_bringup`
(runtime do `ros2_control`, digital twin mock), existiu neste repositório mas foi removido —
não subia nada que o hardware real dependesse. O nó que falava com ele, `gait_bridge.py`,
segue no pacote mas ficou órfão (ver a tabela abaixo).

## Organograma

```
src/
├── ax12_control/                 # pacote: driver + ferramentas de operação
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
│       ├── gait_bridge.py        # órfão — o pacote que ele alimentava foi removido
│       ├── adam.rviz
│       └── matriz_ciclo.yaml     # única marcha commitada (ver seção "Marchas")
│
└── adam_description/             # pacote: modelo físico do robô (só dados)
    ├── package.xml
    ├── CMakeLists.txt
    ├── urdf/
    │   ├── adam_fixed.urdf       # fonte de verdade (origens visuais corrigidas à mão)
    │   ├── adam.urdf.xacro       # gerado — não editar à mão
    │   ├── adam.ros2_control.xacro
    │   ├── adam.urdf             # versão antiga, sem as correções de origem
    │   └── adam.csv              # export de referência (BOM/juntas), não usado em runtime
    ├── meshes/                   # 17 arquivos .STL (pernas e braços)
    ├── config/
    │   ├── adam.rviz
    │   └── joint_names_adam.yaml # sobra do exportador SolidWorks, não usada em runtime
    ├── scripts/                  # geradores offline (python3 puro, não instalados)
    │   ├── gen_xacro.py          # gera adam.urdf.xacro a partir do adam_fixed.urdf
    │   └── fix_urdf_origins.py   # corrige as origens visuais do export do SolidWorks
    └── launch/
        ├── display.launch.py     # visualizar URDF no RViz — use_gui_sliders:=true/false
        ├── gazebo.launch.py      # Gazebo (legado, não mantido)
        └── display.launch        # launch ROS1 (legado, não mantido)
```

## Cada pacote em detalhe

### `ax12_control/` — controle dos motores

O único pacote com código próprio (Python). Tudo que fala com o hardware ou gera/visualiza
marcha mora aqui. Documentação aprofundada: [docs/arquitetura.md](../docs/arquitetura.md).

> `matriz_ciclo.yaml` é a única marcha commitada no repositório. Veja "Criar uma marcha nova"
> em [docs/arquitetura.md](../docs/arquitetura.md#criar-uma-marcha-nova) para escrever a sua.

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
| `gait_bridge.py` | **Órfã.** Convertia QoS entre `send_gait` (BEST_EFFORT) e os `JointTrajectoryController` do `adam_bringup` (RELIABLE) — esse pacote foi removido, então hoje publica sem ninguém escutando |
| `adam.rviz` | Config do RViz usada por `visualizar_marcha.launch.py` |
| `package.xml` / `setup.py` / `setup.cfg` / `resource/` | Metadados do pacote (dependências, `console_scripts`, instalação) |

### `adam_description/` — modelo do robô

Descrição física do Adam: URDF, malhas 3D e os launch de visualização. Sem código de nó
próprio — é um pacote `ament_cmake` de dados. Não sobe `ros2_control` nenhum (o pacote que
fazia isso, `adam_bringup`, foi removido); `display.launch.py` é o único jeito de ver o
modelo hoje.

| Item | Função |
|---|---|
| `adam_fixed.urdf` | Fonte de verdade da geometria (origens visuais corrigidas manualmente) |
| `adam.urdf.xacro` | Versão com `<ros2_control>` (gerada — sem consumidor no repositório hoje) |
| `meshes/*.STL` | As 17 peças do robô (pernas e braços) referenciadas pelo URDF via `package://adam_description/meshes/...` |
| `scripts/gen_xacro.py` | Regenera `adam.urdf.xacro` a partir do `adam_fixed.urdf`, injetando limites de junta — **rode este script para editar limites, nunca edite o `.xacro` direto** |
| `scripts/fix_urdf_origins.py` | Corrige as origens visuais/colisão do export cru do SolidWorks, gerando o `adam_fixed.urdf` |
| `display.launch.py` | RViz com o modelo: `use_gui_sliders:=true` (padrão) para sliders manuais, `:=false` para espelhar o robô real via `/joint_states` da rede |
| `gazebo.launch.py`, `display.launch` | Legados (Gazebo / ROS1), não mantidos |

## Comandos para rodar cada código

Antes de qualquer comando, builde e sourcie a partir da raiz do repositório:

```bash
colcon build
source install/setup.bash
```

> Os comandos abaixo usam `matriz:=<nome>` como placeholder — `matriz_ciclo` é a única marcha
> commitada (ver nota acima); `<nome>` também aceita um `.yaml` seu, criado seguindo "Criar
> uma marcha nova" em [docs/arquitetura.md](../docs/arquitetura.md#criar-uma-marcha-nova).

**`ax12_control` — nós (`ros2 run`):**

```bash
ros2 run ax12_control ax12_controller   # Raspberry Pi — liga o torque e fala com os motores
ros2 run ax12_control ax12_controller --ros-args -p ligar_torque:=false   # modo observador (só leitura)
ros2 run ax12_control send_gait --ros-args -p matriz:=<nome>   # PC de comando — envia uma marcha
ros2 run ax12_control ax12_monitor      # PC de comando — telemetria no terminal
ros2 run ax12_control marcha_continua --ros-args -p matriz:=<nome>   # ciclo continuo no robo real
```

> `gait_bridge` não está na lista acima: continua instalado, mas ficou órfão (ver nota no
> topo deste arquivo) — rodá-lo hoje não tem efeito prático.

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

**`adam_description` — launch (visualização do modelo):**

```bash
ros2 launch adam_description display.launch.py                         # sliders manuais
ros2 launch adam_description display.launch.py use_gui_sliders:=false  # espelha o robô real
```

**`adam_description/scripts/` — geradores offline (sem `ros2 run`, rodar da raiz do pacote):**

```bash
cd src/adam_description
python3 scripts/gen_xacro.py          # regenera adam.urdf.xacro a partir do adam_fixed.urdf
python3 scripts/fix_urdf_origins.py   # regenera adam_fixed.urdf a partir do export cru
```

## Clonar e buildar — mesmo padrão em qualquer máquina

O repositório já é o workspace: tem `src/` na raiz, então não precisa criar uma pasta de
workspace separada nem symlink. A receita é a mesma em qualquer máquina — PC de comando ou
Raspberry Pi (a Pi também roda `adam_description`, via `controle_manual.launch.py`, jog manual
com RViz espelhando o robô real):

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
