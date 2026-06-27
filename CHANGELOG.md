# Changelog

Histórico de mudanças relevantes deste pacote (`ax12_control`). Formato baseado em
[Keep a Changelog](https://keepachangelog.com/pt-BR/1.0.0/). Histórico anterior a este
arquivo: ver `git log`.

## [Não lançado]

### Adicionado
- `controle_manual.py` + `controle_manual.launch.py` — janela Qt com um slider por
  junta para jog manual dos motores AX-12 reais. Publica só `/joint_trajectory`; o
  RViz acompanha via a telemetria real já publicada pelo `ax12_controller`,
  evitando dois publishers competindo em `/joint_states`. Pensado para rodar tudo
  numa máquina só (a Raspberry Pi), substituindo o fluxo PC+Pi via rede para testes
  manuais de junta a junta.

### Removido
- `src/matrizes-de-movimento/`: `cin_inve.yaml`/`otimizada.yaml` eram cópias duplicadas das
  marchas em `src/ax12_control/ax12_control/`; `otimizacao.h` (header C de protótipo de 18
  motores) não tinha valor de referência. Não arquivados.

### Renomeado
- Pacote `adam` renomeado para `adam_urdf` (pasta `src/adam_urdf/`, `package.xml`,
  `CMakeLists.txt`, todos os caminhos `package://adam/meshes/...` nos arquivos URDF/xacro, e
  todas as chamadas `get_package_share_directory`/`FindPackageShare` que o referenciavam).
  Evita ambiguidade com o nome "Adam" do robô. Não afeta `adam_moveit_config`, que continua
  usando `MoveItConfigsBuilder("adam", ...)` — esse "adam" é o nome do robô/SRDF
  (`adam.srdf`), não o pacote URDF.

### Adicionado
- Suporte a 10 motores: rolls de quadril (`pd_roll_quadril_9`, ID 9 e `pe_roll_quadril_10`,
  ID 14), além dos 8 já existentes.
- `gait_bridge.py` — ponte entre `send_gait` (QoS BEST_EFFORT) e os
  `JointTrajectoryController` do `ros2_control` (QoS RELIABLE), usada no digital twin via
  MoveIt2/mock.
- `passo_slider.py` — janela Qt com slider e botões ◀▶ para escolher manualmente a etapa da
  marcha exibida no RViz.
- Seções de referência sobre `ros2_control`, MoveIt2, Qt e `diagnostic_msgs` em
  `docs/ref/referencias-ax12.md`.
- Reorganização da documentação seguindo o guia de Gestão de Conhecimento da equipe: README
  enxuto, `docs/install.md`, `docs/troubleshooting.md`, `docs/ref/`, `docs/adr.md`, `AGENTS.md`
  e arquivos de saúde da comunidade em `.github/`.

### Alterado
- Todo o código movido para `src/`: o pacote ROS inteiro (`package.xml`, `setup.py`, `launch/`
  e o módulo Python) agora vive em `src/ax12_control/`. O `colcon` encontra o pacote
  recursivamente, então o comando de build não muda.
- `adam/` e `adam_moveit_config/` (que viviam na raiz do repositório) movidos para
  `src/adam_urdf/` e `src/adam_moveit_config/`, junto do `ax12_control` — o repositório agora é um
  mini-workspace com os 3 pacotes lado a lado em `src/`.

### Removido
- `legacy/` (`controller_antigo.py`, `send_antigo.py`) e os protótipos `AX12Controller_v1.py`/
  `AX12Controller_v2.py` — sem valor de referência, removidos em vez de arquivados.

### Corrigido
- `ax12_controller.py`: `joint_map`/`joint_limits` estavam com nomes de junta no padrão antigo
  (pré-URDF), o que descartava silenciosamente todos os comandos de marcha — revertido para o
  padrão de nomes do URDF, que é o que `send_gait` realmente publica.
- `display.launch.py` (pacote `adam_urdf`): `joint_state_publisher_gui` agora é condicional
  (`use_gui_sliders`), evitando conflito com o `/joint_states` real ao espelhar o robô.
- Build quebrado: um achatamento manual de `src/ax12_control/ax12_control/*` para `src/*`
  tirou a pasta que o `setup.py` espera para o módulo Python — `colcon build` falhava com
  `can't copy 'ax12_control/adam.rviz': doesn't exist`. Estrutura de pacote restaurada.
