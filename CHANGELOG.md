# Changelog

Histórico de mudanças relevantes deste pacote (`ax12_control`). Formato baseado em
[Keep a Changelog](https://keepachangelog.com/pt-BR/1.0.0/). Histórico anterior a este
arquivo: ver `git log`.

## [Não lançado]

### Alterado
- Reorganização dos pacotes seguindo a convenção do ecossistema ROS 2. O workspace volta a
  ter 3 pacotes:
  - `adam_urdf` renomeado para **`adam_description`** (`_description` é o sufixo padrão para
    um pacote só de URDF/malhas). Atualizados `package.xml`, `CMakeLists.txt`, todas as URIs
    `package://adam_urdf/meshes/...` nos URDF/xacro/csv, e as chamadas
    `get_package_share_directory`/`FindPackageShare` nos launch dos dois pacotes.
  - Criado **`adam_bringup`**, para onde foram `config/ros2_controllers.yaml` e
    `launch/mock.launch.py`. Subir `controller_manager`/controllers não é papel de um pacote
    `_description`; o `adam_description` volta a ser só dado do modelo.
  - `gen_xacro.py` e `fix_urdf_origins.py` movidos para `adam_description/scripts/` (estavam
    soltos em `urdf/` e na raiz do pacote). Os dois passaram a resolver caminhos relativos ao
    próprio arquivo, então rodam de qualquer diretório; verificado que `gen_xacro.py`
    regenera o `adam.urdf.xacro` byte-idêntico.
  - `ax12_control/package.xml` passou a declarar `<exec_depend>adam_description</exec_depend>`
    — os launch files já carregavam o URDF via `get_package_share_directory`, mas a
    dependência não estava declarada.
  - `adam_description/package.xml`: autor/maintainer `TODO` (sobra do exportador SolidWorks)
    preenchidos e licença alinhada com o `LICENSE` do repositório (MIT, era BSD).

### Removido
- `src/adam_description/export.log` — 2259 linhas de log bruto do plugin SW2URDF, artefato de
  ferramenta que não deveria estar versionado.
- `src/ax12_control/firmware/` — os dois sketches Arduino para a placa OpenCR
  (`opencr_dxl_imu_bridge/`, `opencr_hurocup/`, 33 arquivos). Não passavam pelo `colcon`/`ros2`
  e nenhum código Python importava deles. O comentário em `ax12_controller.py` que apontava
  para `src/ax12_control/firmware/` (contexto do bloco IMU do OpenCR no ID 200) foi atualizado
  para não referenciar mais um caminho do repositório — a leitura de `/imu/data`
  (`taxa_imu` > 0) continua funcionando normalmente se a placa já tiver o firmware
  `opencr_dxl_imu_bridge` gravado; só a fonte desse firmware não mora mais aqui.
- Todas as 9 matrizes de marcha (`otimizada.yaml`, `cin_inve.yaml`, `cin_inve_roll.yaml`,
  `matriz_zmp.yaml`, `cin_inve_2.yaml`, `teste_equilibrio.yaml`, `marcha_pe.yaml`,
  `marcha_avanco.yaml`, `marcha_avanco_suave.yaml`) — o repositório não traz mais nenhuma
  marcha pronta. Os parâmetros `matriz` dos nós (`send_gait`, `visualizar_marcha`,
  `marcha_manual`, `marcha_continua`, `medir_roll`, `controle_pe`) mantêm seus valores padrão
  antigos como string, mas o arquivo correspondente não existe mais — rodar sem passar
  `-p matriz:=<nome>` com um `.yaml` próprio falha com `FileNotFoundError`. `adam.rviz` não foi
  afetado (não é marcha). Documentação atualizada em `README.md`, `src/README.md`,
  `docs/install.md` e `docs/arquitetura.md` (seção "Criar uma marcha nova" agora começa do
  zero, sem "copie um `.yaml` existente").
- `src/ax12_control/scripts/` — utilitários offline (`gerar_mega_matriz.py`,
  `gerar_marcha_pe.py`, `converter_matriz_lugar.py`, `teste_motores.py`) e a bateria
  `validacao_ik/` (`ik_harness.py`, `teste_x_trim.py`, `verificar_x_pes.py`,
  `altura_quadril.py`). Eram scripts `python3` avulsos, fora do grafo ROS — nenhum nó ou
  launch file dependia deles.
- `src/adam_moveit_config/` — pacote MoveIt2 gerado pelo Setup Assistant (SRDF, planning
  groups, `demo.launch.py`/`move_group.launch.py`). O workspace passa de 3 para 2 pacotes
  ROS (`ax12_control`, `adam_urdf`). O `gait_bridge` continua funcionando: o Caso 2
  (`send_gait` → `ros2_control`) depende só de `adam_urdf/launch/mock.launch.py`, que já sobe
  o `controller_manager` e os `JointTrajectoryController`s sozinho, sem MoveIt2. Planejamento
  de movimento (`move_group`) fica indisponível até o pacote ser regenerado.

### Adicionado
- `cin_inve_2.yaml` — marcha completa por ZMP (10 juntas x 8 etapas), gerada de
  `cin_ive_2.mat`, mesma estrutura da `matriz_zmp` mas com amplitude de passada maior nas
  juntas de pitch.

### Alterado
- Instalação simplificada: o clone do repositório agora É o workspace do `colcon` (tem `src/`
  na raiz) — não usa mais workspace separado com symlink nem sparse-checkout. Receita única em
  qualquer máquina: `git clone ... ~/dev/Controle-Ax12---ROS2 && cd ~/dev/Controle-Ax12---ROS2
  && colcon build`. Atualizado em `docs/install.md`, `src/README.md` e
  `docs/troubleshooting.md`.

### Adicionado
- `marcha_manual.py` + `marcha_manual.launch.py` — janela Qt com slider/botões ◀▶ para
  escolher a coluna (etapa) de uma matriz de marcha; o robô real vai à pose escolhida e o RViz
  espelha a posição real. Une o seletor de passo do `visualizar_marcha`/`passo_slider` com o
  envio ao hardware do `controle_manual`. Reaproveita o carregador/validador de matriz do
  `send_gait` e passa pelo `ax12_controller` (herda a correção de juntas invertidas).
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
- `ax12_controller.py`: as 4 juntas de pitch de tornozelo e quadril
  (`pd_picht_tornozelo_3`, `pe_picht_tornozelo_4`, `pd_picht_quadril_7`,
  `pe_pich_quadril_8`) estão com o motor montado com o eixo invertido em relação ao URDF —
  o mesmo comando movia o modelo no RViz para um lado e o robô real para o outro. Adicionado o
  conjunto `juntas_invertidas`, que troca o sinal do ângulo só na fronteira rad↔unidades do
  motor (escrita e leitura), mantendo limites, marcha, `/joint_states` e RViz na convenção do
  URDF. Como bônus, o clamp de limites passou a proteger o lado mecânico correto (os
  `joint_limits` já estavam gravados na convenção do URDF).
- `ax12_controller.py`: `joint_map`/`joint_limits` estavam com nomes de junta no padrão antigo
  (pré-URDF), o que descartava silenciosamente todos os comandos de marcha — revertido para o
  padrão de nomes do URDF, que é o que `send_gait` realmente publica.
- `display.launch.py` (pacote `adam_urdf`): `joint_state_publisher_gui` agora é condicional
  (`use_gui_sliders`), evitando conflito com o `/joint_states` real ao espelhar o robô.
- Build quebrado: um achatamento manual de `src/ax12_control/ax12_control/*` para `src/*`
  tirou a pasta que o `setup.py` espera para o módulo Python — `colcon build` falhava com
  `can't copy 'ax12_control/adam.rviz': doesn't exist`. Estrutura de pacote restaurada.
