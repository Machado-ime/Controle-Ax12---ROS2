# Changelog

Histórico de mudanças relevantes deste pacote (`ax12_control`). Formato baseado em
[Keep a Changelog](https://keepachangelog.com/pt-BR/1.0.0/). Histórico anterior a este
arquivo: ver `git log`.

## [Não lançado]

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

### Corrigido
- `ax12_controller.py`: `joint_map`/`joint_limits` estavam com nomes de junta no padrão antigo
  (pré-URDF), o que descartava silenciosamente todos os comandos de marcha — revertido para o
  padrão de nomes do URDF, que é o que `send_gait` realmente publica.
- `display.launch.py` (pacote `adam`): `joint_state_publisher_gui` agora é condicional
  (`use_gui_sliders`), evitando conflito com o `/joint_states` real ao espelhar o robô.
