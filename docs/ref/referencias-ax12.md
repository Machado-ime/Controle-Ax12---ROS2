# Referências — controle do AX-12

Coletânea de repositórios e documentação úteis para o controle dos servos Dynamixel AX-12 (Protocolo 1.0) e para o restante da stack do digital twin (ros2_control, MoveIt2, RViz/Qt). Serve de ponto de partida ao consultar dúvidas de hardware, SDK, marcha ou visualização.

> **Stack deste projeto:** `dynamixel_sdk` (Python) + `GroupSyncWrite` + Protocolo 1.0 sob ROS 2 Jazzy, mais `ros2_control` (mock + futuro driver real), MoveIt2 (planejamento, ainda não gerado) e PyQt5/`python_qt_binding` (slider de passo). As referências mais alinhadas estão marcadas com ⭐.

- **Data da última atualização:** 2026-06-19

---

## 1. Documentação oficial ROBOTIS

| Recurso | Descrição |
|---|---|
| ⭐ [AX-12A — e-Manual](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) | Página canônica do motor: **control table completa** (endereços, faixas, defaults), specs elétricas e mecânicas. Consulta obrigatória para qualquer endereço de registrador. |
| ⭐ [DYNAMIXEL SDK — e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) | Visão geral do SDK oficial, instalação e API (PortHandler, PacketHandler, grupos sync). |
| ⭐ [Sync Read/Write Tutorial (Python)](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sync_read_write_tutorial/sync_read_write_tutorial_python/) | Passo a passo de `GroupSyncWrite`/`GroupSyncRead` — base direta do que o `ax12_controller.py` faz (atenção: exemplo é Protocolo 2.0; adaptar endereços para 1.0). |
| [Protocol 1.0 — e-Manual](https://emanual.robotis.com/docs/en/dxl/protocol1/) | Formato dos pacotes (Ping, Read, Write, SyncWrite), checksum e IDs de broadcast — o protocolo que o AX-12 usa. |
| [emanual no GitHub](https://github.com/ROBOTIS-GIT/emanual) | Fonte em Markdown de toda a documentação acima (útil para buscar termos ou abrir issue/PR). |
| [Datasheet AX-12A (PDF)](https://www.generationrobots.com/media/Dynamixel-AX-12-user-manual.pdf) | Manual em PDF para consulta offline / arquivo. |

## 2. SDK e bibliotecas

| Repositório | Descrição |
|---|---|
| ⭐ [ROBOTIS-GIT/DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) | SDK oficial (C/C++/Python/etc.), Protocolo 1.0 e 2.0. É a dependência usada por este projeto. Contém exemplos em `python/tests/protocol1_0/`. |
| [jeremiedecock/pyax12](https://github.com/jeremiedecock/pyax12) | Biblioteca Python leve e madura focada **só no AX-12** (Protocolo 1.0). Boa para consultar como abstrair a control table sem o SDK pesado. |
| [FunPythonEC/AX12_uPy](https://github.com/FunPythonEC/AX12_uPy) | Controle de AX-12 via **MicroPython** (Protocolo 1.0) — referência para microcontrolador embarcado. |
| [simondlevy/dynamixel-ax12](https://github.com/simondlevy/dynamixel-ax12) | Biblioteca Python minimalista para o AX-12 — código curto, bom para entender o protocolo. |
| [rparak/Bioloid-Dynamixel-AX12A](https://github.com/rparak/Bioloid-Dynamixel-AX12A) | Biblioteca open-source para o AX-12A, com foco no kit Bioloid. |
| [jumejume1/AX-12A-servo-library](https://github.com/jumejume1/AX-12A-servo-library) | Biblioteca Arduino/C para o AX-12A — referência para a camada de bytes do protocolo serial. |

## 3. Integração com ROS 2

| Repositório | Descrição |
|---|---|
| ⭐ [CyberDNS/ros2_control_dynamixel_ax12a](https://github.com/CyberDNS/ros2_control_dynamixel_ax12a) | Hardware interface **ros2_control especificamente para AX-12A**. A referência mais próxima caso este projeto migre para o framework `ros2_control`. |
| [ROBOTIS-GIT/dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) | Plugin oficial ROS 2 (`ros2_control`) para Dynamixel. Suporta Humble/Jazzy/Rolling. Abordagem mais robusta/genérica que a nossa. |
| [dynamixel-community/dynamixel_hardware](https://github.com/dynamixel-community/dynamixel_hardware) | Pacote `ros2_control` mantido pela comunidade para Dynamixel — alternativa ao oficial. |
| [schnili/dynamixel_ros2_control](https://github.com/schnili/dynamixel_ros2_control) | Outra hardware interface `ros2_control` para motores Dynamixel. |

## 4. Marcha bípede / humanoides (referência de algoritmo)

> Não usam AX-12 diretamente, mas servem de referência para geração de marcha e arquitetura de controle de pernas.

| Repositório | Descrição |
|---|---|
| [open-rdc/ROS2_Walking_Pattern_Generator](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator) | Gerador de padrão de marcha (walking controller) para humanoides em **ROS 2**. Referência direta para evoluir o `send_gait`. |
| [AshwinderPalSingh/ros2-2D-humanoid-simulation](https://github.com/AshwinderPalSingh/ros2-2D-humanoid-simulation) | Simulador 2D educacional de humanoide com marcha coordenada, em Python puro e em ROS 2. Bom para prototipar marcha antes do hardware. |
| [leggedrobotics/free_gait](https://github.com/leggedrobotics/free_gait) | Arquitetura versátil para controle de robôs com pernas (ETH Zürich). Conceitos de planejamento de passo. |
| [DRCL-USC/Hector_Simulation](https://github.com/DRCL-USC/Hector_Simulation) | Locomoção bípede via MPC baseado em força/momento (ROS/MATLAB). Referência avançada de controle. |
| ⭐ [well-robotics/STRIDE](https://github.com/well-robotics/STRIDE) ([artigo](https://arxiv.org/pdf/2407.02648)) | Plataforma bípede open-source de baixo custo (<US$2000) para pesquisa/ensino, peças off-the-shelf. Escala/objetivo muito parecidos com o Adam — boa referência de arquitetura mecânica + software. |
| [MEVITA](https://arxiv.org/pdf/2508.17684) | Robô bípede open-source montado com componentes de e-commerce + chapa metálica soldada. Referência de design de baixo custo/DIY, mesma filosofia do Adam. |
| [OpenWalker (ROSIN)](https://www.rosin-project.eu/ftp/open-source-balancing-and-walking-control-framework-for-humanoid-robots-in-ros-openwalker) | Framework de balanço/marcha para humanoides baseado em `ros_control`, com módulos prontos para prototipagem rápida de controladores de marcha. |

## 5. ros2_control, MoveIt2 e visualização (RViz/Qt)

> Stack do "digital twin" (visualizar_marcha, gait_bridge, mock.launch.py, adam.ros2_control.xacro). Ainda em mock; vira referência obrigatória nas Fases 2–4 do plano (MoveIt2 + driver real).

| Recurso | Descrição |
|---|---|
| ⭐ [Mock Components — ros2_control docs](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) | Documenta o `mock_components/GenericSystem` usado em `adam.ros2_control.xacro` enquanto o driver real (Fase 4) não existe. |
| ⭐ [joint_trajectory_controller — ros2_control docs](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) | Controller usado em `perna_direita_controller`/`perna_esquerda_controller` (`ros2_controllers.yaml`); explica `allow_partial_joints_goal`, interpolação e tolerâncias. |
| [joint_state_broadcaster — ros2_control docs](https://control.ros.org/jazzy/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html) | Publica `/joint_states` a partir do hardware (mock ou real) — alimenta `robot_state_publisher` e o RViz em `mock.launch.py`. |
| [ros2_control hardware interface types](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html) | Tipos `SystemInterface`/`ActuatorInterface`/`SensorInterface` — referência direta para o `adam_ax12_hardware/AX12System` da Fase 4. |
| ⭐ [MoveIt Setup Assistant — tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) | Passo a passo para gerar o `adam_moveit_config` (Fase 2, ainda pendente): planning groups, SRDF, matriz de auto-colisão. |
| [moveit_configs_utils (MoveItConfigsBuilder)](https://moveit.picknik.ai/main/api/html/namespacemoveit__configs__builder.html) | API para carregar o pacote MoveIt gerado dentro de um launch file Python — usar ao escrever o `demo.launch.py` da Fase 3. |
| [Using Xacro to clean up your code — ROS 2 docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) | Base do `adam.urdf.xacro`/`adam.ros2_control.xacro` (macros, `xacro:arg`, `xacro:if`/`xacro:unless`). |
| [python_qt_binding — index.ros.org](https://index.ros.org/p/python_qt_binding/) | Binding Qt "oficial" do ROS 2 (usado com fallback para PyQt5 puro em `passo_slider.py`). |
| [RViz empty/black no WSL — ros2/rviz#834](https://github.com/ros2/rviz/issues/834) | Mesmo sintoma do Fase 0 deste projeto (tela cinza/preta no WSLg); a solução aplicada (`QT_QPA_PLATFORM=xcb` + `LIBGL_ALWAYS_SOFTWARE=1`) segue essa linha de troubleshooting. |
| [rmw_cyclonedds_cpp — package docs](https://index.ros.org/p/rmw_cyclonedds_cpp/) | RMW usado via `SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')` em `mock.launch.py` para evitar a incompatibilidade de ABI entre `libfastrtps`/`libfastcdr`. |

## 6. Mensagens de diagnóstico

| Recurso | Descrição |
|---|---|
| [diagnostic_msgs — docs.ros.org](https://docs.ros.org/en/ros2_packages/humble/api/diagnostic_msgs/) | `DiagnosticArray`/`DiagnosticStatus`/`KeyValue` publicados pelo `ax12_controller.py` (`/diagnostics`) e consumidos pelo `ax12_monitor.py`. |
| [A Practical Guide to Using ROS Diagnostics — Foxglove](https://foxglove.dev/blog/a-practical-guide-to-using-ros-diagnostics) | Convenções de uso (níveis OK/WARN/ERROR/STALE, agregação) — útil se este projeto migrar para `diagnostic_updater`/`diagnostic_aggregator` em vez de montar o `DiagnosticArray` à mão. |

## 7. Tutoriais e leitura

| Recurso | Descrição |
|---|---|
| [Trossen Robotics — Controlling AX-12 Servos](http://forums.trossenrobotics.com/tutorials/misc.php?do=printfriendly&e=3275) | Tutorial clássico explicando o pacote serial do AX-12 byte a byte (start bytes `0xFF 0xFF`, ID, instrução, checksum). Ótimo para entender o protocolo "na unha". |
| [Dynamixel AX-12A e Arduino — porta serial](https://robottini.altervista.org/dynamixel-ax-12a-and-arduino-how-to-use-the-serial-port) | Como falar com o AX-12 pela serial via Arduino — útil para depurar a camada física (half-duplex). |
| [ArbotiX RoboController](https://vanadiumlabs.github.io/arbotix/) | Documentação do controlador ArbotiX, comum em projetos AX-12 legados. |

---

> Mantenha esta lista como **ponteiros** (camada "reference" da gestão de conhecimento): nada de copiar código para cá — linke a fonte e descreva em uma linha por que ela importa.
