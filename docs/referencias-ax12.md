# Referências — controle do AX-12

Coletânea de repositórios e documentação úteis para o controle dos servos Dynamixel AX-12 (Protocolo 1.0). Serve de ponto de partida ao consultar dúvidas de hardware, SDK ou marcha.

> **Stack deste projeto:** `dynamixel_sdk` (Python) + `GroupSyncWrite` + Protocolo 1.0, sob ROS 2 Jazzy. As referências mais alinhadas estão marcadas com ⭐.

- **Data da última atualização:** 2026-06-16

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

## 5. Tutoriais e leitura

| Recurso | Descrição |
|---|---|
| [Trossen Robotics — Controlling AX-12 Servos](http://forums.trossenrobotics.com/tutorials/misc.php?do=printfriendly&e=3275) | Tutorial clássico explicando o pacote serial do AX-12 byte a byte (start bytes `0xFF 0xFF`, ID, instrução, checksum). Ótimo para entender o protocolo "na unha". |
| [Dynamixel AX-12A e Arduino — porta serial](https://robottini.altervista.org/dynamixel-ax-12a-and-arduino-how-to-use-the-serial-port) | Como falar com o AX-12 pela serial via Arduino — útil para depurar a camada física (half-duplex). |
| [ArbotiX RoboController](https://vanadiumlabs.github.io/arbotix/) | Documentação do controlador ArbotiX, comum em projetos AX-12 legados. |

---

> Mantenha esta lista como **ponteiros** (camada "reference" da gestão de conhecimento): nada de copiar código para cá — linke a fonte e descreva em uma linha por que ela importa.
