# Controle AX-12 — ROS 2

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12%2B-yellow)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Pacote ROS 2 (`ax12_control`) para controle dos servomotores Dynamixel AX-12 do robô bípede **Adam** via rede Wi-Fi.

```
   PC de comando                              Raspberry Pi
┌─────────────────┐    /joint_trajectory   ┌──────────────────┐    USB/serial     ┌──────────────┐
│    send_gait    │ ─────────────────────▶ │  ax12_controller │ ────────────────▶ │ Motores AX-12│
└─────────────────┘   (JointTrajectory,    └──────────────────┘  1 Mbps, Proto 1.0└──────────────┘
        ▲              QoS BEST_EFFORT,
        │                 depth = 1)
   *.yaml (marcha)
```

## Visão geral

O sistema é distribuído entre duas máquinas ligadas pela rede:

- **Raspberry Pi** — conectada aos motores via USB, roda o `ax12_controller` (único processo que acessa o barramento serial).
- **PC de comando** — roda o `send_gait`, que lê a marcha de um arquivo `.yaml` e publica os passos via DDS/Wi-Fi.

Estado atual: **10 motores ativos** nas pernas (pitch e roll de tornozelo, pitch de joelho, pitch e roll de quadril). Marchas configuráveis por arquivo YAML sem recompilar o código. Também é possível visualizar a marcha no RViz sem hardware nenhum (digital twin) — veja [docs/install.md](docs/install.md).

## Pré-requisitos

| Requisito | Versão | Instalar |
|---|---|---|
| Ubuntu | 24.04 | — |
| ROS 2 | Jazzy | [guia oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| colcon | — | `sudo apt install python3-colcon-common-extensions` |
| Dynamixel SDK | 3.x | `sudo apt install ros-jazzy-dynamixel-sdk` ¹ |
| PyYAML | — | `sudo apt install -y python3-yaml` ² |
| RViz2 + robot_state_publisher | — | `sudo apt install ros-jazzy-rviz2 ros-jazzy-robot-state-publisher` ³ |
| python_qt_binding | — | `sudo apt install ros-jazzy-python-qt-binding` ³ |

> ¹ Só na Raspberry Pi. &nbsp; ² Só no PC de comando. &nbsp; ³ Só para visualizar a marcha no RViz (sem hardware).

Passo a passo completo de instalação e primeira execução: **[docs/install.md](docs/install.md)**.

## Estrutura do repositório

```
Controle-Ax12---ROS2/
├── AGENTS.md                 # instruções para agentes de IA
├── CHANGELOG.md              # histórico de mudanças relevantes
├── README.md                 # este arquivo — quickstart
├── LICENSE
├── .github/                  # CODEOWNERS, templates de issue/PR, CONTRIBUTING
├── src/
│   ├── ax12_control/          # o pacote ROS deste README (package.xml, setup.py, launch/, código)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── launch/
│   │   │   ├── visualizar_marcha.launch.py
│   │   │   └── controle_manual.launch.py
│   │   └── ax12_control/      # módulo Python (nome repetido = convenção ament_python)
│   │       ├── ax12_controller.py   # nó de hardware (Raspberry Pi)
│   │       ├── send_gait.py         # gerador de marcha (PC de comando)
│   │       ├── ax12_monitor.py      # painel de telemetria no terminal (PC)
│   │       ├── visualizar_marcha.py # nó de visualização RViz (sem hardware)
│   │       ├── passo_slider.py      # janela Qt p/ escolher a etapa da marcha manualmente
│   │       ├── controle_manual.py   # janela Qt: jog manual dos motores reais + RViz junto
│   │       ├── gait_bridge.py       # ponte send_gait -> ros2_control (Caso 2, MoveIt)
│   │       ├── adam.rviz            # config RViz pré-configurado para o Adam
│   │       ├── otimizada.yaml       # marcha padrão (6 juntas, pitch)
│   │       └── cin_inve.yaml        # marcha por cinemática inversa (8 juntas)
│   ├── adam_urdf/               # pacote ROS (ament_cmake): URDF, meshes e launch do Adam
│   └── adam_moveit_config/     # pacote MoveIt2 gerado p/ planejamento de movimento
└── docs/
    ├── install.md           # tutorial: instalação + primeira execução
    ├── troubleshooting.md   # guia: problemas conhecidos e soluções
    ├── adr.md                # explicação: diário de bordo do projeto
    └── ref/                  # referência: links externos e cola de comandos
        ├── referencias-ax12.md
        └── comandos-ros.md
```

> O `colcon` encontra o pacote recursivamente — clonar este repositório dentro do `src/` de
> um workspace funciona normalmente, mesmo com o `src/` extra deste repositório (vira
> `<workspace>/src/Controle-Ax12---ROS2/src/ax12_control/`).

## Estrutura ROS

| Nó | Roda em | Função |
|---|---|---|
| `ax12_controller` | Raspberry Pi | Único processo no barramento serial — escreve posição/velocidade e lê telemetria dos 10 motores |
| `send_gait` | PC de comando | Lê a marcha de um `.yaml` e publica `/joint_trajectory` |
| `ax12_monitor` | PC de comando | Painel de telemetria ao vivo no terminal |
| `visualizar_marcha` + `passo_slider` | PC de comando | Digital twin no RViz sem hardware |
| `controle_manual` | Raspberry Pi | Jog manual por slider — move o motor real e o RViz ao mesmo tempo (RViz via telemetria real do `ax12_controller`) |
| `gait_bridge` | PC de comando | Ponte para `ros2_control`/MoveIt2 (pacotes `adam_urdf`/`adam_moveit_config`, em `src/`) |

| Tópico | Tipo | QoS |
|---|---|---|
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | BEST_EFFORT / depth 1 |
| `/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RELIABLE |
| `/hardware_errors` | `std_msgs/String` | RELIABLE |

Detalhamento completo (fluxo de inicialização, conversões, mapa de juntas, tolerância a falhas): **[docs/arquitetura.md](docs/arquitetura.md)**.

## Recursos oficiais

- [AX-12A — e-Manual (ROBOTIS)](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) — control table completa do motor.
- [DYNAMIXEL SDK — e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) — SDK oficial usado por este pacote.
- [ROBOTIS-GIT/DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) — repositório do SDK (dependência deste projeto).

Lista completa de referências (hardware, ros2_control, MoveIt2, projetos de robôs bípedes): **[docs/ref/referencias-ax12.md](docs/ref/referencias-ax12.md)**.

## Links gerais

- [docs/install.md](docs/install.md) — instalação e primeira execução
- [docs/troubleshooting.md](docs/troubleshooting.md) — problemas conhecidos e soluções
- [docs/arquitetura.md](docs/arquitetura.md) — como o sistema funciona por dentro
- [docs/adr.md](docs/adr.md) — diário de bordo (decisões, testes, aprendizados)
- [docs/ref/](docs/ref/) — referências externas e cola de comandos ROS 2
- [src/README.md](src/README.md) — organograma e comandos dos 3 pacotes ROS
- [.github/CONTRIBUTING.md](.github/CONTRIBUTING.md) — como contribuir
- [AGENTS.md](AGENTS.md) — instruções para agentes de IA

## Licença

MIT — veja [LICENSE](LICENSE).
