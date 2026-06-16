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

Estado atual: **8 motores ativos** nas pernas (pitch e roll de tornozelo, pitch de joelho, pitch de quadril). Marchas configuráveis por arquivo YAML sem recompilar o código.

## Pré-requisitos

| Requisito | Versão | Instalar |
|---|---|---|
| Ubuntu | 24.04 | — |
| ROS 2 | Jazzy | [guia oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| colcon | — | `sudo apt install python3-colcon-common-extensions` |
| Dynamixel SDK | 3.x | `sudo apt install ros-jazzy-dynamixel-sdk` ¹ |
| PyYAML | — | `sudo apt install -y python3-yaml` ² |

> ¹ Só na Raspberry Pi. &nbsp; ² Só no PC de comando.

## Instalação

```bash
mkdir -p ~/ax12_control_ws/src
cd ~/ax12_control_ws/src
git clone https://github.com/Machado-ime/Controle-Ax12---ROS2.git

cd ~/ax12_control_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ax12_control

echo "source /opt/ros/jazzy/setup.bash"            >> ~/.bashrc
echo "source ~/ax12_control_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0"                       >> ~/.bashrc
source ~/.bashrc
```

> `ROS_DOMAIN_ID` deve ser **igual** nas duas máquinas. O valor `0` é o padrão; escolha outro (0–101) se houver outros robôs ou outra rede ROS no mesmo ambiente.

Para atualizar depois de um `git pull`:

```bash
colcon build --packages-select ax12_control && source ~/.bashrc
```

## Como rodar

**1. Raspberry Pi** (motores ligados e fonte de alimentação conectada):

```bash
ros2 run ax12_control ax12_controller
```

Saída esperada: porta aberta → torque ligado motor a motor → `Pronto para receber comandos`.

**2. PC de comando:**

```bash
ros2 run ax12_control send_gait                                # marcha padrão (otimizada.yaml)
ros2 run ax12_control send_gait --ros-args -p matriz:=cin_inve # outra marcha
```

**3. Telemetria** (opcional, em outro terminal no PC):

```bash
ros2 run ax12_control ax12_monitor
```

`Ctrl+C` em qualquer lado encerra com segurança — o controlador desliga o torque de todos os motores ao sair.

## Estrutura

```
Controle-Ax12---ROS2/
├── ax12_control/
│   ├── ax12_controller.py   # nó de hardware (Raspberry Pi)
│   ├── send_gait.py         # gerador de marcha (PC de comando)
│   ├── ax12_monitor.py      # painel de telemetria no terminal (PC)
│   ├── otimizada.yaml       # marcha padrão (6 juntas, pitch)
│   └── cin_inve.yaml        # marcha por cinemática inversa (8 juntas)
├── docs/
│   ├── arquitetura.md       # como o sistema funciona por dentro
│   ├── referencias-ax12.md  # links de SDKs, docs e projetos de referência
│   ├── troubleshooting/
│   │   └── problemas-comuns.md
│   └── bizuario_ros.md      # cola de comandos ROS 2 para diagnóstico
├── matrizes de movimento/   # origem/referência das marchas (não instalado no build)
│   └── otimização.h         # header C de protótipo legado (18 motores)
├── legacy/                  # versões antigas para referência histórica
├── package.xml
└── setup.py
```

Documentação técnica aprofundada: [docs/arquitetura.md](docs/arquitetura.md)  
Problemas e soluções: [docs/troubleshooting/problemas-comuns.md](docs/troubleshooting/problemas-comuns.md)  
Referências externas (SDKs, docs, projetos): [docs/referencias-ax12.md](docs/referencias-ax12.md)

## Problemas conhecidos

Ver [docs/troubleshooting/problemas-comuns.md](docs/troubleshooting/problemas-comuns.md).

## Licença

MIT — veja [LICENSE](LICENSE).
