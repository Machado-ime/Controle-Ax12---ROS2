# Controle AX-12 — ROS 2

Pacote ROS 2 (`ax12_control`) para controle dos servomotores **Dynamixel AX-12** de um robô bípede. O sistema é distribuído em duas máquinas que se comunicam pela rede (DDS/Wi-Fi):

- **Raspberry Pi** (conectada aos motores via USB): roda o nó `ax12_controller`, que recebe trajetórias e escreve nos motores.
- **PC de comando**: roda o `send_gait`, que publica a sequência de marcha.

```
   PC de comando                          Raspberry Pi
┌─────────────────┐                  ┌──────────────────────┐      USB/serial
│    send_gait    │ ──────────────▶  │   ax12_controller    │ ───────────────▶  Motores AX-12
└─────────────────┘  /joint_trajectory └────────────────────┘   (1 Mbps, Protocolo 1.0)
                    (JointTrajectory,
                  QoS BEST_EFFORT, depth=1)
```

## Estrutura do repositório

```
├── package.xml              # Manifesto ROS 2 (dependências)
├── setup.py                 # Entry points dos executáveis
├── setup.cfg
├── resource/                # Marcador do ament index
├── ax12_control/            # Código-fonte (módulo Python)
│   ├── ax12_controller.py   # Nó de interface de hardware (Raspberry Pi)
│   └── send_gait.py         # Cliente de marcha (PC)
├── docs/
│   └── bizuario_ros.md      # Cola de comandos úteis do ROS 2
└── legacy/                  # Versões antigas (referência, fora do build)
```

## Pré-requisitos

- Ubuntu com **ROS 2** instalado (testado com Humble) nas duas máquinas, na mesma rede e com o mesmo `ROS_DOMAIN_ID`.
- Na Raspberry Pi: **Dynamixel SDK**:

```bash
sudo apt install ros-$ROS_DISTRO-dynamixel-sdk
# ou, se não houver pacote apt para sua distro:
pip3 install dynamixel-sdk
```

## Instalação (nas duas máquinas)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Machado-ime/Controle-Ax12---ROS2.git
cd ~/ros2_ws
colcon build --packages-select ax12_control
source install/setup.bash
```

> Adicione `source ~/ros2_ws/install/setup.bash` ao seu `~/.bashrc` para não precisar repetir a cada terminal.

## Uso

**1. Na Raspberry Pi** (com os motores ligados na porta `/dev/ttyACM0`):

```bash
ros2 run ax12_control ax12_controller
```

O nó abre a serial, liga o torque dos motores mapeados e fica escutando o tópico `/joint_trajectory`.

**2. No PC de comando:**

```bash
ros2 run ax12_control send_gait
```

Publica o ciclo de marcha em loop. `Ctrl+C` para parar (o controlador desliga o torque ao ser encerrado).

## Mapa de juntas

| Junta                  | ID do motor |
|------------------------|-------------|
| `PD_tornozelo_pitch_1` | 1           |
| `PE_tornozelo_pitch_2` | 2           |
| `PD_joelho_pitch_5`    | 5           |
| `PE_joelho_pitch_6`    | 6           |
| `PD_quadril_pitch_7`   | 7           |
| `PE_quadril_pitch_8`   | 8           |

Os demais motores (rolls, braços e pescoço, IDs 3–4 e 9–18) estão comentados no código e podem ser reativados em `ax12_control/ax12_controller.py` (`joint_map`) e `ax12_control/send_gait.py` (`matriz_movimento` e `nomes_juntas`).

## Detalhes técnicos

- **Mensagem**: `trajectory_msgs/JointTrajectory` — posições em **radianos**, velocidades em **rad/s**.
- **QoS**: `BEST_EFFORT`, `KEEP_LAST`, `depth=1` nos dois lados (precisa ser igual!). Escolha deliberada para Wi-Fi: comando perdido é descartado em vez de reenviado atrasado.
- **Conversões** (feitas no controlador):
  - Posição: ±2,618 rad (±150°) → 0–1023 (resolução do AX-12).
  - Velocidade: rad/s × 86,03 → 1–1023.
- A escrita das posições usa `GroupSyncWrite` para que todos os motores iniciem o movimento simultaneamente.

## Diagnóstico

Comandos úteis de inspeção (nós, tópicos, QoS) em [docs/bizuario_ros.md](docs/bizuario_ros.md).

## Licença

[MIT](LICENSE)
