# Nó da cabeça (`cabeca`)

Versão resumida do `ax12_controller` para os 2 motores AX-12 da cabeça. Código:
`src/ax12_control/ax12_control/cabeca.py`.

| Junta | ID | Movimento | Limite |
|---|---|---|---|
| `cabeca_yaw_1` | 1 | gira para os lados | ±150° (sem medição ainda) |
| `cabeca_picht_2` | 2 | inclina para cima/baixo | ±150° (sem medição ainda) |

Os nomes ainda não existem no URDF do `adam_description`. Quem definir os nomes lá troca
também o dicionário `JUNTAS` no topo do `cabeca.py`.

## Barramento próprio

Na corrente das pernas, os IDs 1 e 2 **já são** os rolls de tornozelo
(`pd_roll_tornozelo_1` e `pe_roll_tornozelo_2`). Por isso a cabeça precisa de um adaptador USB
próprio:

- no mesmo barramento, um comando para a cabeça moveria os tornozelos;
- dois processos não podem abrir a mesma porta serial ao mesmo tempo.

Quando os dois nós rodam juntos, passe a porta da cabeça pelo parâmetro `device`.

## Como rodar

```bash
colcon build --packages-select ax12_control
source install/setup.bash
ros2 run ax12_control cabeca --ros-args -p device:=/dev/ttyUSB0
```

Comando de teste (cabeça 20° para um lado e 10° para baixo):

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [cabeca_yaw_1, cabeca_picht_2], points: [{positions: [0.35, -0.17]}]}"
```

Parâmetros: `device` (`/dev/ttyACM0`), `baudrate` (`1000000`), `velocidade_padrao` (`100`,
usada quando a mensagem vem sem `velocities`), `taxa_leitura` (`5.0` Hz; `0` desliga o
`/joint_states`) e `pausa_pos_abertura` (`0.2` s, necessária com a OpenCR).

## O que ficou de fora em relação ao `ax12_controller`

- **Reconexão automática:** se a porta cair, o nó para de escrever e avisa no log. Para
  voltar, reinicie o nó.
- **Modo observador** (`ligar_torque:=false`): o nó sempre liga o torque ao iniciar e o
  desliga ao sair.
- **`/diagnostics` e `/hardware_errors`:** os avisos saem só no log do nó. O `/joint_states`
  publica posição e velocidade, sem esforço.
- **Detecção de perda no barramento:** um motor que não responde sai do `/joint_states`
  naquela leitura e gera um aviso no log, no máximo um a cada 5 s.

A tabela de controle, as conversões rad↔unidades e as exceções de porta caída são importadas
do `ax12_controller`, para os dois nós não divergirem nesses números.
