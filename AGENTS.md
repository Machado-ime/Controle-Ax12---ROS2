# AGENTS.md — instruções para agentes de IA

## Sobre o projeto

`ax12_control` é um pacote ROS 2 (Jazzy, `ament_python`) que controla os 10 servomotores
Dynamixel AX-12 das pernas do robô bípede **Adam** via Protocolo 1.0 (`dynamixel_sdk`), entre
uma Raspberry Pi (motores) e um PC de comando (gerador de marcha), comunicando-se por
DDS/Wi-Fi. Veja [docs/arquitetura.md](docs/arquitetura.md) para como o sistema funciona por
dentro.

## Como buildar e rodar

O repositório inteiro é um mini-workspace: `src/ax12_control/`, `src/adam_urdf/` e
`src/adam_moveit_config/` são 3 pacotes ROS lado a lado. Buildar a partir da raiz do
repositório funciona normalmente — o `colcon` encontra cada `package.xml` recursivamente:

```bash
colcon build --packages-select ax12_control
source install/setup.bash
ros2 run ax12_control ax12_controller   # Raspberry Pi
ros2 run ax12_control send_gait         # PC de comando
```

Sem hardware (RViz, digital twin): `ros2 launch ax12_control visualizar_marcha.launch.py`.
Passo a passo completo: [docs/install.md](docs/install.md).

Não há suíte de testes própria além dos checks padrão do `ament` (`ament_flake8`,
`ament_pep257`, `ament_copyright`, declarados em `package.xml`/`setup.py`): `colcon test
--packages-select ax12_control`.

## Convenções

- Commits em português, modo imperativo, frase curta ("Adiciona X", "Corrige Y", "Atualiza
  Z") — não é Conventional Commits, é a convenção real já usada neste repositório.
- Nomes de arquivo: minúsculas, hífen, sem acento nem espaço.
- Nomes de junta seguem o URDF do pacote `adam_urdf`: `{lado}_{movimento}_{segmento}_{N}` (ex.:
  `pd_picht_tornozelo_3`). O `N` é o ID de projeto no URDF, **não** é o ID físico do motor no
  barramento — os IDs reais vivem em `joint_map`
  (`src/ax12_control/ax12_control/ax12_controller.py`).

## Mapa do repositório

Este repositório é um mini-workspace com 3 pacotes ROS lado a lado em `src/`:

- `src/ax12_control/` — o pacote deste README/AGENTS: `package.xml`, `setup.py`, `launch/` e o
  módulo Python `ax12_control/` (nós, matrizes de marcha `*.yaml` usadas em runtime).
- `src/adam_urdf/` — pacote `ament_cmake` com URDF, meshes e launch files do robô Adam.
- `src/adam_moveit_config/` — pacote MoveIt2 gerado (planning groups, SRDF, controllers).
- `src/matrizes-de-movimento/` — matrizes de marcha originais/fonte (MATLAB), não instaladas
  pelo build; as cópias usadas em runtime estão em `src/ax12_control/ax12_control/*.yaml`.
- `docs/` — documentação: `install.md` (tutorial), `troubleshooting.md` (guia),
  `ref/` (referência), `adr.md` (explicação/diário de bordo). Documenta principalmente o
  pacote `ax12_control`; `adam_urdf`/`adam_moveit_config` ainda não têm docs próprias aqui.

## Regras importantes

- `package.xml` e `setup.py` (em `src/ax12_control/`) declaram as dependências reais — ao
  adicionar um nó/launch novo que importa um pacote ROS, declare a dependência ali também.
- Atualize o README e o `CHANGELOG.md` no mesmo PR que muda comportamento visível.
- Aconteceu algo relevante (decisão, problema, teste)? Registre em [docs/adr.md](docs/adr.md).
- Resolveu um problema? Registre em [docs/troubleshooting.md](docs/troubleshooting.md), com a
  mensagem de erro literal.
