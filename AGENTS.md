# AGENTS.md — instruções para agentes de IA

## Sobre o projeto

`ax12_control` é um pacote ROS 2 (Jazzy, `ament_python`) que controla os 10 servomotores
Dynamixel AX-12 das pernas do robô bípede **Adam** via Protocolo 1.0 (`dynamixel_sdk`), entre
uma Raspberry Pi (motores) e um PC de comando (gerador de marcha), comunicando-se por
DDS/Wi-Fi. Veja [docs/arquitetura.md](docs/arquitetura.md) para como o sistema funciona por
dentro.

## Como buildar e rodar

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
- Nomes de junta seguem o URDF do pacote `adam`: `{lado}_{movimento}_{segmento}_{N}` (ex.:
  `pd_picht_tornozelo_3`). O `N` é o ID de projeto no URDF, **não** é o ID físico do motor no
  barramento — os IDs reais vivem em `joint_map` (`ax12_control/ax12_controller.py`).

## Mapa do repositório

- `ax12_control/` — nós ROS (controlador de hardware, gerador de marcha, monitor, visualizador
  RViz, ponte para `ros2_control`) e as matrizes de marcha (`*.yaml`) usadas em runtime.
- `launch/` — launch files.
- `docs/` — documentação: `install.md` (tutorial), `troubleshooting.md` (guia),
  `ref/` (referência), `adr.md` (explicação/diário de bordo).
- `legacy/` — versões antigas, mantidas só como referência histórica. Não usar nem editar.
- `matrizes-de-movimento/` — matrizes de marcha originais/fonte (MATLAB), não instaladas pelo
  build; as cópias usadas em runtime estão em `ax12_control/*.yaml`.

## Regras importantes

- O pacote `adam` (URDF, meshes, `ros2_control`) mora em outro workspace/repositório — não
  existe dentro deste repo. Não invente caminhos para ele.
- `package.xml` e `setup.py` declaram as dependências reais — ao adicionar um nó/launch novo
  que importa um pacote ROS, declare a dependência ali também.
- Atualize o README e o `CHANGELOG.md` no mesmo PR que muda comportamento visível.
- Aconteceu algo relevante (decisão, problema, teste)? Registre em [docs/adr.md](docs/adr.md).
- Resolveu um problema? Registre em [docs/troubleshooting.md](docs/troubleshooting.md), com a
  mensagem de erro literal.
