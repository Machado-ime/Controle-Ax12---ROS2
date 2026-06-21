# Instalação e primeira execução

Guia completo, do zero, para instalar o pacote e rodar os quatro cenários do projeto. Para um
resumo rápido, veja o [README](../README.md).

## Pré-requisitos

Ver tabela em [README — Pré-requisitos](../README.md#pré-requisitos).

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

> `ROS_DOMAIN_ID` deve ser **igual** nas duas máquinas (Raspberry Pi e PC de comando). O valor
> `0` é o padrão; escolha outro (0–101) se houver outros robôs ou outra rede ROS no mesmo
> ambiente.

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

**4. Visualizar no RViz (sem hardware):**

```bash
ros2 launch ax12_control visualizar_marcha.launch.py matriz:=cin_inve   # ou otimizada
```

Abre o RViz com o modelo 3D do Adam (URDF lido automaticamente do pacote `adam_urdf`) mais uma
janela Qt com um slider para escolher manualmente a etapa da marcha exibida. Para avançar
sozinho no tempo em vez de manual, passe `passo_s:=0.5` (segundos por etapa). Não exige
Raspberry Pi nem motores.

`Ctrl+C` em qualquer lado encerra com segurança — o controlador desliga o torque de todos os
motores ao sair.

## Próximos passos

- Entendendo o sistema por dentro: [docs/arquitetura.md](arquitetura.md).
- Deu algum problema? [docs/troubleshooting.md](troubleshooting.md).
- Quer trocar a marcha? Veja "Criar uma marcha nova" em
  [docs/arquitetura.md](arquitetura.md#criar-uma-marcha-nova).
