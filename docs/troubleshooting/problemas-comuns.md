# Problemas comuns

- **Data da última atualização:** 2026-06-16
- **Robô:** Adam (bípede AX-12 / ROS 2 Jazzy)

---

| Sintoma | Causa provável | Solução |
|---|---|---|
| `Nao foi possivel abrir a porta` | Porta errada ou sem permissão | Confira com `ls /dev/ttyACM* /dev/ttyUSB*`. Permissão: `sudo usermod -aG dialout $USER` e relogue. Parâmetro: `--ros-args -p device:=/dev/ttyUSB0` |
| `send_gait` roda mas o robô não se mexe | Máquinas em domínios diferentes ou QoS incompatível | `echo $ROS_DOMAIN_ID` deve ser igual nas duas. `ros2 topic info /joint_trajectory -v` deve listar 1 publisher e 1 subscription |
| `Package 'ax12_control' not found` | Workspace não carregado | `source ~/ax12_control_ws/install/setup.bash` — confira se está no `~/.bashrc` |
| `ERRO no arquivo de marcha (...)` | YAML malformado, chave faltando ou matriz inconsistente | Corrija o `.yaml` — veja o formato em [arquitetura.md](../arquitetura.md#marchas-arquivos-yaml) |
| `arquivo de marcha nao encontrado` | Nome errado no parâmetro ou pacote não recompilado | Confira o nome (sem pasta, sem extensão) e rode `colcon build --packages-select ax12_control` |
| Motor não responde ao ligar o torque | Motor sem alimentação, ID errado ou cabo solto | Wizard 2.0 com o barramento direto no PC ajuda a confirmar quais IDs estão vivos |
| Motores travam / vão para posição estranha ao ligar | Fonte insuficiente para o pico de corrente simultâneo | O código já escalona com 50 ms entre motores; se persistir, verifique a fonte (corrente de pico de todos os AX-12 somada) |
| Wi-Fi da Raspberry Pi cai ao iniciar a marcha | Pico de corrente dos motores derruba a tensão da Pi, que reseta o Wi-Fi | Fonte separada para a Pi (5 V independente dos motores). Paliativo imediato: rode o controlador dentro do `tmux` — se o SSH cair, o nó continua rodando |
| SSH cai e os motores param | Sem `tmux`, o SIGHUP mata o processo e o `destroy_node()` desliga o torque | `tmux new -s robo` antes de rodar o controlador |
| Nós não se enxergam no Wi-Fi | Firewall ou multicast bloqueado | `ros2 multicast receive` / `ros2 multicast send` para testar. Libere: `sudo ufw allow in proto udp` |
| Motor em overload frequente (torque > 80%) | Carga mecânica acima do limite ou ganho muito alto | Reduza a velocidade (`-p velocidade_padrao:=50`), verifique alinhamento mecânico; monitore com `ax12_monitor` |

---

Mais comandos de diagnóstico em [bizuario_ros.md](../bizuario_ros.md).
