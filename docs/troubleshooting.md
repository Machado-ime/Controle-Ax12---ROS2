# Troubleshooting

- **Última atualização:** 2026-06-21
- **Robô:** Adam (bípede AX-12 / ROS 2 Jazzy)

## Referência rápida

| Sintoma | Causa provável | Solução |
|---|---|---|
| `Nao foi possivel abrir a porta` | Porta errada ou sem permissão | Confira com `ls /dev/ttyACM* /dev/ttyUSB*`. Permissão: `sudo usermod -aG dialout $USER` e relogue. Parâmetro: `--ros-args -p device:=/dev/ttyUSB0` |
| `send_gait` roda mas o robô não se mexe | Máquinas em domínios diferentes ou QoS incompatível | `echo $ROS_DOMAIN_ID` deve ser igual nas duas. `ros2 topic info /joint_trajectory -v` deve listar 1 publisher e 1 subscription |
| `Package 'ax12_control' not found` | Workspace não carregado | `source ~/dev/Controle-Ax12---ROS2/install/setup.bash` — confira se está no `~/.bashrc` |
| `ERRO no arquivo de marcha (...)` | YAML malformado, chave faltando ou matriz inconsistente | Corrija o `.yaml` — veja o formato em [arquitetura.md](arquitetura.md#marchas-arquivos-yaml) |
| `arquivo de marcha nao encontrado` | Nome errado no parâmetro ou pacote não recompilado | Confira o nome (sem pasta, sem extensão) e rode `colcon build --packages-select ax12_control` |
| Motor não responde ao ligar o torque | Motor sem alimentação, ID errado ou cabo solto | Wizard 2.0 com o barramento direto no PC ajuda a confirmar quais IDs estão vivos |
| Motores travam / vão para posição estranha ao ligar | Fonte insuficiente para o pico de corrente simultâneo | O código já escalona com 50 ms entre motores; se persistir, verifique a fonte (corrente de pico de todos os AX-12 somada) |
| Wi-Fi da Raspberry Pi cai ao iniciar a marcha | Pico de corrente dos motores derruba a tensão da Pi, que reseta o Wi-Fi | Ver runbook completo abaixo: [Raspberry Pi — Wi-Fi cai ao iniciar o ciclo de marcha](#raspberry-pi--wi-fi-cai-ao-iniciar-o-ciclo-de-marcha) |
| SSH cai e os motores param | Sem `tmux`, o SIGHUP mata o processo e o `destroy_node()` desliga o torque | `tmux new -s robo` antes de rodar o controlador |
| Nós não se enxergam no Wi-Fi | Firewall ou multicast bloqueado | `ros2 multicast receive` / `ros2 multicast send` para testar. Libere: `sudo ufw allow in proto udp` |
| Motor em overload frequente (torque > 80%) | Carga mecânica acima do limite ou ganho muito alto | Reduza a velocidade (`-p velocidade_padrao:=50`), verifique alinhamento mecânico; monitore com `ax12_monitor` |

---

## Raspberry Pi — Wi-Fi cai ao iniciar o ciclo de marcha

- **Data:** 2026-06-16
- **Robô / equipamento:** Adam — Raspberry Pi + motores AX-12 na mesma fonte 5 V
- **Sintoma:** o Wi-Fi da Raspberry Pi para de funcionar assim que o `send_gait` começa a
  enviar passos; sem Wi-Fi, o DDS perde o link com o PC e o movimento para.
- **Mensagem de erro (literal):** nenhuma — o link de rede simplesmente cai (sem exceção no
  processo ROS).
- **Contexto:** acontece no início de cada ciclo de marcha, quando todos os motores saem do
  repouso simultaneamente — o pico de corrente de partida é máximo nesse momento.

### Hipóteses testadas
- [x] Pico de corrente dos motores → afundamento de tensão no barramento 5 V → Pi entra em
  throttling / reseta o módulo Wi-Fi → **confirmado como causa raiz**
- [x] EMI do PWM dos AX-12 na faixa de 2,4 GHz interferindo no Wi-Fi → possível fator
  agravante, mas o afundamento de tensão é suficiente para explicar o reset
- [x] Ruído da PSU chaveada sob carga alta → pode piorar o afundamento, mas não é a causa
  isolada

### Causa raiz

A fonte de 5 V alimenta **tanto** a Raspberry Pi quanto os motores AX-12 pelo mesmo barramento.
No início do ciclo de marcha, a corrente de pico de todos os motores somada provoca um
afundamento de tensão que faz a Pi entrar em modo de undervoltage — o módulo Wi-Fi é um dos
primeiros subsistemas a ser afetado.

Para confirmar: rodar `vcgencmd get_throttled` logo após o evento. O valor `0x50000` (bits 16
e 18) indica que houve afundamento de tensão no passado desde o boot; `0x50005` indica que
está acontecendo agora.

```bash
vcgencmd get_throttled
# 0x50000 → afundamento JÁ ocorreu (bits de histórico)
# 0x00000 → sem evento de undervoltage
```

### Solução

**Definitiva:** alimentar a Raspberry Pi com uma fonte 5 V dedicada e independente dos
motores. Os motores ficam na fonte principal (12 V com regulador, ou fonte própria de 5 V com
corrente suficiente para o pico dos AX-12); a Pi em outra entrada de 5 V.

**Paliativa (sem trocar a fonte):** rodar o controlador dentro do `tmux`. Assim, se o Wi-Fi
cair e a sessão SSH se perder, o processo continua vivo na Pi — o SIGHUP que normalmente
mataria o `ros2 run` não chega ao nó.

```bash
# Na Raspberry Pi, antes de rodar o controlador:
tmux new -s robo
ros2 run ax12_control ax12_controller

# Para reconectar ao tmux após queda do SSH:
tmux attach -t robo
```

### Como evitar ou detectar antes

- Medir a tensão no conector da Pi com multímetro durante o início da marcha — queda abaixo de
  4,75 V confirma o problema.
- Monitorar `vcgencmd get_throttled` periodicamente via cron ou script na Pi.
- Sempre usar `tmux` na Pi para desacoplar o processo do ROS da sessão SSH, mesmo depois de
  resolver a fonte — protege contra qualquer queda de rede futura.
- Ao dimensionar a fonte dos motores, somar a corrente de stall de todos os AX-12 ativos
  (≈ 1,5 A cada) como pior caso de pico de partida.

---

Mais comandos de diagnóstico em [ref/comandos-ros.md](ref/comandos-ros.md).
