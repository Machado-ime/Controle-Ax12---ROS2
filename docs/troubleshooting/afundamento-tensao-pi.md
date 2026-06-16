# Raspberry Pi — Wi-Fi cai ao iniciar o ciclo de marcha

- **Data:** 2026-06-16
- **Robô / equipamento:** Adam — Raspberry Pi + motores AX-12 na mesma fonte 5 V
- **Sintoma:** o Wi-Fi da Raspberry Pi para de funcionar assim que o `send_gait` começa a enviar passos; sem Wi-Fi, o DDS perde o link com o PC e o movimento para.
- **Contexto:** acontece no início de cada ciclo de marcha, quando todos os motores saem do repouso simultaneamente — o pico de corrente de partida é máximo nesse momento.

## Hipóteses testadas

- [x] Pico de corrente dos motores → afundamento de tensão no barramento 5 V → Pi entra em throttling / reseta o módulo Wi-Fi → **confirmado como causa raiz**
- [x] EMI do PWM dos AX-12 na faixa de 2,4 GHz interferindo no Wi-Fi → possível fator agravante, mas o afundamento de tensão é suficiente para explicar o reset
- [x] Ruído da PSU chaveada sob carga alto → pode piorar o afundamento, mas não é a causa isolada

## Causa raiz

A fonte de 5 V alimenta **tanto** a Raspberry Pi quanto os motores AX-12 pelo mesmo barramento. No início do ciclo de marcha, a corrente de pico de todos os motores somada provoca um afundamento de tensão que faz a Pi entrar em modo de undervoltage — o módulo Wi-Fi é um dos primeiros subsistemas a ser afetado.

Para confirmar: rodar `vcgencmd get_throttled` logo após o evento. O valor `0x50000` (bits 16 e 18) indica que houve afundamento de tensão no passado desde o boot; `0x50005` indica que está acontecendo agora.

```bash
vcgencmd get_throttled
# 0x50000 → afundamento JÁ ocorreu (bits de histórico)
# 0x00000 → sem evento de undervoltage
```

## Solução

**Definitiva:** alimentar a Raspberry Pi com uma fonte 5 V dedicada e independente dos motores. Os motores ficam na fonte principal (12 V com regulador, ou fonte própria de 5 V com corrente suficiente para o pico dos AX-12); a Pi em outra entrada de 5 V.

**Paliativa (sem trocar a fonte):** rodar o controlador dentro do `tmux`. Assim, se o Wi-Fi cair e a sessão SSH se perder, o processo continua vivo na Pi — o SIGHUP que normalmente mataria o `ros2 run` não chega ao nó.

```bash
# Na Raspberry Pi, antes de rodar o controlador:
tmux new -s robo
ros2 run ax12_control ax12_controller

# Para reconectar ao tmux após queda do SSH:
tmux attach -t robo
```

## Como evitar ou detectar antes

- Medir a tensão no conector da Pi com multímetro durante o início da marcha — queda abaixo de 4,75 V confirma o problema.
- Monitorar `vcgencmd get_throttled` periodicamente via cron ou script na Pi.
- Sempre usar `tmux` na Pi para desacoplar o processo do ROS da sessão SSH, mesmo depois de resolver a fonte — protege contra qualquer queda de rede futura.
- Ao dimensionar a fonte dos motores, somar a corrente de stall de todos os AX-12 ativos (≈ 1,5 A cada) como pior caso de pico de partida.
