# Roteiro de bancada — trazer a OpenCR e os motores de volta

Sequência de testes para validar o `ax12_controller` contra hardware real,
uma variável por vez. Cada fase só faz sentido se a anterior passou.

Todas as correções de robustez de conexão (setembro/2026) foram validadas
apenas com porta serial falsa (`pty`), que **não** reproduz o comportamento
de baudrate da OpenCR nem o rail de 12 V. Este roteiro é a validação real.

> **Regra geral:** em toda fase com motor, rode primeiro em modo observador
> (`-p ligar_torque:=false`). Ele lê a telemetria sem escrever nada no
> barramento — não liga torque, não o desliga ao sair, e descarta comandos.
> Metade dos riscos desaparece e você ainda valida conversão e sentido.

---

## Fase 0 — varredura do barramento (antes de qualquer nó ROS)

**Objetivo:** estabelecer a verdade de referência. Sem isso, um nó que não
acha motores te deixa sem saber se o problema é o código ou o barramento.

```bash
ros2 run ax12_control scan_bus
```

Se a porta não for `/dev/ttyACM0`:

```bash
ros2 run ax12_control scan_bus --device /dev/ttyUSB0
```

**O que ele responde, de graça:**

| saída | o que significa |
|---|---|
| `Modo de baixa latencia: ATIVO` | o `ax12_controller` vai usar orçamento de 4 ms/pacote |
| `Modo de baixa latencia: INDISPONIVEL` | vai manter 16 ms/pacote — ciclo de telemetria 3x mais lento com barramento ruim |
| `OpenCR no ID 200: RESPONDE` | firmware estilo OP3 — o IMU existe, pode usar `-p taxa_imu:=50.0` |
| `OpenCR no ID 200: mudo` | firmware `usb_to_dxl` (ponte pura) — o ID 200 **nunca** responde, por construção |
| lista de IDs + modelo | quais motores existem de fato, e em qual baudrate |

O script é **somente leitura** — só instruções READ, que não alteram
registrador, não ligam torque e não movem motor. Pode rodar com o robô montado.

**Anote o resultado.** Ele é a referência de todas as fases seguintes.

---

## Fase 1 — OpenCR sem nenhum motor

**Objetivo:** validar a camada de conexão isoladamente. Esta é a **única fase
em que dá para arrancar o cabo USB com segurança**, e é exatamente a correção
que não pôde ser testada com `pty`.

```bash
ros2 run ax12_control ax12_controller --ros-args -p ligar_torque:=false
```

### 1.1 — Partida

Esperado: porta abre, os 10 motores não respondem, e sai a mensagem do ramo
`0/N`, que aponta para energia e cabo (e **não** manda conferir IDs):

```
NENHUM dos 10 motores respondeu. Isso quase nunca e ID errado: a porta
serial abriu, entao o adaptador esta vivo — o que falta e energia ou cabo...
```

### 1.2 — O ciclo de telemetria está aguentando?

Truque sem instrumentar nada: os avisos de falha saem a cada 25 ciclos, então
**o intervalo entre rajadas mede o tempo real do ciclo**.

| orçamento | ciclo previsto | rajadas a cada | veredito |
|---|---|---|---|
| 4 ms (baixa latência ativa) | ~101 ms | **~5,0 s** | cabe no período de 200 ms |
| 16 ms (sem baixa latência) | ~341 ms | **~8,5 s** | estoura; baixe `taxa_leitura` |

Se as rajadas vierem a cada ~8,5 s, o loop está atrasando o executor e os
comandos vão chegar tarde. Nesse caso: `-p taxa_leitura:=2.0`.

### 1.3 — Arrancar o cabo USB (o teste principal)

Com o nó rodando, **puxe a USB**. Esperado:

```
PORTA SERIAL CAIU durante a leitura de telemetria! Reconexao automatica a cada 1 s.
```

Sem traceback, sem o nó morrer. Religue o cabo: deve reconectar sozinho em
cerca de 1 s.

> Isso valida a correção do `termios.error`, que não herda de `OSError` e
> antes escapava de todos os guardas, derrubando o nó inteiro quando a USB
> era arrancada. No WSL, lembre que desanexar pelo `usbipd` tem o mesmo efeito.

### 1.4 — Encerramento

```bash
# no terminal do nó: Ctrl+C
# ou, de outro terminal:
kill -TERM $(pgrep -f ax12_controller)
```

Os dois devem sair limpo, sem traceback. (O `SIGTERM` é como `systemd`,
Docker e o `ros2 launch` encerram um nó.)

### O que esperar de incômodo nesta fase

- **~2 avisos por segundo, para sempre.** Com 10 motores ausentes a 5 Hz,
  cada um avisa a cada 25 ciclos = 10 avisos a cada 5 s. Para um barramento
  permanentemente vazio isso é demais — se incomodar, é sinal de que vale
  implementar recuo exponencial (25, 50, 100, 200…). Silencie com
  `-p taxa_leitura:=0.0` enquanto testa o resto.
- **`/joint_states` e `/diagnostics` não publicam nada.** O código só publica
  se ao menos um motor respondeu. `ros2 topic hz /joint_states` fica mudo.
  É o comportamento atual; vale decidir se é aceitável.

---

## Fase 1.5 — motor ligado, 12 V desligado

**Objetivo:** validar que o **diagnóstico está certo**, não só que a mensagem
aparece. Custa 30 segundos.

Conecte o motor ID 1 fisicamente, mas deixe a fonte de 12 V **desligada**.
Rode a fase 1 de novo. A mensagem `0/10` deve aparecer — e agora você sabe
que o conselho dela ("verifique a fonte 12 V") é exatamente o certo para a
situação real. É a diferença entre uma mensagem bonita e uma mensagem útil.

Ligue a fonte com o nó rodando: o timer de reconexão não age aqui (a porta
nunca caiu), mas na próxima leitura os motores devem começar a responder e
sair `voltou a responder de forma estavel`.

---

## Fase 2 — motor ID 1 (`pd_roll_tornozelo_1`)

| | |
|---|---|
| Junta | `pd_roll_tornozelo_1` — roll do tornozelo direito |
| Limites | −0,873 a +0,593 rad (−50° a +34°) |
| Invertido? | **não** (só os 4 pitchs e os rolls de quadril estão em `juntas_invertidas`) |

> **Segurança mecânica:** é um roll de tornozelo. Se estiver montado num robô
> em pé, movê-lo pode derrubá-lo. Prefira o motor na bancada, ou o robô
> suspenso/apoiado.

### 2a — Ler antes de escrever (risco zero)

```bash
ros2 run ax12_control ax12_controller --ros-args -p ligar_torque:=false
```

Noutro terminal:

```bash
ros2 topic echo /joint_states
```

**Gire o eixo à mão** e confira três coisas de uma vez:

1. **Conversão:** o ângulo em `/joint_states` bate com a realidade? (compare
   com um transferidor; 0 rad deve ser o centro do curso, unidade 512)
2. **Sentido:** girando para um lado, o valor sobe ou desce? Isso confirma ou
   desmente o fato de o ID 1 **não** estar em `juntas_invertidas`.
3. **Telemetria:** tensão plausível (~11–12,6 V) e temperatura ambiente?

Nesta fase a mensagem de resumo muda para o ramo parcial (`1/10`), com
conselho diferente do `0/10` — suspeitar dos ausentes, não do barramento.

### 2b — Só então, com torque

```bash
ros2 run ax12_control ax12_controller
```

Comece pequeno e devagar:

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pd_roll_tornozelo_1'], points: [{positions: [0.1], velocities: [0.3]}]}"
```

Depois teste o **clamp**, mandando muito além do limite:

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pd_roll_tornozelo_1'], points: [{positions: [9.0], velocities: [0.3]}]}"
```

Esperado: para em +0,593 rad e avisa **uma vez só**. Repita o mesmo comando
várias vezes — não deve avisar de novo (memória de transição). Mande 0.0 e
depois 9.0 outra vez: aí sim avisa novamente.

E o **NaN**, que antes ia silenciosamente para o batente:

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pd_roll_tornozelo_1'], points: [{positions: [.nan], velocities: [0.3]}]}"
```

Esperado: `posicao invalida (nan) — junta ignorada`, e o motor **não se move**.

---

## Fase 2.5 — cabo intermitente

**Objetivo:** única prova física do balde furado, a correção mais sutil de
todas — feita exatamente para o cabo mal crimpado.

Com o ID 1 rodando e respondendo, **mexa ou afrouxe levemente o conector de
3 pinos** durante a operação.

Esperado: aparece `falhando na leitura (nivel N; sobe 1 por falha, desce 1
por acerto)`. Com a lógica antiga (contador que zerava a cada sucesso e
avisava só em `== 25` exato), um motor que falha 24 vezes e responde uma
**nunca** chegava ao limiar — passava despercebido para sempre.

Firme o cabo de volta: quando o balde drenar até zero, sai `voltou a
responder de forma estavel`.

Teste também **desconectar o cabo de dados do motor** (diferente de arrancar
a USB da fase 1) e ver a degradação.

---

## Fase 3 — dois motores

Em relação à fase 2 muda uma coisa só, mas é a razão de existir do
`GroupSyncWrite`: **os dois partem juntos**, num único pacote.

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pd_roll_tornozelo_1', 'pe_roll_tornozelo_2'], points: [{positions: [0.2, 0.2], velocities: [0.5, 0.5]}]}"
```

Filme em câmera lenta se puder — simultaneidade é difícil de julgar a olho.

Teste também o **espelhamento**. Os limites de PD e PE são espelhados
(−0,873/+0,593 contra −0,593/+0,873), então:

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['pd_roll_tornozelo_1', 'pe_roll_tornozelo_2'], points: [{positions: [0.2, -0.2], velocities: [0.5, 0.5]}]}"
```

> **Nota:** `pd` e `pe` são pernas **opostas** (direita e esquerda), ambos
> roll de tornozelo. Se a intenção era duas juntas da **mesma** perna, os
> pares são: direita = IDs 1, 3, 5, 7, 9 / esquerda = IDs 2, 4, 6, 8, 10.

---

## Fase 4 — os 10 motores

Só aqui a característica de tempo real aparece: 10 motores **respondendo**
têm custo bem diferente de 10 mudos, e é esse número que decide se
`taxa_leitura:=5.0` é sustentável.

Também é a primeira vez que os 500 ms de `_ligar_torque` (10 × 50 ms de
espera para a fonte estabilizar) acontecem de verdade — inclusive **dentro do
timer de reconexão**, bloqueando o executor por meio período. É um ponto
conhecido e ainda em aberto; observe se atrapalha na prática.

Acompanhe com:

```bash
ros2 run ax12_control ax12_monitor
```

> O `ax12_monitor` filtra `/diagnostics` pelo prefixo `ax12/`, então mostra
> corretamente as juntas deste nó.

---

## Resumo: o que cada fase valida sozinha

| fase | valida |
|---|---|
| **0** | qual firmware, quais IDs existem de fato, se a baixa latência funciona |
| 1 | `termios.error` no arranque real da USB, orçamento de latência, encerramento |
| **1.5** | se o conselho da mensagem `0/N` é o certo |
| 2a | conversão e sentido, sem risco nenhum |
| 2b | escrita, clamp, guarda de NaN, torque |
| **2.5** | balde furado no cenário que o motivou |
| 3 | simultaneidade do SyncWrite, espelhamento |
| **4** | tempo real com carga de verdade |

---

## Se algo der errado

| sintoma | primeira suspeita |
|---|---|
| `A PORTA ... NAO ABRIU` | dispositivo não existe, não anexado no WSL, ou outro processo segurando |
| `NENHUM dos 10 motores respondeu` | fonte 12 V, chave de power, cabo de 3 pinos — **não** é ID |
| `Apenas N/10 responderam` | aí sim: ID regravado, cabo solto a partir de um ponto, motor queimado |
| `baudrate ... nao e suportado` | erro de digitação; o nó lista os valores válidos |
| motor anda para o lado errado | `juntas_invertidas` — veja a fase 2a |
| avisos de leitura inundando | barramento ausente; `-p taxa_leitura:=0.0` enquanto diagnostica |
