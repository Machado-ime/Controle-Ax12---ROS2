# Relatório — Controle AX-12

Diário de bordo da equipe. Relatos em ordem cronológica inversa (mais recente no topo).
Append-only: não reescreva relatos antigos; corrija com um relato novo e datado.

---

## 2026-07-01 — Claude (a pedido de Fernando) — Corrige juntas de pitch com eixo invertido
`[problema]`
Testando o `controle_manual` com o robô real (motores ligados no PC via usbipd), o Fernando
notou que ao comandar as juntas de pitch de quadril e tornozelo o motor real ia para o lado
**oposto** do modelo no RViz (URDF leva o pé à frente, motor leva atrás). Testado junta a
junta: as 4 juntas de pitch de tornozelo e quadril (`pd_picht_tornozelo_3`,
`pe_picht_tornozelo_4`, `pd_picht_quadril_7`, `pe_pich_quadril_8`) invertidas; joelhos e rolls
corretos.

**Causa raiz:** as pernas foram construídas espelhadas no URDF (os `rpy` dos joints de perna
esquerda e direita são opostos), mas com o mesmo `axis xyz="0 0 1"` local — então o eixo
positivo aponta para lados opostos no mundo, e não bate com a direção física de montagem do
motor. O `ax12_controller` convertia rad→unidades com uma fórmula única, sem sinal por junta.
Pista confirmadora: os `joint_limits` do `pd_picht_tornozelo_3` estavam gravados como
`(-1.4661, 0.5585)`, exatamente a **negação** da faixa física medida do motor
`(-0.5585, 1.4661)` (118°–234° na escala do AX-12) — ou seja, os limites já tinham sido
gravados na convenção do URDF esperando a inversão, mas a conversão nunca a fazia (e por isso
o clamp ainda protegia o lado mecânico errado).

**Solução:** conjunto `juntas_invertidas` no `ax12_controller`, que troca o sinal do ângulo
**apenas na fronteira rad↔unidades do motor** — na escrita (após o clamp, que continua nos
limites do URDF) e na leitura de telemetria (posição e velocidade, antes de publicar
`/joint_states`). Todo o resto — limites, matrizes de marcha, `/joint_states`, RViz, MoveIt —
fica na convenção do URDF. É o papel do flag de direção por junta de um `SystemInterface` do
`ros2_control`. Não mexer no URDF (é a fonte de verdade cinemática; inverter o eixo lá
quebraria as marchas e o planejamento).

**Como evitar:** ao ligar um motor novo, testar a direção no `controle_manual` (real vs RViz)
antes de comandá-lo na marcha; se invertido, adicionar o nome a `juntas_invertidas`.

## 2026-06-30 — Claude (a pedido de Fernando) — Simplifica instalação: clone = workspace
`[decisão]`
Depurando o `controle_manual.launch.py` em duas máquinas (Pi e PC/WSL2) na sessão anterior,
acumulamos workspaces divergentes: `~/ax12_control_ws` com symlinks pro clone, mais um
`~/dev/ax12_control_ws` antigo (pré-rename `adam`→`adam_urdf`, com edições locais não
commitadas e já superadas) sourced por engano no `.bashrc`, e na Pi um sparse-checkout só de
`src/ax12_control` que quebrou assim que o `controle_manual.launch.py` passou a exigir também
`adam_urdf`. Cada incompatibilidade desses custou uma rodada de "package not found".

**Causa raiz comum:** o workspace era uma construção separada do clone git (symlink ou
sparse-checkout), então as duas coisas podiam divergir — um `git pull` no clone não
necessariamente refletia no workspace buildado, e vice-versa.

**Solução:** eliminar a separação. O repositório já tem `src/` na raiz (decisão de
"Código movido para src/", 2026-06-21), então o próprio clone funciona como workspace do
`colcon build` direto — confirmado com um clone limpo (`git clone` + `colcon build` na raiz)
buildando os 3 pacotes (`ax12_control`, `adam_urdf`, `adam_moveit_config`) sem nenhum passo
extra. A receita agora é idêntica em qualquer máquina: clonar em `~/dev/Controle-Ax12---ROS2`,
`colcon build` na raiz, `source install/setup.bash`. Apagamos os workspaces
divergentes (`~/ax12_control_ws`, `~/dev/ax12_control_ws` no PC) e o sparse-checkout da Pi, e
corrigimos `docs/install.md`, `src/README.md` e `docs/troubleshooting.md`.

**Achado lateral:** `adam_moveit_config` existe no GitHub (builda normal num clone limpo) mas
não aparecia no clone antigo do PC porque esse clone tinha um sparse-checkout restrito —
não era de fato um pacote faltando no repositório, só uma cópia local limitada.

## 2026-06-27 — Claude (a pedido de Fernando) — Jog manual local (controle_manual.py)
`[decisão]`
Estávamos depurando descoberta DDS entre uma Raspberry Pi e um PC (Wi-Fi, ROS_DOMAIN_ID
batendo, mas `ros2 node list` do PC não via o nó da Pi — suspeita de NAT do WSL2 do lado do
PC). O Fernando decidiu mudar de estratégia: em vez de depender da rede PC↔Pi para testar
junta a junta, trabalhar direto na Raspberry Pi com um controle manual local. Criamos
`controle_manual.py` (slider Qt por junta) + `controle_manual.launch.py`
(`robot_state_publisher` + `ax12_controller` real + RViz + o slider, tudo num só `ros2
launch`).

**Decisão de design:** o slider publica SÓ `/joint_trajectory` (o mesmo tópico que o
`ax12_controller` já assina), nunca `/joint_states` direto. Publicar `/joint_states` pelo
slider também pareceria mais "instantâneo", mas criaria dois publishers do tópico (o slider
com a posição alvo e o `ax12_controller` com a posição real da telemetria) brigando e fazendo
o RViz tremer entre os dois valores. Deixando o RViz espelhar só a telemetria real (mesmo
mecanismo do `display.launch.py use_gui_sliders:=false`), o slider "manda pro motor e pro RViz
ao mesmo tempo" sem essa briga — o RViz só mostra a posição assim que o motor de fato chega
lá. Limites de cada slider copiados de `ax12_controller.joint_limits` (mesma fonte, mantidos
em sincronia manualmente).

## 2026-06-21 — Claude (a pedido de Fernando) — Remove src/matrizes-de-movimento/ (duplicado)
`[decisão]`
O Fernando tinha apagado `src/matrizes-de-movimento/` de propósito (não foi um acidente como
eu tinha assumido num relato anterior) — `cin_inve.yaml` e `otimizada.yaml` ali eram cópias
duplicadas do que já existe em `src/ax12_control/ax12_control/*.yaml`. `otimizacao.h` (header
C de um protótipo de 18 motores, sem cópia em nenhum outro lugar) foi removido junto, mesma
lógica já aplicada a `legacy/` e aos protótipos `AX12Controller_v1.py`/`v2.py`: sem valor de
referência, não arquivar. Atualizados os organogramas/tabelas em `README.md`, `AGENTS.md`,
`src/README.md` e `docs/arquitetura.md` que ainda citavam a pasta.

## 2026-06-21 — Claude (a pedido de Fernando) — Corrige build apos reorganizacao manual
`[problema]`
Depois do merge dos PRs #2 e #3, o Fernando continuou a consolidação manualmente direto na
pasta local (`main`, fora desta worktree): moveu `adam/` e `adam_moveit_config/` (que
viviam na raiz do repositório — eu não sabia que estavam neste mesmo repo até encontrar isso)
para dentro de `src/`, mas achatou `src/ax12_control/ax12_control/*` para `src/*`, removendo a
pasta que o `setup.py` espera para o módulo Python. Resultado: `colcon build` falhava com
`can't copy 'ax12_control/adam.rviz': doesn't exist`. Também apagou (não moveu) `legacy/` e
encontrei dois protótipos não rastreados antes, `AX12Controller_v1.py`/`v2.py`.

**Causa raiz:** nomes de pasta repetidos (`ax12_control/ax12_control/`) parecem redundantes,
mas são a convenção do `ament_python` — o pacote (`package.xml`+`setup.py`) e o módulo Python
importável (mesmo nome) são coisas diferentes e não podem ser achatados num só nível.

**Solução:** restaurada a estrutura `src/ax12_control/{package.xml,setup.py,launch/,
ax12_control/{módulo}}`, irmã de `src/adam_urdf/` e `src/adam_moveit_config/` (que já estavam
corretos, cada um na própria pasta). A pedido do Fernando, `legacy/` ficou removido (não
restaurado) e os protótipos `AX12Controller_v1.py`/`v2.py` também foram apagados — sem valor
de referência. Validado com `colcon build --packages-select ax12_control adam
adam_moveit_config` antes de comitar.

**Como evitar:** ao reorganizar manualmente um pacote `ament_python`, nunca remover a pasta
interna com o mesmo nome do pacote — ela é o módulo Python, não uma duplicata por engano.

## 2026-06-21 — Claude (a pedido de Fernando) — Código movido para src/
`[decisão]`
Movemos todo o código para `src/`: o pacote ROS inteiro (`package.xml`, `setup.py`, `launch/`
e o módulo Python `ax12_control/`) agora vive em `src/ax12_control/`, `legacy/` em
`src/legacy/`, e `matrizes-de-movimento/` (dado/referência de origem das marchas) em
`src/matrizes-de-movimento/` — a pedido do Fernando, para concentrar o máximo possível do
repositório dentro de `src/`. Validamos antes de mexer que o `colcon` descobre o
`package.xml` recursivamente independente da profundidade (testado com `colcon list` e
`colcon build` num workspace de teste com a estrutura aninhada) — o comando de instalação no
README não precisa mudar.

## 2026-06-21 — Claude (a pedido de Fernando) — Reorganização do repositório
`[decisão]`
Reorganizamos a estrutura de documentação seguindo o guia de Gestão de Conhecimento (v4) da
equipe. Mudanças: README reduzido a quickstart (visão geral, pré-requisitos, estrutura,
links); passo a passo completo de instalação movido para `docs/install.md`; pasta
`docs/troubleshooting/` consolidada em `docs/troubleshooting.md` (arquivo único, uma seção
por problema); referências externas e cola de comandos ROS movidas para `docs/ref/`; pasta
`matrizes de movimento/` renomeada para `matrizes-de-movimento/` (convenção de nomes sem
espaço/acento). Adicionados `AGENTS.md`, `CHANGELOG.md` e os arquivos de saúde da comunidade em
`.github/` (CODEOWNERS, templates de issue/PR, CONTRIBUTING).

Este arquivo (`docs/adr.md`) também nasce nesta reorganização — relatos anteriores a
2026-06-21 não foram registrados retroativamente para não inventar contexto/alternativas que
não foram de fato documentadas na hora; para o histórico de código, ver `git log`.
