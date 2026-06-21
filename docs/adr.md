# Relatório — Controle AX-12

Diário de bordo da equipe. Relatos em ordem cronológica inversa (mais recente no topo).
Append-only: não reescreva relatos antigos; corrija com um relato novo e datado.

---

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
ax12_control/{módulo}}`, irmã de `src/adam/` e `src/adam_moveit_config/` (que já estavam
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
