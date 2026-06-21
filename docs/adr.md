# Relatório — Controle AX-12

Diário de bordo da equipe. Relatos em ordem cronológica inversa (mais recente no topo).
Append-only: não reescreva relatos antigos; corrija com um relato novo e datado.

---

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
