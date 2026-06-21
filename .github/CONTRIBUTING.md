# Como contribuir

1. Abra uma *issue* descrevendo o problema ou a melhoria antes de um PR grande — alinhar o
   escopo evita retrabalho.
2. Crie um branch a partir de `main`.
3. Siga as convenções de [AGENTS.md](../AGENTS.md) (nomes de arquivo, nomes de junta,
   mensagens de commit).
4. Se a mudança afeta comportamento visível, atualize **no mesmo PR**:
   - `README.md` e/ou `docs/install.md`,
   - `CHANGELOG.md`,
   - `docs/arquitetura.md`, se mudou como o sistema funciona por dentro,
   - `docs/troubleshooting.md`, se você resolveu um problema antes desconhecido.
5. Confirme que o pacote builda: `colcon build --packages-select ax12_control`.
6. Abra o PR preenchendo o checklist do template.
