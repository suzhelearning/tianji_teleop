# PICO clock and controller-continuity implementation plan

1. Add failing TargetManager tests for capped prediction with live twist and timeout
   freeze; add config validation tests.
2. Implement TargetManager semantics and port them to all three worktrees.
3. Make the PICO Viewer use receiver monotonic time consistently for target sampling.
4. Add shared `q/qdot/qddot` reference-state APIs and tests, then preserve state across
   velocity/acceleration and QP-algorithm switches.
5. Add solver retry and bounded controller fallback tests, then implement both paths and
   preserve failed-solve timing diagnostics.
6. Isolate benchmark temporary paths, correct superseded arm-angle documentation, and
   ignore generated Python bytecode.
7. Build and run the full test suite in each worktree, then inspect diffs and status.
