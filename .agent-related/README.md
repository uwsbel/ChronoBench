# .agent-related/

Agent build-process scaffolding for the ChronoBench v2.0 / PyChrono 10.0 redesign. These files exist
to help the agent (and a human resuming the work) plan, decide, and hand off. They are NOT part of the
benchmark deliverable, and are kept out of the source/doc tree per the lab convention (the global
`CLAUDE.md` rule "Agent scaffolding goes in a `.agent-related/` subfolder").

Contents:
- `HANDOFF.md`       -- resume/handoff ledger for the v2 redesign.
- `SUITE_DESIGN.md`  -- task-suite redesign blueprint (draft): per-task provenance and rationale.
- `PANEL_REDTEAM.md` -- provenance record of the multi-model design red-team.
- `REPRO_SCAN.md`    -- classified scan of `uwsbel/sbel-reproducibility` for sourcing real tasks.

Kept in their normal locations on purpose (not agent-only scaffolding): `demo_data_10/STATUS.md` (the
benchmark's own build status), `CANARY.md` (the functional contamination canary, which must stay public
at the repo root), and `docs/DELTAS_10.md` (the PyChrono 9->10 API-delta reference).
