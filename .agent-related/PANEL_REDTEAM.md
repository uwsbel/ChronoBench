# Design red-team panel (Phase 3) — record + adopted decisions

A multi-model panel (dan-panel-discussion, 3 rounds, expert level) red-teamed the suite blueprint.
Panelists, anonymized + stable: Expert A = OpenAI gpt-5.5 (high effort); Expert B = Perplexity
sonar-reasoning-pro; Expert C = Gemini 3.1-pro-preview. Claude moderated (no vote). This file is the
provenance record; the adopted decisions are folded into `SUITE_DESIGN.md`.

## What the panel converged on (independent, cross-family agreement)

1. Coverage-by-axis is necessary but NOT sufficient; the suite still risks rewarding demo
   regurgitation. The discriminating signal is failure-mode probes, not domain coverage.
2. Solvers/integrators/timesteppers must be a FIRST-CLASS axis, not folded into contact.
3. Cross-domain COUPLING (rigid-flexible, FEA-contact, vehicle-terrain-sensor, robot-environment) is
   the primary discriminator; isolated per-axis tasks are weak.
4. The Python/C++ SWIG boundary (callbacks, ChFunction subclassing, GC/lifecycle) is a distinct,
   high-yield axis; at least one mandatory task.
5. A LAYERED oracle: execution integrity -> semantic/required-API topology (partial credit) ->
   behavioral invariants (robust/tolerant/metamorphic, not exact trajectories). Strongest consensus.
6. Every task must emit machine-checkable observables (a virtual experiment), not just a runnable
   scene; demote reference-similarity.
7. Cap ALL ground locomotion (vehicles + rovers) at ~3-4 tasks, not vehicles-only; keep a rover only
   if it probes URDF/import/control/sensors, not rolling dynamics.
8. Axis-specific staging, not a uniform create -> modify -> extend.
9. State-management / reproducible logging (seeding, checkpoint/restart, schemas) as a first-class axis.

## Genuine disagreements (preserved)

1. Scoring weight: C wants semantic/AST grading mandatory (partial credit, avoid zero-saturation);
   A wants execution + behavioral oracles primary; B brokers an explicit hybrid triad. (Resolved by
   adopting the layered oracle, which contains all three as layers.)
2. Repair/debug staging: A + B for; C against (calls it SWE-bench-style editing).
3. Scale: B keeps a moderate-N multicore probe; C says test scale syntactically on small N (no
   large-N CI burn); A allows one bounded scale probe, no perf ranking.
4. Metamorphic parameter variation: A/B endorse; C warns it implies fragile code parsing. (Sidestep:
   put parameters in the PROMPT, not injected into generated code.)
5. Sensor embodiment: B wants geometry-aware sensing now; A keeps camera/LiDAR/radar deferred to GPU.

## Decisions adopted (Dan, 2026-06-29)

1. **Judge: DE-SCOPED from the panel's full layered oracle (post-panel honest review).** Adopt L1
   (execution gate) now, L3 behavioral invariants only where physics is clean, L2 kept MINIMAL
   ("necessary capability present", not "preferred idiom"), keep the existing reference+api rubric
   LLM for residual/partial credit, and RECALIBRATE the combined judge vs human judgment. Reason: a
   full L2 required-API/topology layer is itself a new rigid idiom-bias vector, and a full rewrite
   forfeits the v1.0 judge's human calibration. So "more thorough" is clear; "less biased" holds for
   L1 but only conditionally elsewhere, hence the de-scope.
2. **Adopt the failure-mode reframe**: tasks become contracted virtual experiments tagged by failure
   mode; add a first-class solver/integrator axis and a SWIG-callback/lifecycle axis; require
   cross-domain coupling tasks.
3. **Tighten redundancy cap**: ALL ground locomotion (vehicles + rovers) <= 3-4 tasks total; a rover
   stays only for URDF/import/control/sensor probing.
4. **Include repair staging** for some axes (contact/numerics/import/state): a turn presents a
   degraded script to diagnose and fix.

Also folded from the consensus: scale tested via config-on-small-N (no large-N CI burn);
metamorphic variation via prompt parameters; a state-management/reproducibility axis; per-task
failure-triage so results are interpretable rather than a single scalar.

## Usage (tokens; pricing not tracked)

OpenAI gpt-5.5 in 19,817 / out 16,753; Perplexity sonar-reasoning-pro in 19,856 / out 9,904;
Gemini 3.1-pro-preview in 20,606 / out 5,251 (+5,030 reasoning). Cumulative in ~60.3k / out ~31.9k,
9 paid calls.
