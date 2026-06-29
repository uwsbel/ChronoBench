# Task contract: pendulum (pilot, turn 1)

Pilot task to validate the de-scoped judge (L1 execution gate + one clean L3 invariant + rubric).
Deliberately simple so the behavioral oracle is unambiguous.

- **Axis:** mechanism kinematics & constraints (frame-sensitive, instrumented).
- **Simulator:** PyChrono 10.0 (CPU, headless).
- **Prompt parameters:** pivot at origin; revolute joint about global Z; compact bob mass m=1.0 kg at
  distance L=1.0 m from the pivot; gravity g=9.81 m/s^2 along -Y; released from rest at small angle
  theta0=5 deg from the downward vertical; integrate dt=1e-3 s to t_end=5.0 s.
- **Required capabilities (L2, minimal):** a `ChSystem`, a revolute joint constraining the bob to the
  ground, gravity set, a time-stepping loop. (Capability-present checks only, not preferred idioms.)
- **Execution requirements:** headless (NO visualization system instantiated), no absolute paths, no
  network, bounded runtime.
- **Output schema:** write `out.csv` with header `t,theta` (theta in rad from downward vertical), and
  print one JSON line `{"period_est": <s>, "theta_max": <rad>}`.
- **L1 (execution integrity):** imports, constructs, runs to t_end under timeout, no NaN/inf, emits
  the schema.
- **L3 (behavioral invariants):** period_est within +/-10% of the small-angle period
  T = 2*pi*sqrt(L/g) ~= 2.006 s; theta_max <= 1.2*theta0 (amplitude does not grow -> no energy blow-up).
- **Failure triage:** import / construct / run / missing-output / contract-violation / invariant-fail.
