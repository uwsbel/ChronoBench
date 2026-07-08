# PARKED email draft: FSI-SPH buoyancy bias vs Archimedes

Status: DRAFTED 2026-07-06, NOT sent (Dan's call: wait for the NVIDIA re-test).
Send condition: the finding duplicates on the NVIDIA/Linux machine (CUDA backend), density sweep
500/900/1100/1200, ideally at spacings 0.025 and 0.0125. If it duplicates: send this via the
dan-email skill, then build the single-file repro demo for Radu (design sketch: density sweep +
Archimedes comparison table, demo_FSI_ObjectDrop solver settings verbatim, spacing as a CLI knob).
If it does NOT duplicate: investigate the HIP backend specifically before mailing anyone.

To: negrut@wisc.edu
Subject: FSI-SPH buoyancy bias vs Archimedes (ChronoBench GPU-task calibration, 2026-07-06)

---

Dan,

The FSI finding from the ChronoBench GPU-task calibration, written up so it stands on its own. Agreed plan: park it, re-test on the NVIDIA/Linux machine, and only knock on Radu's door if it duplicates there.

SETUP
1. PyChrono 10.0, source-built with the HIP backend on Windows (gfx1151). Solver settings copied verbatim from the shipped demo_FSI_ObjectDrop.py: ChFsiProblemCartesian, initial spacing 0.025 m, RK2 integration, 4 BCE layers, XSPH shifting (eps 0.5), artificial viscosity 0.03, TAIT equation of state, ADAMI boundary treatment, delta-SPH 0.1, variable time step, depth-based initial pressure.
2. Rigid sphere, radius R = 0.12 m (about 4.8 particles per radius), released just above a 0.8 x 0.8 x 0.5 m water tank (density 1000, viscosity 1). The settled center height is the mean over the last second of a 3 s run; that mean repeats to about 1 mm across independent runs.

ARCHIMEDES vs SPH (settled center relative to the still-water surface)
1. density 500:  predicted  0.000 m (half-submerged exactly); measured +0.043 m.
2. density 900:  predicted -0.073 m; measured -0.010 m.
3. density 1200: predicted SINKS (no flotation solution exists for a density ratio above 1); measured FLOATS indefinitely, hovering at -0.039 m.

The headline is row 3: this is not a tolerance issue but a qualitative one. At this resolution the float/sink threshold itself is wrong, and a body 1.2x the density of water never sinks. The floating rows show the same effect quantitatively: everything rides 4 to 6 cm high of the exact closed-form prediction.

HYPOTHESIS
The BCE marker skin makes the body hydrodynamically larger than its geometric radius by about half an initial spacing. (R + h/2)^3 / R^3 = 1.35 at R = 0.12 and h = 0.025, i.e. roughly 35% extra effective buoyant volume. That single number matches both the float-height shift and the fact that a 1.2 density ratio (< 1.35) still floats. If the scaling is right, the effective sink threshold sits near 1.35x water at this resolution and should approach 1 as the spacing shrinks.

CAVEATS
1. The measurement is mine: the shipped ObjectDrop demo drops an object but never measures the settled draft, so this would be invisible in it. The solver configuration, though, is the demo's own.
2. Only one resolution tested. The key untested prediction is that the bias shrinks roughly like the spacing; a two-spacing sweep is the first thing to run on the NVIDIA machine.
3. Observed on the HIP/Windows build. Nothing about the mechanism is backend-specific, but CUDA duplication is exactly what we want before reporting it.

NEXT STEP (parked)
On the NVIDIA machine: re-run the density sweep (500 / 900 / 1100 / 1200) on the CUDA backend, ideally at spacings 0.025 and 0.0125. If it duplicates, I prepare a single-file, self-contained PyChrono repro (density sweep plus an Archimedes comparison table) for Radu. I have made a note of this and will bring it up when we get to that machine.

For ChronoBench itself this is already handled: the fsi_object_drop task anchors its bands to Archimedes for ordering and the float/sink split, but calibrates and freezes the absolute values on the pinned build, with the bias documented in the task contract (demo_data_10/fsi_object_drop/CONTRACT.md; the raw numbers are also in docs/DELTAS_10.md).

Claude (from your ChronoBench session)
