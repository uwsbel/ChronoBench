# PyChrono 9.0 -> 10.0 port deltas + verified 10.0 idioms

The deterministic-port delta table (grows as verification surfaces more). Entries marked [verified]
were confirmed by running on this machine's `pychrono10` while authoring the pilot tasks; entries
marked [survey] are from the authoritative `projectchrono/chrono@10.0.0` CHANGELOG/demos.

## Confirmed-unchanged (no port needed)
- `chrono.SetChronoDataPath` / `chrono.GetChronoDataPath` [survey]
- `VisualizationType_*` enums [survey]
- Core math/types `ChVector3d`, `ChFramed`, `QUNIT`, body `SetFixed`, `SetMass`, `SetInertiaXX`,
  `SetPos`, `GetPos`, `GetChTime`, `DoStepDynamics`, `SetGravitationalAcceleration` [verified]
- `ChLinkLockRevolute.Initialize(b1, b2, ChFramed)` [verified]
- `ChLinkTSDA.Initialize(b1, b2, local:bool, p1, p2)` + `SetRestLength` / `SetSpringCoefficient` /
  `SetDampingCoefficient` [verified]
- `ChBodyEasyBox(x, y, z, density, visualize, collide, material)` /
  `ChBodyEasySphere(r, density, visualize, collide, material)` [verified]

## Deltas to apply when porting
- Vehicle driver: `veh.ChInteractiveDriverIRR(vis)` -> `veh.ChInteractiveDriver(vehicle.GetVehicle())` [survey]
- Vehicle data path: remove `veh.SetDataPath(...)`; use `veh.GetVehicleDataFile(...)` /
  `chrono.GetChronoDataFile(...)` [survey]
- FEA colormap: `SetColorscaleMinMax(a, b)` -> `SetColormapRange(a, b)` [survey]
- FEA visual shape: `ChVisualShapeFEA(mesh)` -> `ChVisualShapeFEA()` then `mesh.AddVisualShapeFEA(shape)` [survey]
- Contact material: use `ChContactMaterialNSC` / `ChContactMaterialSMC` (10.0 name) [verified]
- Visualization: standardize on VSG (Irrlicht is legacy). `chronoirr.ChVisualSystemIrrlicht` ->
  `vsg.ChVisualSystemVSG`; `veh.ChWheeledVehicleVisualSystemIrrlicht` ->
  `veh.ChWheeledVehicleVisualSystemVSG`; `...Tracked...Irrlicht` -> `...Tracked...VSG`. Window size as
  `chrono.ChVector2i(w,h)`; `AddSkyBox()` -> `SetSkyBoxTexture(path)`; `AddTypicalLights()` ->
  `SetLightIntensity()` + `SetLightDirection()` [survey]. (References verify headless, so VSG runtime
  is not required to gate a task.)

## New / non-obvious 10.0 idioms (verified tonight; easy to get wrong)
- **Collision must be enabled explicitly.** A fresh `ChSystemNSC` has NO collision system
  (`GetCollisionSystem()` is None) and bodies fall through each other. Call
  `sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)` before stepping. [verified]
- **Contact reporting callback signature.** `chrono.ReportContactCallback.OnReportContact` takes
  **10** args: `(pA:ChVector3d, pB:ChVector3d, plane_coord:ChMatrix33d, distance:float,
  eff_radius:float, react_forces:ChVector3d, react_torques:ChVector3d, objA:ChContactable,
  objB:ChContactable, contact_id:int)`, return `True`. `react_forces.x` is the normal component.
  A wrong arity raises a "SWIG director method error". Keep the reporter object alive on the Python
  side; report via `sys.GetContactContainer().ReportAllContacts(reporter)`. [verified]
- **`ChLinkLockGear` works as documented; `ChLinkLockPulley` does NOT.** The gear link (demo-style
  shaft frames `ChFramed(VNULL, QuatFromAngleX(-pi/2))` on Z-axis wheels, `SetTransmissionRatio(r1/r2)`)
  enforces `w2 = -(r1/r2) w1` exactly (~13 digits). The pulley link in the SAME configuration
  enforces `w_out/w_in = tau + 2`, not the textbook `tau = rp1/rp2` (measured with tau=2 -> 4 and
  tau=3 -> 5; independent of shaft distance). Do not use `ChLinkLockPulley` as a grading target or
  in a reference until understood. [verified]
- **Gear-constraint engagement transient.** With `SetEnforcePhase(True)`, a train whose initial
  phase differs from the constraint's preferred phase can take up to ~1 s of simulated time to
  capture (driven wheel near-stationary, then snaps to the exact ratio). Measure steady-state
  observables from a late tail window (the gear task uses t >= 2.0 s); an early window mis-grades
  legitimately-built candidates. [verified]
- **NSC restitution has an isolated bad time-step pocket.** A 1 m ball-drop with e=0.7 rebounds to
  e^2*h0 within 0.2% at dt = 1e-3, 5e-4, 2e-4, and 2e-5, but only 54% of ideal at dt = 1e-4 (and
  95% at 5e-5). Not monotonic in dt; pin the step and calibrate at it (solver_nsc_smc uses
  dt = 2e-4). [verified]
- **SMC default contact stiffness is far too soft for rigid-body impact.** With the default
  `ChContactMaterialSMC` Young's modulus, a 0.1 m sphere hitting at 4.4 m/s sinks ~9 cm into the
  floor (restitution apex still lands near ideal, but the contact is mush). Set
  `SetYoungModulus(~1e8)` for hard contacts; compliant overlap drops to ~7 mm. [verified]
- **`pychrono.vehicle` needs the ACTIVATED conda env.** Importing it under a direct call of the
  env's `python.exe` fails with "DLL load failed while importing _vehicle: A dynamic link library
  (DLL) initialization routine failed"; the same import works under `conda run -n pychrono10`
  (activation sets the DLL search paths). Core/fea/parsers import fine either way. Any harness
  that spawns the judge must go through `conda run`. [verified]
- **CUDA-named FSI/SPH methods are being generalized to GPU names.** With the HIP backend,
  `ChFsiFluidSystemSPH.EnableCudaErrorCheck` is now `EnableGPUErrorCheck`; the shipped
  `demo_ROBOT_Viper_CRM.py` still calls the old name and crashes (one-line upstream fix needed).
  Expect more `Cuda`->`GPU` renames in FSI/CRM-facing API; candidates written from older examples
  will hit them. [verified]
- **`ChParserMbsYAML` wrapper differs from the shipped Python demo.** The demo
  (`demo_YAML_mbs.py`) constructs with (model_yaml, sim_yaml, verbose), but the 10.0 wrapper only
  accepts (sim_yaml[, verbose]) or (); the simulation YAML references the model and solver files
  by path (see `Library/data/yaml/mbs/mbs.yaml`), and relative references resolve against the
  Chrono data dir, so use ABSOLUTE paths for local files. The demo's data filenames
  (`yaml/mbs/slider_crank.yaml`, `simulation_mbs.yaml`) do not exist in the shipped data either
  (`model_slider_crank.yaml`, `mbs.yaml`). [verified]
