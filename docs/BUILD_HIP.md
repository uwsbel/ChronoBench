# Building PyChrono with the HIP (AMD GPU) backend on native Windows (2026-07-04)

The reproducible recipe used on this machine to get FSI-SPH and CRM terrain running from Python on
an AMD GPU, no conda package exists for this yet (the HIP-on-Windows support merged upstream on
2026-07-04: PR #759 "Enable building the HIP backend on native Windows" and #760 "Fix Chrono::VSG
with the HIP backend"). The authoritative general guide is the Chrono repo's
`docs/README_AMD_GPU.md` (Windows section); this file records the exact local instantiation plus
the Python-module step on top of it.

## Machine and toolchain (validated configuration)

1. Windows 11, AMD Radeon 8060S (Strix Halo, `gfx1151`), unified memory.
2. AMD HIP SDK 7.1 at `C:\Program Files\AMD\ROCm\7.1` (ROCm clang, HIP runtime, rocThrust).
3. Visual Studio 2022 Build Tools (MSVC STL + Windows SDK; CMake + Ninja bundled).
4. Eigen 3.4.0 at `C:\libs\eigen-3.4.0`.
5. Conda env `chrono-build` (conda-forge): python 3.12, swig 4.4.1, numpy, used for the Python
   bindings; the resulting `pychrono` is run FROM this env.
6. Chrono source: `WinRepos/chrono`, `main` @ `178fb99f61` (includes #759/#760).

## Key constraints (from the upstream guide, confirmed here)

1. ONE compiler family for everything: `CMAKE_C_COMPILER`, `CMAKE_CXX_COMPILER`, and
   `CMAKE_HIP_COMPILER` all set to the SDK's clang (targets the MSVC ABI; still a native build).
2. `-DCH_ENABLE_OPENMP=OFF` (the Windows HIP SDK ships no libomp).
3. Configure AND build inside a VS x64 developer shell (clang/lld-link need the MSVC libs):
   `& 'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\Launch-VsDevShell.ps1' -Arch amd64 -SkipAutomaticLocation`

## Configure (the exact cache of `build_hip`)

Base configure per `README_AMD_GPU.md` (Ninja, Release, ROCm clang x3, `CHRONO_GPU_BACKEND=HIP`,
`CMAKE_HIP_ARCHITECTURES=gfx1151`, Eigen path, OpenMP OFF) with modules:
FSI=ON, DEM=ON, VEHICLE=ON, FEA=ON, VSG=ON; SENSOR/ROS/PARSERS/MODAL=OFF.

The PYTHON module added on top (this session):

```powershell
$PY   = "C:/Users/dn/.conda/envs/chrono-build/python.exe"
$SWIG = "C:/Users/dn/.conda/envs/chrono-build/Library/bin/swig.exe"
cmake -S . -B build_hip -G Ninja `
  -DCH_ENABLE_MODULE_PYTHON=ON `
  -DPython3_EXECUTABLE="$PY" `
  -DSWIG_EXECUTABLE="$SWIG"
cmake --build build_hip -j 14
```

Note: toggling the Python module invalidated the cache broadly and triggered a near-full rebuild
(~1350 Ninja targets), not an incremental one; budget accordingly. [BUILD OUTCOME + WALL TIME:
filled in below after the build completes.]

## Running the result

1. Environment: `conda run -n chrono-build`, with `PATH` prepended with
   `C:\Program Files\AMD\ROCm\7.1\bin` (HIP runtime DLLs) and `<chrono>\build_hip\bin` (Chrono
   DLLs), and `PYTHONPATH` including `<chrono>\build_hip\bin` (where `pychrono/` lands).
2. Data: `chrono.SetChronoDataPath("<chrono>/data/")` (and the vehicle data subtree for CRM
   vehicle demos) when running source-tree demos.
3. Smoke demos (source tree): `src/demos/python/fsi/demo_FSI_ObjectDrop.py`,
   `src/demos/python/vehicle/demo_VEH_TireTestRig_CRM.py`,
   `src/demos/python/robot/demo_ROBOT_Viper_CRM.py`.

## Benchmark integration decision

The HIP build is a SECOND, parallel environment used only by future `fsi_*` / `crm_*` tasks. The
existing 12-task suite stays pinned to the untouched `pychrono10` conda env (its 36/36 self-check
remains valid); new task contracts will name the new environment explicitly.

## Gotchas found and fixed (this machine, 2026-07-04)

1. **`/DWNT` in the SWIG wrapper targets breaks clang.** Five Python targets (cascade, vehicle,
   robot, parsers, ros) passed the MSVC flag spelling `"/DWNT"` to
   `target_compile_definitions()`; ROCm clang receives `-D/DWNT` and errors with "macro name must
   be an identifier". Fixed to the portable `"WNT"` (local chrono commit `5623ad893b`,
   upstream-PR candidate; identical behavior under MSVC).
2. **`vsg-17.dll` must be resolvable at import time.** `_fsi.pyd` links `Chrono_fsisph_vsg.dll`
   which needs the VSG runtime; add `C:\libs\vsg\bin` to `PATH` alongside the ROCm and build
   `bin` directories (the generated `pychrono/__init__.py` converts `PATH` entries into
   `os.add_dll_directory` calls, so `PATH` is sufficient).
3. Toggling `CH_ENABLE_MODULE_PYTHON` invalidated the Ninja cache broadly: a near-full rebuild
   (~1350 targets), not an incremental one.

## Status log

1. 2026-07-04: configure with PYTHON module OK (25.7 s; SWIG 4.4.1; use `Python3_EXECUTABLE`,
   not the legacy `PYTHON_EXECUTABLE`).
2. 2026-07-04: build SUCCEEDED after the `/DWNT` fix: `build_hip/bin/pychrono/` with `_core`,
   `_fea`, `_fsi`, `_vehicle`, `_robot`, `_vsg3d` extension modules. `import pychrono` and
   `import pychrono.fsi` verified from the `chrono-build` env.
3. 2026-07-04 validation, both GREEN (chrono-build env; PATH = ROCm bin + build bin + vsg bin):
   - FSI-SPH `demo_FSI_ObjectDrop.py --no_vis --t_end 1.0`: ran to completion, 8,003 SPH steps,
     wall 76.6 s (~77 s per simulated second at demo resolution).
   - CRM `demo_ROBOT_Viper_CRM.py` (headless copy, tend = 3): ran to completion, wall 50.6 s
     (~17 s per simulated second). Task-timeout planning: budget 600-900 s for fsi_*/crm_* task
     turns at comparable resolutions.
4. One more demo-side gotcha: the shipped `demo_ROBOT_Viper_CRM.py` still calls
   `EnableCudaErrorCheck`, renamed `EnableGPUErrorCheck` in the CUDA->GPU naming generalization;
   the demo needs a one-line upstream fix (only this one demo affected in src/demos/python).
