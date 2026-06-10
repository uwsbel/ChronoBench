"""Beam finite-element cantilever simulation (PyChrono FEA).

Models a deformable Euler-Bernoulli beam built from finite elements and nodes.
A circular-section aluminium beam is meshed into several beam elements; the root
node is clamped (fixed) and a transverse tip force bends the beam. The system is
an SMC system solved with a Pardiso MKL direct solver and an HHT timestepper
(required for the stiff beam stiffness matrix). The FEA mesh carries two visual
shapes (a bending-moment scalar field plus node-coordinate glyphs) and is rendered
in real time with the Irrlicht engine. Expected behavior: the beam deflects under
the tip load and settles to a static bent equilibrium.

World convention: Y-up, gravity (0, -9.81, 0), matching the FEA beam ground truth.
"""

import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Constants === beam geometry / material / simulation parameters
BEAM_LENGTH = 2.0          # m, root-to-tip span
BEAM_DIAMETER = 0.05       # m, circular cross-section diameter
N_ELEMENTS = 16            # number of Euler beam elements along the span
DENSITY = 2700.0           # kg/m^3, aluminium
YOUNG_MODULUS = 73.0e9     # Pa, aluminium
POISSON = 0.30             # for shear modulus
RAYLEIGH = 0.001           # structural damping
TIP_FORCE = chrono.ChVector3d(0, -60.0, 0)   # N, transverse tip load (bends beam)

TIME_STEP = 1e-3           # s, small step for the stiff beam
SIM_END = 5.0              # s, run long enough to settle to static deflection
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

A_POINT = chrono.ChVector3d(0, 0, 0)                  # root (clamped) end
B_POINT = chrono.ChVector3d(BEAM_LENGTH, 0, 0)        # free (tip) end

# === System & gravity === SMC system, Y-up gravity (FEA beam convention)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed FEA beam (no rigid-body contact) — no collision system needed.

# === FEA mesh === Euler-Bernoulli beam section + builder
# Keep strong references (mesh/section/builder) to avoid SWIG GC of node containers.
refs = {}

mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
refs["mesh"] = mesh

# FEA beam: no contact material needed — driven by constraints + gravity + tip load only.
section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(BEAM_DIAMETER)
section.SetDensity(DENSITY)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetShearModulusFromPoisson(POISSON)
section.SetRayleighDamping(RAYLEIGH)
refs["section"] = section

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, section, N_ELEMENTS, A_POINT, B_POINT, chrono.ChVector3d(0, 1, 0))
refs["builder"] = builder

# Store node container before indexing (SWIG GC pitfall), then clamp root + load tip.
beam_nodes = builder.GetLastBeamNodes()
refs["beam_nodes"] = beam_nodes
root_node = beam_nodes.front()
tip_node = beam_nodes.back()
root_node.SetFixed(True)               # cantilever clamp at the root
tip_node.SetForce(TIP_FORCE)           # transverse load that bends the beam

sys.Add(mesh)

# === Solver & timestepper === direct Pardiso MKL + HHT (stiff beam stiffness matrix)
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA visualization shapes === scalar moment field + node-coordinate glyphs
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-60.0, 60.0)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.02)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Beam Finite Elements")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.6, 2.2), chrono.ChVector3d(1.0, -0.2, 0))
vis.AddTypicalLights()
vis.AddGrid(0.25, 0.25, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(1.0, -0.6, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === advance the stiff beam to its static bent equilibrium

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot, then clean frames
