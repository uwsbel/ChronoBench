"""
Jeffcott Rotor simulation using PyChrono FEA (IGA beam + flywheel).

System type: ChSystemSMC
Structure: A hollow circular IGA beam shaft with a flywheel body welded to the mid-span node.
The beam is supported by a motor constraint at the front end and a bearing at the back end.
The motor drives shaft rotation with a sine speed function.
Expected behavior: The flywheel spins and the flexible shaft exhibits rotor-dynamic whirl under
the applied motor torque and gravity, demonstrating gyroscopic effects and shaft bending.

Parameters (modified from baseline):
  beam_L=10, beam_ro=0.060, beam_ri=0.055, flywheel_radius=0.30,
  gravity=(0,-3.71,0), motor_function=Sine(60,0.1), camera=(0,2,8).
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Constants ===
CH_PI = math.pi

# Beam geometry
beam_L   = 10.0        # beam length [m]
beam_ro  = 0.060       # outer radius [m]
beam_ri  = 0.055       # inner radius [m]
n_spans  = 20          # number of IGA spans (linear order, as in reference)
beam_order = 1         # linear IGA order (reference uses 1)

# Flywheel
fly_radius  = 0.30     # flywheel radius [m]
fly_height  = 0.10     # flywheel thickness [m]
fly_density = 7800.0   # steel [kg/m3]

# Beam material (steel)
beam_density = 7800.0
beam_E       = 210e9
beam_nu      = 0.3

# Simulation
time_step    = 0.002
sim_end      = 10.0
render_fps   = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Derived section properties (hollow circle, CH_PI approx as in reference)
CH_PI_REF = 3.1456  # use same approx as reference for exact equivalence
beam_area = CH_PI_REF * (beam_ro**2 - beam_ri**2)
beam_Iy   = (CH_PI_REF / 4.0) * (beam_ro**4 - beam_ri**4)
beam_Iz   = (CH_PI_REF / 4.0) * (beam_ro**4 - beam_ri**4)
beam_J    = (CH_PI_REF / 2.0) * (beam_ro**4 - beam_ri**4)

# === System & gravity ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))
# FEA beam: no contact material needed — driven by constraints + gravity + motor only

# Pardiso MKL solver for stiff IGA beams
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# === FEA Mesh — IGA shaft ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True, 2)  # 2 integration points for cubic gravity precision
sys.Add(mesh)

# IGA Cosserat section (hollow circle)
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(beam_density)
minertia.SetArea(beam_area)
minertia.SetIyy(beam_Iy)
minertia.SetIzz(beam_Iz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(beam_E)
melasticity.SetShearModulusFromPoisson(beam_nu)
melasticity.SetIyy(beam_Iy)
melasticity.SetIzz(beam_Iz)
melasticity.SetJ(beam_J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)   # SetAsCircularSection would overwrite Iyy/Izz/J

# Build the IGA beam along X axis from 0 to beam_L
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    n_spans,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    beam_order,
)

# Get mid-span node for flywheel attachment (keep reference to avoid SWIG GC)
node_mid = builder.GetLastBeamNodes()[math.floor(builder.GetLastBeamNodes().size() / 2.0)]

# === Flywheel body welded to mid-span node ===
mbodyflywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,   # cylinder axis along Y
    fly_radius, fly_height,
    fly_density,
)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # flywheel initial center
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)  # rotate 90° so cylinder is on X
    )
)
sys.Add(mbodyflywheel)

# Weld flywheel to mid-beam node (rigid, all 6 DOF)
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# === Fixed truss (ground reference body) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === Bearing at back end: free tx, constrain ty,tz, free rx, constrain ry,rz ===
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes().back(),
    truss,
    chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()),
)
sys.Add(bearing)

# === Motor at front end: rotates the beam shaft ===
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes().front(),   # body A (slave: beam front node)
    truss,                                 # body B (master: fixed ground)
    chrono.ChFramed(
        builder.GetLastBeamNodes().front().GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y),  # motor axis = beam X axis
    )
)
sys.Add(rotmotor1)

# Motor speed function: Sine(amplitude=60, frequency=0.1)
f_ramp = chrono.ChFunctionSine(60, 0.1)
rotmotor1.SetMotorFunction(f_ramp)

# === FEA Visualization ===
# Shape 1: deformed surface
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

# Shape 2: node coordinate-system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Pre-solve static linear equilibrium
sys.DoStaticLinear()

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor — IGA Beam + Flywheel")
vis.Initialize()                                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === Main loop ===


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
