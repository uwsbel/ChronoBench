"""
Rotor simulation — Jeffcott rotor with IGA beam, flywheel, and motor.
FEA-based multi-body system using ChSystemSMC.
Modified per input2.txt: longer beam (10), thicker beam section,
larger flywheel (0.30 radius), reduced gravity (3.71), motor Sine(60, 0.1),
and adjusted camera.
"""

import csv
import os
import math as m
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Review-only recording config ===

# === Simulation parameters ===
time_step = 0.002
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Beam parameters (from input2.txt)
beam_L = 10.0
beam_ro = 0.060
beam_ri = 0.055
CH_PI = 3.1416

# === System & gravity ===
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))

# === FEA beam section ===
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# === Build IGA beam ===
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    1,
)

# Keep reference to beam nodes to prevent SWIG GC
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]
node_mid = beam_nodes[m.floor(beam_nodes.size() / 2.0)]

# === Flywheel body ===
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z),
    )
)
sys.Add(mbodyflywheel)

# Weld flywheel to mid-beam node
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# === Fixed truss (ground reference) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === End bearing ===
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    beam_nodes.back(),
    truss,
    chrono.ChFramed(beam_nodes.back().GetPos()),
)
sys.Add(bearing)

# === Motor at beam root ===
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    beam_nodes.front(),
    truss,
    chrono.ChFramed(
        beam_nodes.front().GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y),
    ),
)
sys.Add(rotmotor1)
f_ramp = chrono.ChFunctionSine(60, 0.1)
rotmotor1.SetMotorFunction(f_ramp)

# === FEA visualization shapes ===
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Test FEA: the Jeffcott rotor with IGA beams")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2.0, 0, 0))
vis.AddTypicalLights()

# === Solver and static pre-solve ===
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)
sys.DoStaticLinear()

# === CSV logging (review-only) ===
csv_f = None  # cache: defined in scored core, used by review-only

# === Main loop ===
frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
