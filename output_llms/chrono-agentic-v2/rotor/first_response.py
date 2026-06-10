"""
Jeffcott Rotor Simulation — IGA (Cosserat) Beam with Flywheel and Motor Drive

Models a Jeffcott rotor: an IGA beam shaft fixed in bearings at both ends,
with a flywheel (disc) welded at the beam midpoint and a rotational motor
driving one end of the shaft. Uses ChSystemSMC with Pardiso MKL solver and
HHT timestepper. Demonstrates large-rotation FEA beam dynamics and rotor whirl.

System: ChSystemSMC (Y-up, gravity (0, -9.81, 0))
FEA: IGA/Cosserat beam with two bearing constraints and flywheel weld
Motor: ChLinkMotorRotationSpeed at shaft end
Visualization: Irrlicht window with ChVisualShapeFEA surface + glyph shapes
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Named constants ===
TIME_STEP    = 0.002          # s — IGA rotor timestep
SIM_END      = 6.0            # s
RENDER_FPS   = 50.0           # Hz
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Beam geometry (shaft)
BEAM_L    = 1.2               # m — total shaft length
BEAM_RO   = 0.010             # m — outer radius of shaft (hollow / solid)
BEAM_RI   = 0.000             # m — inner radius (solid shaft)
N_SPANS   = 10                # number of IGA spans
IGA_ORDER = 3                 # cubic IGA

# Shaft material (steel)
SHAFT_DENSITY = 7800.0        # kg/m^3
SHAFT_E       = 210e9         # Pa
SHAFT_NU      = 0.3

# Derived shaft cross-section properties
BEAM_AREA = math.pi * BEAM_RO**2 - math.pi * BEAM_RI**2
BEAM_IYY  = math.pi / 4.0 * (BEAM_RO**4 - BEAM_RI**4)
BEAM_IZZ  = BEAM_IYY
BEAM_J    = math.pi / 2.0 * (BEAM_RO**4 - BEAM_RI**4)

# Flywheel (disc) properties
FW_MASS   = 0.5               # kg
FW_RADIUS = 0.07              # m
FW_WIDTH  = 0.02              # m

# Motor speed
MOTOR_SPEED = 2.0 * math.pi * 6.0  # rad/s — 6 rev/s

# FEA beam: no contact material needed — driven by constraints + gravity + motor only

# === System & gravity ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up

# Pardiso MKL solver — required for stiff IGA beams
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical minimal form for stiff beam/IGA
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Fixed truss (ground reference) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.Add(truss)

# === FEA mesh — IGA / Cosserat beam ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Build Cosserat inertia section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(SHAFT_DENSITY)
minertia.SetArea(BEAM_AREA)
minertia.SetIyy(BEAM_IYY)
minertia.SetIzz(BEAM_IZZ)

# Build Cosserat elasticity section
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(SHAFT_E)
melasticity.SetShearModulusFromPoisson(SHAFT_NU)
melasticity.SetIyy(BEAM_IYY)
melasticity.SetIzz(BEAM_IZZ)
melasticity.SetJ(BEAM_J)

# Combine into section; set draw radius for visualization
msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(BEAM_RO)

# Build IGA beam from X=0 to X=BEAM_L (along world X axis)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    N_SPANS,
    chrono.ChVector3d(0.0, 0.0, 0.0),    # A — left end
    chrono.ChVector3d(BEAM_L, 0.0, 0.0), # B — right end
    chrono.VECT_Y,                        # suggested section Y direction
    IGA_ORDER
)

# SWIG GC guard: store node container before indexing
beam_nodes = builder.GetLastBeamNodes()
node_list  = [beam_nodes[i] for i in range(beam_nodes.size())]  # cache: avoids GC

node_A   = node_list[0]          # left bearing node
node_B   = node_list[-1]         # right bearing node (motor end)
node_mid = node_list[len(node_list) // 2]  # midpoint for flywheel

sys.Add(mesh)

# === Flywheel (rigid disc welded at beam midpoint) ===
flywheel = chrono.ChBody()
flywheel.SetName("flywheel")
flywheel.SetMass(FW_MASS)
flywheel.SetInertiaXX(chrono.ChVector3d(
    0.5 * FW_MASS * FW_RADIUS**2,                     # Ixx (axial)
    0.25 * FW_MASS * FW_RADIUS**2 + FW_MASS * FW_WIDTH**2 / 12.0,  # Iyy (diametral)
    0.25 * FW_MASS * FW_RADIUS**2 + FW_MASS * FW_WIDTH**2 / 12.0,  # Izz (diametral)
))
flywheel.SetPos(node_mid.GetPos())
sys.Add(flywheel)

# Visual cylinder for the flywheel disc (oriented along X = shaft axis)
fw_cyl = chrono.ChVisualShapeCylinder(FW_RADIUS, FW_WIDTH)
flywheel.AddVisualShape(
    fw_cyl,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))
)

# Weld flywheel to mid-beam node (all 6 DOF fixed)
weld_fw = chrono.ChLinkMateFix()
weld_fw.Initialize(node_mid, flywheel)
sys.Add(weld_fw)

# === Bearings — ChLinkMateGeneric constraining xyz + ry + rz (leaving rx free) ===
# Left bearing: constrain tx, ty, tz, ry, rz (leave shaft rotation about X free)
bearing_A = chrono.ChLinkMateGeneric()
bearing_A.Initialize(node_A, truss, False, node_A.Frame(), node_A.Frame())
bearing_A.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(bearing_A)

# Right bearing: same constraint pattern — motor drives rotation, bearing holds position
bearing_B = chrono.ChLinkMateGeneric()
bearing_B.Initialize(node_B, truss, False, node_B.Frame(), node_B.Frame())
bearing_B.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(bearing_B)

# === Rotational motor at right end (node_B → truss) — ChLinkMotorRotationSpeed ===
# Full motor-link; no separate revolute needed
motor_frame = chrono.ChFramed(
    node_B.GetPos(),
    chrono.QuatFromAngleY(chrono.CH_PI_2)  # local +Z onto world +X (shaft axis)
)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_B.GetBodyPtr() if hasattr(node_B, 'GetBodyPtr') else flywheel,
                 truss, motor_frame)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
# The motor drives the beam end via a direct node approach — use bearing_B node directly
# Re-initialize: motor links body objects; for FEA beam end we need a hub body
# Use a hub body welded to node_B instead
sys.Remove(motor)  # remove stale attempt

# Hub body at motor end — welded to beam end node
hub = chrono.ChBody()
hub.SetName("motor_hub")
hub.SetMass(0.01)
hub.SetInertiaXX(chrono.ChVector3d(1e-5, 1e-5, 1e-5))
hub.SetPos(node_B.GetPos())
sys.Add(hub)

hub_cyl = chrono.ChVisualShapeCylinder(BEAM_RO * 2, BEAM_RO * 4)
hub.AddVisualShape(hub_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

weld_hub = chrono.ChLinkMateFix()
weld_hub.Initialize(node_B, hub)
sys.Add(weld_hub)

# Remove bearing_B (hub takes over the constraint) — re-add using hub
sys.Remove(bearing_B)

# Bearing B on hub body: constrain tx, ty, tz, ry, rz
bearing_B2 = chrono.ChLinkMateGeneric()
bearing_B2.Initialize(hub, truss, False,
                      chrono.ChFramed(node_B.GetPos()),
                      chrono.ChFramed(node_B.GetPos()))
bearing_B2.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(bearing_B2)

# Motor on hub: spin hub (and thus beam) at MOTOR_SPEED about shaft axis (X)
motor2 = chrono.ChLinkMotorRotationSpeed()
motor2.Initialize(
    hub, truss,
    chrono.ChFramed(node_B.GetPos(), chrono.QuatFromAngleY(chrono.CH_PI_2))
)
motor2.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.Add(motor2)

# Pre-solve static step to settle rotor under gravity before dynamic stepping
sys.DoStaticLinear()

# === FEA Visualization (attach to mesh BEFORE vis.Initialize) ===
# Shape 1 — deformed surface (beam surface coloured by speed norm)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyph coordinate systems
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor — IGA Beam with Flywheel")
vis.Initialize()                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, 0.4, -1.2), chrono.ChVector3d(0.6, 0.0, 0.0))
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
