"""
Jeffcott Rotor Simulation — IGA Cosserat beam with welded flywheel and sine speed motor.

System:     ChSystemSMC + Pardiso MKL solver + Euler Implicit Linearized timestepper.
Beam:       IGA Cosserat beam, length=10 m, hollow circular cross-section
            (outer radius 0.060 m, inner radius 0.055 m), steel density 7800 kg/m³.
Flywheel:   Rigid cylinder body (radius 0.30 m, height 0.10 m, density 7800 kg/m³)
            welded to the beam midpoint node via ChLinkMateFix.
Motor:      ChLinkMotorRotationSpeed at beam root node (bearing A), driven by
            ChFunctionSine(amplitude=60 rad/s, frequency=0.1 Hz) — sinusoidal speed
            excitation that spins the rotor shaft.
Bearings:   Motor (full motor-link) at node A constrains translation and transverse
            rotation; ChLinkMateGeneric at node B constrains translation + ry, rz,
            allowing rx (shaft rotation) — simulating journal bearings at both ends.
            Flywheel welded to mid-beam node via ChLinkMateFix.
Gravity:    (0, -3.71, 0) — Mars-like reduced gravitational acceleration.
Camera:     Eye at (0, 2, 8) looking at origin.
Expected:   Flexible rotor shaft whirls under sinusoidal speed excitation;
            flywheel oscillates laterally due to shaft bending + gravity.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Beam geometry
beam_L    = 10.0          # shaft length [m]
beam_ro   = 0.060         # outer radius [m]
beam_ri   = 0.055         # inner radius [m]
n_spans   = 10            # IGA span count
iga_order = 3             # cubic B-splines

# Derived cross-section properties — hollow circular shaft
area = math.pi * (beam_ro**2 - beam_ri**2)
Iyy  = math.pi / 4.0 * (beam_ro**4 - beam_ri**4)
Izz  = Iyy
J    = math.pi / 2.0 * (beam_ro**4 - beam_ri**4)

# Flywheel
fw_radius  = 0.30     # [m] — updated per prompt
fw_height  = 0.10     # [m]
fw_density = 7800.0   # [kg/m³] steel

# Gravity — Mars-like reduced gravity
grav_y = -3.71        # [m/s²]

# Motor: sine speed function
motor_amplitude = 60.0   # [rad/s] amplitude of ChFunctionSine
motor_frequency = 0.1    # [Hz]

# Simulation timing
time_step    = 0.002   # [s]
sim_end      = 10.0    # [s]
render_fps   = 50.0    # [Hz]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Camera (modified per prompt)
cam_eye    = chrono.ChVector3d(0, 2, 8)
cam_target = chrono.ChVector3d(0, 0, 0)

# === System & gravity ===
# FEA scenes require ChSystemSMC; no collision body contact — collision system omitted
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, grav_y, 0))

# Pardiso MKL direct solver — required for stiff IGA beam stiffness matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())

# Euler implicit linearized timestepper — stable for IGA Cosserat beam with large rotations
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA beam mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# IGA Cosserat section — hollow circular shaft, steel
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800.0)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
# SetDrawCircularRadius sets the visual rendering radius without overwriting Iyy/Izz/J
msection.SetDrawCircularRadius(beam_ro)

# Build IGA beam from origin to beam_L along world X axis
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    n_spans,
    chrono.ChVector3d(0, 0, 0),        # beam root  (node A — motor/bearing A)
    chrono.ChVector3d(beam_L, 0, 0),   # beam tip   (node B — bearing B)
    chrono.VECT_Y,                     # suggested section Y direction
    iga_order,                         # cubic splines
)

# Strong Python refs to prevent SWIG garbage-collection of node shared_ptrs
beam_nodes_ref = builder.GetLastBeamNodes()
all_nodes = [beam_nodes_ref[i] for i in range(beam_nodes_ref.size())]
node_A   = all_nodes[0]              # beam root — motor + bearing A
node_B   = all_nodes[-1]            # beam tip  — bearing B
n_mid    = len(all_nodes) // 2
node_mid = all_nodes[n_mid]          # midpoint  — flywheel attachment

sys.Add(mesh)

# === Flywheel body (rigid cylinder) ===
# FEA beam: no contact material needed — driven by constraints + gravity + motor only
mbodyflywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y, fw_radius, fw_height, fw_density
)
mbodyflywheel.SetPos(chrono.ChVector3d(beam_L / 2.0, 0, 0))
sys.Add(mbodyflywheel)

# === Fixed truss (ground reference for motor and bearing B) ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(truss)

# === Constraints ===

# --- Weld flywheel to midpoint node (all 6 DOF rigidly fixed) ---
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, mbodyflywheel)
sys.Add(weld)

# --- Motor at beam root (node A): ChLinkMotorRotationSpeed is a full motor-link ---
# It constrains tx, ty, tz, ry, rz of node_A to truss AND prescribes rx (shaft speed)
# No separate bearing_A constraint needed — the motor-link covers it
f_ramp = chrono.ChFunctionSine(motor_amplitude, motor_frequency)
motor = chrono.ChLinkMotorRotationSpeed()
# Align motor frame: local +Z must coincide with beam axis (world +X)
q_motor_axis = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_Y)
motor.Initialize(
    node_A, truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), q_motor_axis),
)
motor.SetSpeedFunction(f_ramp)
sys.Add(motor)

# --- Bearing B (beam tip): constrain tx, ty, tz + ry, rz; allow rx (shaft rotation) ---
bearing_B = chrono.ChLinkMateGeneric()
bearing_B.Initialize(node_B, truss, False, node_B.Frame(), node_B.Frame())
bearing_B.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(bearing_B)

# === FEA Visualization (two shapes: surface + glyphs) ===
# Shape 1 — deformed surface colored by displacement
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node coordinate-system glyphs
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization (Irrlicht) ===
# Irrlicht call order: Initialize() FIRST, then scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor — IGA L=10, Flywheel R=0.30, Sine Motor, Mars Gravity")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(cam_eye, cam_target)
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only recording setup ===


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
except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
