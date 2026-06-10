"""
Gear train simulation — two meshing spur gears driven by a speed motor.

Changes from base (input2.txt):
  radA = 1.5 (was 2), radB = 3.5 (was 4)
  Truss dimensions: 15 x 8 x 2 (was 20 x 10 x 2)
  Motor speed: 3 rad/s (was 6)
  Gear B z-position: -2 (was -1)
  Visual shaft: radA*0.3 x 10 (was radA*0.4 x 13)
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters ===
radA = 1.5
radB = 3.5
density = 1000.0

# Truss dimensions (modified)
truss_w = 15.0
truss_d = 8.0
truss_h = 2.0

# Gear positions
interaxis = radA + radB
gearA_y = 0.0
gearB_y = 0.0
gearA_z = 0.0
gearB_z = -2.0  # modified from -1

# Motor speed (modified)
motor_speed = 3.0  # rad/s

# Shaft visual dimensions (modified)
shaft_radius = radA * 0.3  # was radA * 0.4
shaft_length = 10.0         # was 13

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(50)

# === Materials ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.95)
mat.SetRestitution(0.0)

# === Ground / Truss ===
truss = chrono.ChBodyEasyBox(truss_w, truss_d, truss_h, density, True, False, mat)
truss.SetFixed(True)
sys.AddBody(truss)

# === Gear A (driver) - cylinder axis along Y ===
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, radA * 0.5, density, True, True, mat)
gearA.SetPos(chrono.ChVector3d(0, gearA_y, gearA_z))
sys.AddBody(gearA)

# === Gear B (driven) - cylinder axis along Y ===
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, radB * 0.5, density, True, True, mat)
gearB.SetPos(chrono.ChVector3d(interaxis, gearB_y, gearB_z))
sys.AddBody(gearB)

# === Motor driving Gear A (ChLinkMotorRotationSpeed is a FULL motor-link) ===
motor_link = chrono.ChLinkMotorRotationSpeed()
motor_link.Initialize(
    gearA, truss,
    chrono.ChFramed(chrono.ChVector3d(0, gearA_y, gearA_z), chrono.QUNIT)
)
motor_link.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor_link)

# === Revolute joint for Gear B to truss (free rotation) ===
revB = chrono.ChLinkLockRevolute()
revB.Initialize(gearB, truss, chrono.ChFramed(chrono.ChVector3d(interaxis, gearB_y, gearB_z), chrono.QUNIT))
sys.AddLink(revB)

# === Gear mesh constraint (speed ratio between gears) ===
gear_mesh = chrono.ChLinkLockGear()
gear_mesh.Initialize(gearA, gearB, chrono.ChFramed())
q_align = chrono.QuatFromAngleX(-math.pi / 2)
gear_mesh.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, q_align))
gear_mesh.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, q_align))
gear_mesh.SetTransmissionRatio(radA / radB)
gear_mesh.SetEnforcePhase(False)
sys.AddLink(gear_mesh)

# === Visualization shaft (visual only) ===
shaft_cyl = chrono.ChVisualShapeCylinder(shaft_radius, shaft_length)
shaft_cyl.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
gearA.AddVisualShape(shaft_cyl, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleY(math.pi / 2)
))

# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(interaxis / 2, 0, -1))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -3), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Simulation loop ===
time_step = 5e-4
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
