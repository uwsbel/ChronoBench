"""
Gear train simulation with bevel gear D and pulley E.

Changes from turn2 (base):
  - Add bevel gear D (radius 5) at (-10, 0, -9) with revolute joint to truss.
  - Add pulley E (radius 2) at (-10, -11, -9) with revolute joint to truss.
  - Synchro belt constraint between D and E (ratio radD/radE).
  - Belt visual connecting D and E.
"""

import csv
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Base turn2 parameters ===
radA = 1.5
radB = 3.5
density = 1000.0

truss_w = 15.0
truss_d = 8.0
truss_h = 2.0

interaxis = radA + radB
gearA_y = 0.0
gearB_y = 0.0
gearA_z = 0.0
gearB_z = -2.0

motor_speed = 3.0  # rad/s

shaft_radius = radA * 0.3
shaft_length = 10.0

# === New gear D and pulley E parameters (per input3.txt) ===
radD = 5.0
radE = 2.0
gearD_x = -10.0
gearD_y = 0.0
gearD_z = -9.0
pulleyE_x = -10.0
pulleyE_y = -11.0
pulleyE_z = -9.0

belt_length = abs(pulleyE_y - gearD_y)  # = 11.0
belt_midpoint_y = (gearD_y + pulleyE_y) / 2.0  # = -5.5
belt_radius = 0.2

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Materials ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.3)
mat.SetRestitution(0.0)

# === Ground / Truss ===
truss = chrono.ChBodyEasyBox(truss_w, truss_d, truss_h, density, True, False, mat)
truss.SetFixed(True)
sys.AddBody(truss)

# === Gear A (driver) ===
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, radA * 0.5, density, True, True, mat)
gearA.SetPos(chrono.ChVector3d(0, gearA_y, gearA_z))
sys.AddBody(gearA)

# === Gear B (driven) ===
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, radB * 0.5, density, True, True, mat)
gearB.SetPos(chrono.ChVector3d(interaxis, gearB_y, gearB_z))
sys.AddBody(gearB)

# === Motor driving Gear A ===
motor_link = chrono.ChLinkMotorRotationSpeed()
motor_link.Initialize(
    gearA, truss,
    chrono.ChFramed(chrono.ChVector3d(0, gearA_y, gearA_z), chrono.QUNIT)
)
motor_link.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor_link)

# === Gear mesh constraint (A and B) ===
gear_mesh = chrono.ChLinkLockGear()
gear_mesh.Initialize(gearA, gearB, chrono.ChFramed())
gear_mesh.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gear_mesh.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gear_mesh.SetTransmissionRatio(radA / radB)
gear_mesh.SetEnforcePhase(True)
sys.AddLink(gear_mesh)

# === Visual shaft for Gear A ===
shaft_cyl = chrono.ChVisualShapeCylinder(shaft_radius, shaft_length)
shaft_cyl.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
gearA.AddVisualShape(shaft_cyl, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleY(math.pi / 2)
))

# === Gear D (bevel gear, radius 5) ===
gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, radD * 0.5, density, True, True, mat)
gearD.SetPos(chrono.ChVector3d(gearD_x, gearD_y, gearD_z))
sys.AddBody(gearD)

# Visual: bevel gear distinctive color
gearD.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.5, 0.1))

# Shaft for gear D
shaftD_len = 10.0
shaftD_rad = radD * 0.15
shaftD_cyl = chrono.ChVisualShapeCylinder(shaftD_rad, shaftD_len)
shaftD_cyl.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
gearD.AddVisualShape(shaftD_cyl, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleY(math.pi / 2)
))

# === Pulley E (radius 2) ===
pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, radE * 0.5, density, True, True, mat)
pulleyE.SetPos(chrono.ChVector3d(pulleyE_x, pulleyE_y, pulleyE_z))
sys.AddBody(pulleyE)

# Visual: pulley distinctive color
pulleyE.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.9))

# Shaft for pulley E
shaftE_len = 10.0
shaftE_rad = radE * 0.2
shaftE_cyl = chrono.ChVisualShapeCylinder(shaftE_rad, shaftE_len)
shaftE_cyl.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
pulleyE.AddVisualShape(shaftE_cyl, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleY(math.pi / 2)
))

# === Revolute: gear D to truss ===
revoluteD = chrono.ChLinkLockRevolute()
revoluteD.Initialize(
    gearD, truss,
    chrono.ChFramed(chrono.ChVector3d(gearD_x, gearD_y, gearD_z), chrono.QUNIT)
)
sys.AddLink(revoluteD)

# === Revolute: pulley E to truss ===
revoluteE = chrono.ChLinkLockRevolute()
revoluteE.Initialize(
    pulleyE, truss,
    chrono.ChFramed(chrono.ChVector3d(pulleyE_x, pulleyE_y, pulleyE_z), chrono.QUNIT)
)
sys.AddLink(revoluteE)

# === Belt constraint: gear D and pulley E (synchro, ratio radD/radE) ===
belt_constraint = chrono.ChLinkLockGear()
belt_constraint.Initialize(gearD, pulleyE, chrono.ChFramed())
belt_constraint.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
belt_constraint.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
belt_constraint.SetTransmissionRatio(radD / radE)  # = 5/2 = 2.5
belt_constraint.SetEnforcePhase(True)
sys.AddLink(belt_constraint)

# === Belt visual connecting gear D and pulley E ===
belt_vis = chrono.ChVisualShapeCylinder(belt_radius, belt_length)
belt_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
gearD.AddVisualShape(belt_vis, chrono.ChFramed(
    chrono.ChVector3d(0, belt_midpoint_y - gearD_y, 0),
    chrono.QuatFromAngleY(math.pi / 2)
))

# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train with Bevel Gear and Pulley")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -20, 5), chrono.ChVector3d(-5, -5, -5))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -3), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Simulation loop ===
time_step = 1e-3
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
