"""
Gear Train Simulation — PyChrono MBS demo.

Models a two-gear spur gear train with a motor-driven input gear (A) meshing
with a larger output gear (B), both supported on a fixed truss body. The gears
are connected via a ChLinkLockGear constraint with the correct transmission
ratio. A ChLinkMotorRotationSpeed drives gear A at a constant angular speed.
Both gears are also constrained to the truss via revolute joints at their shaft
axes.

System type : ChSystemNSC (pure jointed MBS — no contact, no collision)
Main bodies : truss (fixed), mbody_gearA (input), mbody_gearB (output),
              mbody_shaftA (visual shaft)
Expected    : Gear A spins at 3 rad/s; gear B spins at radA/radB the rate.
              Smooth, stable kinematic motion with correct transmission ratio.
"""

import math as m
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
radA = 1.5          # radius of gear A (input)
radB = 3.5          # radius of gear B (output)
interaxis12 = radA + radB   # center-to-center distance

time_step = 1e-3
sim_end   = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


# === System & gravity ===
# Pure jointed MBS — no contact/collision, so SetCollisionSystemType is omitted
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# -- Contact material (visual-only truss uses this) --
mat = chrono.ChContactMaterialNSC()

# -- Truss (fixed support) --
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))
sys.AddBody(mbody_truss)

# -- Gear A (input, driven by motor) --
mbody_gearA = chrono.ChBody()
mbody_gearA.SetMass(1.0)
mbody_gearA.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
sys.AddBody(mbody_gearA)

# Visual shape for gear A: thin cylinder representing the gear disc
cylA = chrono.ChVisualShapeCylinder(radA, 0.5)
cylA.SetColor(chrono.ChColor(0.6, 0.3, 0.2))
mbody_gearA.AddVisualShape(cylA, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))

# -- Gear B (output) --
mbody_gearB = chrono.ChBody()
mbody_gearB.SetMass(3.0)
mbody_gearB.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
sys.AddBody(mbody_gearB)

# Visual shape for gear B: thin cylinder representing the gear disc
cylB = chrono.ChVisualShapeCylinder(radB, 0.5)
cylB.SetColor(chrono.ChColor(0.2, 0.4, 0.6))
mbody_gearB.AddVisualShape(cylB, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))

# -- Visual shaft A (axle connecting truss to gear A) --
mbody_shaftA = chrono.ChBody()
mbody_shaftA.SetMass(0.1)
mbody_shaftA.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
mbody_shaftA.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(mbody_shaftA)

# Visual shape for shaft A (modified size: radA*0.3 radius, length 10)
shaftA_cyl = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
shaftA_cyl.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
mbody_shaftA.AddVisualShape(shaftA_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))

# === Joints / constraints ===

# -- Motor: drives gear A at constant angular speed against the truss --
# ChLinkMotorRotationSpeed is a full motor-link (revolute + speed prescription)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(
    mbody_gearA, mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, -1), chrono.QuatFromAngleX(-m.pi / 2))
)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))  # 3 rad/s (modified)
sys.AddLink(link_motor)

# -- Revolute: gear B rotates freely on the truss --
link_revolute_B = chrono.ChLinkLockRevolute()
link_revolute_B.Initialize(
    mbody_gearB, mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -2), chrono.QuatFromAngleX(-m.pi / 2))
)
sys.AddLink(link_revolute_B)

# -- Lock joint: visual shaft A is rigidly fixed to gear A's rotation axis --
link_shaftA = chrono.ChLinkLockLock()
link_shaftA.Initialize(
    mbody_shaftA, mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)
sys.AddLink(link_shaftA)

# -- Gear constraint: couples gear A and gear B rotation --
link_gear = chrono.ChLinkLockGear()
link_gear.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gear.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetTransmissionRatio(radA / radB)  # correct gear ratio
link_gear.SetEnforcePhase(True)
sys.AddLink(link_gear)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train — PyChrono 9 MBS")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()  # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 6, -6), chrono.ChVector3d(3, 0, 0))
vis.AddTypicalLights()

# === CSV setup (review-only) ===

frame = 0

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad parameter
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # end of simulation loop
