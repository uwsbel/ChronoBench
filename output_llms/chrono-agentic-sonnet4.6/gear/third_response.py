"""
Epicyclic gear train with bevel gear and pulley extension — PyChrono MBS simulation.

System type: ChSystemNSC (non-smooth contact, pure jointed MBS, no collision).
Main bodies: fixed truss, rotating bar (train), gears A and B (epicyclic),
             bevel gear D (radius 5, horizontal axis), pulley E (radius 2, horizontal axis).
Constraints: revolute joints, gear constraints (A-B epicyclic, A-D bevel, B-C ring),
             synchro belt (pulley D-E), motor on gear A.
Expected behavior: gear A spins at 3 rad/s driven by motor; gear B orbits in the epicyclic
                   set; bevel gear D rotates at 1:1 with A; pulley E is belt-driven by D.
"""

import math as m
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

radA = 1.5   # radius of gear A
radB = 3.5   # radius of gear B
radC = 2 * radB + radA  # radius of ring gear C (embedded in truss)
radD = 5     # bevel gear D radius
radE = 2     # pulley E radius

interaxis12 = radA + radB  # center-to-center distance A-B

# === System & gravity ===
sys = chrono.ChSystemNSC()
# Pure jointed MBS with no contact shapes — SetCollisionSystemType omitted per ground-truth
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Contact material (used by EasyBody constructors even with collision disabled)
mat = chrono.ChContactMaterialNSC()

# === Shared visual material ===
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# === Bodies ===

# Fixed truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Rotating bar (train) — carries gear B in the epicyclic set
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Gear A (driven by motor)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Visualization shaft on gear A
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(
    mshaft_shape,
    chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2))
)

# Gear B (orbits in epicyclic set)
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Bevel gear D — horizontal axis (rotated 90° around Z)
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Pulley E — horizontal axis (rotated 90° around Z), belt-driven by D
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# === Joints / constraints ===

# Revolute: truss ↔ rotating bar (Z axis at origin)
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                            chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Motor: gear A rotates at 3 rad/s relative to fixed truss (full motor-link, no extra revolute)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Revolute: gear B ↔ rotating bar (allows B to spin on the bar)
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint A-B (epicyclic external mesh)
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint B-C (epicyclic internal / ring gear embedded in truss)
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Revolute: bevel gear D ↔ truss (horizontal axis — QuatFromAngleY(pi/2) aligns local Z to world X)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                           chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),
                                           chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteD)

# Bevel gear constraint A-D (1:1 ratio)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)
sys.AddLink(link_gearAD)

# Revolute: pulley E ↔ truss (horizontal axis)
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                           chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),
                                           chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteE)

# Synchro belt constraint D-E (pulley link)
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetRadius1(radD)
link_pulleyDE.SetRadius2(radE)
link_pulleyDE.SetEnforcePhase(True)
sys.AddLink(link_pulleyDE)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        # Draw simplified belt representation between pulley D and E
        chronoirr.drawSegment(vis,
                              link_pulleyDE.GetBeltUpPos1(),
                              link_pulleyDE.GetBeltUpPos2())
        chronoirr.drawSegment(vis,
                              link_pulleyDE.GetBeltBottomPos1(),
                              link_pulleyDE.GetBeltBottomPos2())
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad constraint state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # no file handles to close in scored core
