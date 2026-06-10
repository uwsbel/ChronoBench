import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # g = 9.81 down (Y-up)

radA = 4.0                                                           # gear A pitch radius
radB = 6.0                                                           # gear B pitch radius
radC = 4.0                                                           # gear C / driving pulley radius
radD = 5.0                                                           # bevel gear D radius
radE = 2.0                                                           # pulley E radius
gear_w = 0.5                                                         # gear disc thickness
gear_rho = 1000.0                                                    # gear material density

mat = chrono.ChContactMaterialNSC()                                  # shared contact material

# truss / fixed support — all revolute joints attach to it
truss = chrono.ChBody()                                             # fixed reference frame
truss.SetFixed(True)                                                # immovable truss
sys.AddBody(truss)

bar_vis = chrono.ChVisualShapeBox(0.4, 0.4, 12.0)                   # truss cross-bar visual
bar_vis.SetColor(chrono.ChColor(0.0, 0.0, 0.0))                     # black truss
truss.AddVisualShape(bar_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# gear A — driven by the motor, spins about world Z
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radA, gear_w, gear_rho)  # disc A
gearA.SetPos(chrono.ChVector3d(0, 0, -1))                           # A at origin column
gearA.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.3, 0.1))     # orange A
sys.AddBody(gearA)

linkA = chrono.ChLinkLockRevolute()                                # A ↔ truss hinge about Z
linkA.Initialize(gearA, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))
sys.AddLink(linkA)

# motor spins gear A at a constant rate
motor = chrono.ChLinkMotorRotationSpeed()                           # prescribed-speed motor (full link)
motor.Initialize(gearA, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))
motor.SetSpeedFunction(chrono.ChFunctionConst(6.0))                 # 6 rad/s about Z
sys.AddLink(motor)

# gear B — meshes with A, sits radA+radB to the right
posB = chrono.ChVector3d(radA + radB, 0, -1)                        # tangent contact with A
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radB, gear_w, gear_rho)  # disc B
gearB.SetPos(posB)
gearB.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.4, 0.8))     # blue B
sys.AddBody(gearB)

linkB = chrono.ChLinkLockRevolute()                                # B ↔ truss hinge about Z
linkB.Initialize(gearB, truss, chrono.ChFramed(posB))
sys.AddLink(linkB)

# gear constraint A ↔ B (spur mesh, parallel Z shafts)
gearAB = chrono.ChLinkLockGear()                                   # spur gear pair A-B
gearAB.Initialize(gearA, gearB, chrono.ChFramed())
gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QUNIT)) # A shaft along Z
gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QUNIT)) # B shaft along Z
gearAB.SetTransmissionRatio(radA / radB)                            # ratio rA/rB
gearAB.SetEnforcePhase(True)                                        # keep teeth phased
sys.AddLink(gearAB)

# gear C — second spur gear coaxial-train piece, meshes with B
posC = chrono.ChVector3d(radA + radB + radB + radC, 0, -1)         # to the right of B
gearC = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radC, gear_w, gear_rho)  # disc C
gearC.SetPos(posC)
gearC.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.7, 0.2))     # green C
sys.AddBody(gearC)

linkC = chrono.ChLinkLockRevolute()                                # C ↔ truss hinge about Z
linkC.Initialize(gearC, truss, chrono.ChFramed(posC))
sys.AddLink(linkC)

gearBC = chrono.ChLinkLockGear()                                   # spur gear pair B-C
gearBC.Initialize(gearB, gearC, chrono.ChFramed())
gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QUNIT)) # B shaft along Z
gearBC.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QUNIT)) # C shaft along Z
gearBC.SetTransmissionRatio(radB / radC)                            # ratio rB/rC
gearBC.SetEnforcePhase(True)                                        # keep teeth phased
sys.AddLink(gearBC)

# bevel gear D — radius 5, at (-10,0,-9), rotated 90 deg about Z, hinged about horizontal axis
posD = chrono.ChVector3d(-10, 0, -9)                                # bevel gear D location
rotD = chrono.QuatFromAngleZ(m.pi / 2)                              # 90 deg about Z
gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radD, gear_w, gear_rho)  # disc D
gearD.SetPos(posD)
gearD.SetRot(rotD)                                                  # rotate D by 90 deg about Z
gearD.GetVisualShape(0).SetColor(chrono.ChColor(0.7, 0.2, 0.7))     # purple D
sys.AddBody(gearD)

# D ↔ truss revolute along the horizontal axis (hinge about world X → local +Z mapped to X)
q_horiz = chrono.QuatFromAngleY(chrono.CH_PI_2)                     # local +Z → world X
linkD = chrono.ChLinkLockRevolute()                                # D ↔ truss horizontal hinge
linkD.Initialize(gearD, truss, chrono.ChFramed(posD, q_horiz))
sys.AddLink(linkD)

# 1:1 bevel gear constraint A ↔ D
gearAD = chrono.ChLinkLockGear()                                   # bevel gear pair A-D
gearAD.Initialize(gearA, gearD, chrono.ChFramed())
gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))  # A bevel shaft
gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))  # D bevel shaft
gearAD.SetTransmissionRatio(1.0)                                    # 1:1 ratio A-D
gearAD.SetEnforcePhase(True)                                        # phased bevel teeth
sys.AddLink(gearAD)

# pulley E — radius 2, at (-10,-11,-9), rotated 90 deg about Z, horizontal hinge
posE = chrono.ChVector3d(-10, -11, -9)                              # pulley E location
rotE = chrono.QuatFromAngleZ(m.pi / 2)                             # 90 deg about Z
pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radE, gear_w, gear_rho)  # disc E
pulleyE.SetPos(posE)
pulleyE.SetRot(rotE)                                               # rotate E by 90 deg about Z
pulleyE.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.7, 0.1))  # gold E
sys.AddBody(pulleyE)

linkE = chrono.ChLinkLockRevolute()                               # E ↔ truss horizontal hinge
linkE.Initialize(pulleyE, truss, chrono.ChFramed(posE, q_horiz))
sys.AddLink(linkE)

# synchro belt constraint D ↔ E (pulley / toothed belt)
belt = chrono.ChLinkLockPulley()                                  # synchro belt D-E
belt.Initialize(gearD, pulleyE, chrono.ChFramed())
belt.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))  # D shaft
belt.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))  # E shaft
belt.SetRadius1(radD)                                             # belt radius on D
belt.SetRadius2(radE)                                             # belt radius on E
belt.SetEnforcePhase(True)                                        # synchro (toothed) belt
sys.AddLink(belt)

# simplified visual belt — a thin segment linking the two pulley centers
belt_vis = chrono.ChVisualShapeLine()                            # belt line geometry
belt_vis.SetLineGeometry(chrono.ChLineSegment(posD, posE))       # D center to E center
belt_vis.SetColor(chrono.ChColor(0.1, 0.1, 0.1))                 # dark belt
truss.AddVisualShape(belt_vis)                                   # attach to fixed truss

# Irrlicht visualization — Initialize first, then scene elements (NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gears, bevel gear and synchro-belt pulley")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 12, -20), chrono.ChVector3d(0, -3, -5))
vis.AddTypicalLights()

time_step = 1e-3                                                  # integration step
sim_end = 12.0                                                    # simulation duration
render_fps = 50.0                                                 # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
