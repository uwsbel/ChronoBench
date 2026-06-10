import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system for the gear/pulley train

mat = chrono.ChContactMaterialNSC()                                   # contact material shared among all bodies

radA = 2                                                              # sun gear A radius
radB = 4                                                              # planet gear B radius

# ...the truss (fixed); it also serves as the internal ring gear C of the epicyclic set
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)                                            # truss is fixed to ground
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                        # truss plate

vis_mat = chrono.ChVisualMaterial()                                   # shared visualization material
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))

# ...the rotating bar (carrier) holding the epicyclic planet axle
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))                        # carrier arm

# ...the carrier rotates wrt the truss about the Z axis at the origin (concentric with the sun)
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# ...the first (sun) gear A
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                       # sun on the spin axis
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                   # lay the disc so its spin axis is world Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)          # thin shaft cylinder, visualization only
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# ...impose a rotation speed between the sun gear A and the fixed truss (a FULL motor-link, no extra revolute)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(6))               # constant 6 rad/s on the sun
sys.AddLink(link_motor)

# ...the second (planet) gear B
interaxis12 = radA + radB                                            # sun-planet center distance
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))            # planet at the center distance
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # disc spin axis to world Z
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...the planet gear B revolutes to the CARRIER (so its axle orbits with the carrier)
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                        chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# ...the EXTERNAL sun-planet gear mesh between wheels A and B (ratio radA/radB, phased)
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)                        # external mesh ratio
link_gearAB.SetEnforcePhase(True)                                    # keep teeth phased
sys.AddLink(link_gearAB)

# ...the INTERNAL gear mesh between planet B and the large ring C; ring C IS the fixed truss
radC = 2 * radB + radA                                               # internal ring radius = 2*radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)                        # internal mesh ratio
link_gearBC.SetEpicyclic(True)                                       # internal-teeth (ring) mesh
sys.AddLink(link_gearBC)

# ...the bevel gear D at the side
radD = 5                                                             # bevel gear D radius
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))                    # gear D position
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                  # rotate 90 deg about Z -> spin axis is horizontal
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...D is fixed to the truss with a revolute joint about the horizontal axis (90 deg about Y)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteD)

# ...a 1:1 bevel gear between wheel A and wheel D (only the shaft positions + ratio are needed)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)                                  # 1:1 gear ratio A:D
sys.AddLink(link_gearAD)

# ...the pulley E at the side
radE = 2                                                             # pulley E radius
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))               # pulley E position
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))               # rotate 90 deg about Z -> spin axis is horizontal
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...E is fixed to the truss with a revolute joint about the horizontal axis (90 deg about Y)
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteE)

# ...a synchro-belt constraint between bevel gear D and pulley E (parallel shafts, fixed interaxis)
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetRadius1(radD)                                       # belt radius on gear D
link_pulleyDE.SetRadius2(radE)                                       # belt radius on pulley E
link_pulleyDE.SetEnforcePhase(True)                                  # synchro belts must not slip
sys.AddLink(link_pulleyDE)

# Irrlicht visualization (Initialize FIRST, then scene elements; NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Gears and pulleys")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                        # frame the gear plane + the bevel/pulley
vis.AddTypicalLights()

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)   # projected integrator for stable meshing

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                    # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    # simplified visual representation of the belt linking gear D and pulley E
    chronoirr.drawSegment(vis, link_pulleyDE.GetBeltUpPos1(), link_pulleyDE.GetBeltUpPos2())
    chronoirr.drawSegment(vis, link_pulleyDE.GetBeltBottomPos1(), link_pulleyDE.GetBeltBottomPos2())
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
