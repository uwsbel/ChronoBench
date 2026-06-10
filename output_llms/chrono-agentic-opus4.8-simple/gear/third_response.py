import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC multibody system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))     # g points along -Z
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # Bullet collision detection

mat = chrono.ChContactMaterialNSC()                                  # shared contact material

vis_mat = chrono.ChVisualMaterial()                                  # shared visual material for the gears
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))   # gear teeth texture

radA = 2                                                             # radius of first gear A
radB = 4                                                             # radius of second gear B

mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)   # the fixed truss
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)                                           # truss is the fixed reference
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                       # raised above the gears

mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)   # rotating carrier bar
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))                       # spans the two epicyclic wheels

link_revoluteTT = chrono.ChLinkLockRevolute()                       # carrier spins about world Z at origin
link_revoluteTT.Initialize(mbody_truss, mbody_train, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000)   # first gear A (cylinder along Y)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                      # on the central axis
mbody_gearA.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))           # tip its Y axis into the world Z axis
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)         # thin shaft, aesthetic only
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))

link_motor = chrono.ChLinkMotorRotationSpeed()                      # drive gear A against the fixed truss
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(6))             # constant 6 rad/s
sys.AddLink(link_motor)

interaxis12 = radA + radB                                           # center distance A-B
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000)   # second gear B
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))           # offset by the center distance
mbody_gearB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))          # same axis orientation as A
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)              # textured gear face

link_revolute = chrono.ChLinkLockRevolute()                        # B pinned to the rotating carrier
link_revolute.Initialize(mbody_gearB, mbody_train, chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

link_gearAB = chrono.ChLinkLockGear()                              # gear mesh A-B
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))   # shaft axis along Z
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearAB.SetTransmissionRatio(radA / radB)                     # ratio from the two radii
link_gearAB.SetEnforcePhase(True)                                # keep teeth phased
sys.AddLink(link_gearAB)

radC = 2 * radB + radA                                            # radius of internal ring gear C (the truss)
link_gearBC = chrono.ChLinkLockGear()                            # epicyclic mesh B against ring C
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)                                   # ring has internal teeth
sys.AddLink(link_gearBC)

radD = 5                                                          # radius of bevel gear D
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000)   # bevel gear D
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))                # at the side of the train
mbody_gearD.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))       # rotated 90 deg about world Z
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)          # textured face

link_revoluteD = chrono.ChLinkLockRevolute()                    # D pinned to truss, horizontal (Y) axis
link_revoluteD.Initialize(mbody_gearD, mbody_truss, chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddLink(link_revoluteD)

link_gearAD = chrono.ChLinkLockGear()                           # 1:1 bevel mesh between A and D
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearAD.SetTransmissionRatio(1)                            # 1:1 gear ratio A:D
sys.AddLink(link_gearAD)

radE = 2                                                        # radius of pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000)   # pulley E
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))         # below gear D
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))  # rotated 90 deg about world Z
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)     # textured face

link_revoluteE = chrono.ChLinkLockRevolute()                  # E pinned to truss, horizontal (Y) axis
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss, chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddLink(link_revoluteE)

link_pulleyDE = chrono.ChLinkLockPulley()                     # synchro belt between D and E
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))   # shaft axis along Z
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_pulleyDE.SetRadius1(radD)                               # pulley radius on D
link_pulleyDE.SetRadius2(radE)                               # pulley radius on E
link_pulleyDE.SetEnforcePhase(True)                          # synchro belt does not slip
sys.AddLink(link_pulleyDE)

vis = chronoirr.ChVisualSystemIrrlicht()                     # Irrlicht renderer
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)                                 # window resolution
vis.SetWindowTitle("Gears and pulleys")
vis.Initialize()                                             # create device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))               # eye looking at the gear train
vis.AddTypicalLights()

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)   # stable timestepper for the constraints

time_step = 0.001                                            # integration step
sim_end = 10.0                                              # stop after 10 s
render_fps = 50.0                                           # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    chronoirr.drawSegment(vis, link_pulleyDE.GetBeltUpPos1(), link_pulleyDE.GetBeltUpPos2(), chrono.ChColor(0, 1, 0), True)        # belt upper run
    chronoirr.drawSegment(vis, link_pulleyDE.GetBeltBottomPos1(), link_pulleyDE.GetBeltBottomPos2(), chrono.ChColor(0, 1, 0), True)   # belt lower run
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
