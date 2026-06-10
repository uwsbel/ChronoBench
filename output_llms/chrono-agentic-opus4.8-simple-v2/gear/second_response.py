import os
import math as m
import pychrono as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                              # NSC system for the gear train

mat = chrono.ChContactMaterialNSC()                                     # contact material shared by all bodies

radA = 1.5                                                              # radius of gear A
radB = 3.5                                                              # radius of gear B

# the fixed truss that carries every axle
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)    # truss box (no collision)
sys.Add(mbody_truss)                                                    # add truss
mbody_truss.SetFixed(True)                                              # truss is the fixed reference
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                          # behind the gears

vis_mat = chrono.ChVisualMaterial()                                     # shared visual material
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  # pink/white gear texture

# the rotating bar carrier for the epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)  # carrier bar
sys.Add(mbody_train)                                                    # add carrier
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))                          # carrier center

# carrier rotates relative to the truss about Z at the origin
link_revoluteTT = chrono.ChLinkLockRevolute()                          # truss<->carrier hinge
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # axis Z at origin
sys.AddLink(link_revoluteTT)                                           # add carrier hinge

# gear A : the driven sun wheel
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)  # gear A disc
sys.Add(mbody_gearA)                                                    # add gear A
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                         # at origin plane
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                     # lay disc so its axis is along Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)                  # textured gear A

# thin shaft cylinder, visualization only
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)            # thin shaft, length 10
mbody_gearA.AddVisualShape(mshaft_shape,
    chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))  # along the shaft

# motor imposing a constant rotation speed between gear A and the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                        # prescribed-speed motor (full motor-link)
link_motor.Initialize(mbody_gearA, mbody_truss,
                       chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # drive about gear A axle
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))               # constant angular speed 3 rad/s
sys.AddLink(link_motor)                                              # add motor

# gear B : the planet wheel riding on the carrier
interaxis12 = radA + radB                                            # center distance between gears A and B
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)  # gear B disc
sys.Add(mbody_gearB)                                                 # add gear B
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           # offset to the side of gear A
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # lay disc so its axis is along Z
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)              # textured gear B

# gear B rides on the rotating carrier via a revolute joint
link_revolute = chrono.ChLinkLockRevolute()                        # carrier<->gear B hinge
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))  # parallel axle
sys.AddLink(link_revolute)                                         # add gear B hinge

# gear constraint imposing the A<->B transmission ratio (no tooth collision)
link_gearAB = chrono.ChLinkLockGear()                              # gear mesh A-B
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())  # connect gears A and B
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # gear A shaft axis
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # gear B shaft axis
link_gearAB.SetTransmissionRatio(radA / radB)                     # ratio = rA/rB
link_gearAB.SetEnforcePhase(True)                                # keep teeth phased
sys.AddLink(link_gearAB)                                          # add A-B mesh

# gear constraint between gear B and the fixed internal ring C (the truss)
radC = 2 * radB + radA                                            # ring radius (internal teeth)
link_gearBC = chrono.ChLinkLockGear()                            # gear mesh B-C
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())  # connect gear B to the ring
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # gear B shaft axis
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))  # ring axis
link_gearBC.SetTransmissionRatio(radB / radC)                   # ratio = rB/rC
link_gearBC.SetEpicyclic(True)                                  # internal-teeth (ring) mesh
sys.AddLink(link_gearBC)                                        # add B-C mesh

# bevel gear D at the side
radD = 5                                                        # radius of gear D
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000, True, False, mat)  # gear D disc
sys.Add(mbody_gearD)                                            # add gear D
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))             # off to the side
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))           # axis horizontal
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)        # textured gear D

# gear D is fixed to the truss with a horizontal-axis revolute
link_revoluteD = chrono.ChLinkLockRevolute()                  # truss<->gear D hinge
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleY(m.pi / 2)))  # axis along Z
sys.AddLink(link_revoluteD)                                  # add gear D hinge

# 1:1 bevel gear between wheel A and wheel D
link_gearAD = chrono.ChLinkLockGear()                        # gear mesh A-D
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())  # connect gears A and D
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))  # gear A shaft axis
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))  # gear D shaft axis
link_gearAD.SetTransmissionRatio(1)                          # 1:1 bevel mesh
sys.AddLink(link_gearAD)                                     # add A-D mesh

# pulley E at the side
radE = 2                                                     # radius of pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000, True, False, mat)  # pulley E disc
sys.Add(mbody_pulleyE)                                       # add pulley E
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))      # below gear D
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))      # axis horizontal
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)   # textured pulley E

# pulley E is fixed to the truss with a horizontal-axis revolute
link_revoluteE = chrono.ChLinkLockRevolute()               # truss<->pulley E hinge
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleY(m.pi / 2)))  # axis along Z
sys.AddLink(link_revoluteE)                                # add pulley E hinge

# synchro-belt pulley constraint between gear D and pulley E
link_pulleyDE = chrono.ChLinkLockPulley()                  # belt D-E
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())  # connect D and E
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # D shaft axis
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # E shaft axis
link_pulleyDE.SetRadius1(radD)                            # radius of pulley on D
link_pulleyDE.SetRadius2(radE)                            # radius of pulley on E
link_pulleyDE.SetEnforcePhase(True)                      # synchro belt — no slip
sys.AddLink(link_pulleyDE)                               # add belt D-E

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                # create the Irrlicht window
vis.AttachSystem(sys)                                   # bind the physics system
vis.SetWindowSize(1280, 720)                            # window resolution
vis.SetWindowTitle("Gears and pulleys")                # window title
vis.Initialize()                                        # create device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # standard logo
vis.AddSkyBox()                                         # sky background
vis.AddCamera(chrono.ChVector3d(12, 15, -20))          # camera eye
vis.AddTypicalLights()                                 # standard lighting

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)  # stable integrator for gear constraints

time_step = 1e-3                                        # integration step
sim_end = 10.0                                          # simulation duration
render_fps = 50.0                                       # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                  # begin frame
    vis.Render()                                      # draw scene
    chronoirr.drawSegment(vis, link_pulleyDE.GetBeltUpPos1(), link_pulleyDE.GetBeltUpPos2())        # belt top
    chronoirr.drawSegment(vis, link_pulleyDE.GetBeltBottomPos1(), link_pulleyDE.GetBeltBottomPos2())  # belt bottom
    vis.EndScene()                                    # end frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                # advance one physics step
        if sys.GetChTime() >= sim_end:
            break
