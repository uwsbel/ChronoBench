import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies.

radA = 1.5  # Modified radius for the first gear
radB = 3.5  # Modified radius for the second gear

# ...the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)  # Modified truss dimensions
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# ...the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# ...which must rotate respect to truss along Z axis, in 0,0,0
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# ...the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# for aesthetic reasons, also add a thin cylinder only as a visualization
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)  # Modified visual shaft size
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# ...impose rotation speed between the first gear and the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))  # Modified gear rotation speed
sys.AddLink(link_motor)

# ...the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))  # Modified position of Gear B
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...the second gear is fixed to the rotating bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                        chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# ...the gear constraint between the two wheels A and B.
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# ...the gear constraint between the second wheel B and a large wheel C with inner teeth.
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# ...the bevel gear at the side.
radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...it is fixed to the truss using a revolute joint with horizontal axis.
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteD)

# ... Let's make a 1:1 gear between wheel A and wheel D as a bevel gear.
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)
sys.AddLink(link_gearAD)

# ...the pulley at the side.
radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# ... it is fixed to the truss using a revolute joint with horizontal axis.
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteE)

# ... Let's make a synchro belt constraint between pulley D and pulley E.
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetRadius1(radD)
link_pulleyDE.SetRadius2(radE)
link_pulleyDE.SetEnforcePhase(True)
sys.AddLink(link_pulleyDE)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    # Draw some segments for a simplified representation of pulley
    chronoirr.drawSegment(vis,
                          link_pulleyDE.GetBeltUpPos1(),
                          link_pulleyDE.GetBeltUpPos2())
    chronoirr.drawSegment(vis,
                          link_pulleyDE.GetBeltBottomPos1(),
                          link_pulleyDE.GetBeltBottomPos2())
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
