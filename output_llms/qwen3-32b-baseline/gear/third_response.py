import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies with specific dimensions
radA = 1.5  # Radius for gear A
radB = 3.5  # Radius for gear B
radD = 5.0  # Radius for bevel gear D
radE = 2.0  # Radius for pulley E

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create the first gear (A)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add visual shaft for gear A
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Impose rotation speed on gear A
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Create the second gear (B)
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix gear B to rotating bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train, chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint between A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint between B and truss (large wheel C)
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Add bevel gear D
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))  # Rotate 90° around Z-axis
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gear D
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss, chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QUNIT))
sys.AddLink(link_revoluteD)

# Gear constraint between A and D (1:1 ratio)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, 0, -1), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1.0)
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)

# Add pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.5, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))  # Rotate 90° around Z-axis
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for pulley E
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss, chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QUNIT))
sys.AddLink(link_revoluteE)

# Belt constraint between D and E (synchro belt)
link_gearDE = chrono.ChLinkLockGear()
link_gearDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_gearDE.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearDE.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearDE.SetTransmissionRatio(radE / radD)  # 2/5 ratio for belt drive
sys.AddLink(link_gearDE)

# Add belt visualization (simplified cylinder)
belt_shape = chrono.ChVisualShapeCylinder(0.1, 11.0)  # Radius 0.1, length 11
belt_frame = chrono.ChFramed(chrono.ChVector3d(-10, -5.5, -9), chrono.QUNIT)  # Midpoint between D and E
mbody_truss.AddVisualShape(belt_shape, belt_frame)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and Pulleys with Bevel and Belt')
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
    vis.EndScene()
    sys.DoStepDynamics(1e-3)