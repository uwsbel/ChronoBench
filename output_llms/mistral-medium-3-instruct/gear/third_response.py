import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies with specific dimensions
radA = 1.5
radB = 3.5

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create a revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                          chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                                       chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                     chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Create the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix second gear to the rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                        chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Create the gear constraint between the two gears, A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Create the gear constraint between second gear B and a large wheel C with inner teeth
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# ====================== NEW ADDITIONS ======================

# 1. Add Bevel Gear (Gear D)
radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))  # Rotate 90 degrees around Z-axis

# Create revolute joint for gear D
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                         chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QUNIT))
sys.AddLink(link_revoluteD)

# Create gear constraint between gear A and gear D (1:1 ratio)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(m.pi / 2)))
link_gearAD.SetTransmissionRatio(1.0)  # 1:1 ratio
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)

# Add visual material to gear D
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# 2. Add Pulley (Pulley E)
radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.3, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))  # Rotate 90 degrees around Z-axis

# Create revolute joint for pulley E
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                         chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QUNIT))
sys.AddLink(link_revoluteE)

# Create synchro belt constraint between gear D and pulley E
link_beltDE = chrono.ChLinkLockGear()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_beltDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(m.pi / 2)))
link_beltDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(m.pi / 2)))
link_beltDE.SetTransmissionRatio(radD / radE)  # Ratio based on radii
link_beltDE.SetEnforcePhase(True)
sys.AddLink(link_beltDE)

# Add visual material to pulley E
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# ====================== VISUALIZATION ======================

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Add visual representation of the belt between gear D and pulley E
belt_mat = chrono.ChVisualMaterial()
belt_mat.SetKdColor(chrono.ChColor(0.2, 0.2, 0.2))  # Dark gray color for the belt

# Create a simplified belt visualization (two lines representing the belt)
belt_shape1 = chrono.ChVisualShapeLine()
belt_shape1.SetLineGeometry(chrono.ChLineSegment(chrono.ChVector3d(-10, 0, -9 - radD),
                                               chrono.ChVector3d(-10, -11, -9 - radE)))
belt_shape1.SetMaterial(0, belt_mat)
vis.AddVisualShape(belt_shape1)

belt_shape2 = chrono.ChVisualShapeLine()
belt_shape2.SetLineGeometry(chrono.ChLineSegment(chrono.ChVector3d(-10, 0, -9 + radD),
                                               chrono.ChVector3d(-10, -11, -9 + radE)))
belt_shape2.SetMaterial(0, belt_mat)
vis.AddVisualShape(belt_shape2)

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)