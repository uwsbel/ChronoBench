import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies with specific dimensions (MODIFIED)
radA = 1.5  # Modified radius for first gear
radB = 3.5  # Modified radius for second gear

# Create the truss with modified dimensions (MODIFIED)
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,  # Modified dimensions: 15x8x2
                                   1000,
                                   True,
                                   False,
                                   mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                                   1000,
                                   True,
                                   False,
                                   mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create revolute joint between truss and rotating bar (CORRECTED ChFrameD)
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                           chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                        radA, 0.5,
                                        1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Modified visual shaft dimensions (MODIFIED)
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)  # Modified: 0.3*radA, height 10
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVector3d(0, 3.5, 0),
                                                         chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Modified rotation speed (MODIFIED)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                      chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))  # Modified speed: 3 rad/s
sys.AddLink(link_motor)

# Create second gear with corrected radius calculation (CORRECTED)
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                        radB, 0.4,
                                        1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))  # Modified Z-position: -2
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix second gear to rotating bar (CORRECTED ChFrameD)
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFrameD(chrono.ChVector3d(interaxis12, 0, 0),
                                         chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint between A and B (CORRECTED frames and radius calculation)
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())  # Corrected frame
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint between B and C with corrected radius calculation (CORRECTED)
radC = radB + 2 * radA  # Corrected formula (original had 2*radB + radA)
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())  # Corrected frame
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)