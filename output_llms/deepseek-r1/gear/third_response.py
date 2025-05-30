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

# Create the truss with corrected collision setting
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, True, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create rotating bar support
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Revolute joint between truss and rotating bar (corrected frame orientation)
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, 
                          chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create first gear with corrected visualization
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Visualization shaft (corrected frame orientation)
mshaft_shape = chrono.ChVisualShapeCylinder(radA*0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, 
                          chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), 
                                         chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Motor constraint with corrected parent/child order
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_truss, mbody_gearA, 
                     chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Create second gear with corrected position
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gear B (corrected frame position)
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_train, mbody_gearB,
                        chrono.ChFramed(chrono.ChVector3d(interaxis12,0,0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint AB with corrected parameters
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB,
                      chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QuatFromAngleX(-m.pi/2)),
                      chrono.ChFramed(chrono.ChVector3d(interaxis12,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetTransmissionRatio(-radA/radB)
sys.AddLink(link_gearAB)

# Create bevel gear D
radD = 5.0
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.6, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi/2))  # Rotate 90° around Z-axis
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gear D (horizontal X-axis)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_truss, mbody_gearD,
                         chrono.ChFramed(chrono.ChVector3d(-10,0,-9), chrono.QuatFromAngleY(m.pi/2)))
sys.AddLink(link_revoluteD)

# Gear constraint AD (1:1 ratio)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD,
                      chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QuatFromAngleX(-m.pi/2)),
                      chrono.ChFramed(chrono.ChVector3d(-10,0,-9), chrono.QuatFromAngleY(-m.pi/2)))
link_gearAD.SetTransmissionRatio(1.0)
sys.AddLink(link_gearAD)

# Create pulley E
radE = 2.0
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.4, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi/2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for pulley E
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_truss, mbody_pulleyE,
                         chrono.ChFramed(chrono.ChVector3d(-10,-11,-9), chrono.QuatFromAngleY(m.pi/2)))
sys.AddLink(link_revoluteE)

# Pulley constraint DE
link_pulley = chrono.ChLinkPulley()
link_pulley.Initialize(mbody_gearD, mbody_pulleyE,
                      chrono.ChVector3d(-10,0,-9), chrono.ChVector3d(-10,-11,-9),
                      radD, radE)
sys.AddLink(link_pulley)

# Belt visualization
belt_line = chrono.ChVisualShapeLine()
belt_line.SetLineGeometry(chrono.ChLineSegment(
    chrono.ChVector3d(-10 + radD, 0, -9),
    chrono.ChVector3d(-10 - radE, -11, -9)
))
mbody_truss.AddVisualShape(belt_line)

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Enhanced Gear System')
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