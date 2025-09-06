import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Modify gear sizes
radA = 1.5
radB = 3.5

# Create the truss with modified dimensions
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,  # Modified dimensions: 15x8x2
                                   1000,
                                   True,
                                   False,
                                   mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))

# Shared visualization material
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create rotating bar
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                                   1000,
                                   True,
                                   False,
                                   mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))

# Revolute joint between truss and rotating bar (fix ChFramed to ChFrameD)
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                          chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create first gear (radA = 1.5)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5,
                                        1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.VECT_X))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Modified visual shaft (radius scaled by 0.3, height 10)
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape,
                          chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                                         chrono.Q_from_AngAxis(m.pi/2, chrono.VECT_X)))

# Motor link with modified speed (3 rad/s)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.AddLink(link_motor)

# Calculate new interaxis distance
interaxis12 = radA + radB

# Create second gear (radB = 3.5) with modified position
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4,
                                        1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))  # Modified Z position
mbody_gearB.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.VECT_X))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gearB to rotating bar (fix ChFramed)
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                        chrono.ChFrameD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint between A and B (fix ChFramed)
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngAxis(-m.pi/2, chrono.VECT_X)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngAxis(-m.pi/2, chrono.VECT_X)))
link_gearAB.SetTransmissionRatio(radA/radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint between B and truss (radC calculation)
radC = 2 * radB + radA  # 2*3.5 +1.5 =8.5
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngAxis(-m.pi/2, chrono.VECT_X)))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,-4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB/radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Gears Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(12, 15, -20))
vis.AddTypicalLights()

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)