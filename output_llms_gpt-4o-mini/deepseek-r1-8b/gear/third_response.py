import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Corrected: Added missing variable definitions and error corrections
# Define gear radii and distances
radA = 1.5
radB = 3.5
interaxisAB = radA + radB  # Distance between centers of gears A and B
radC = 2 * radB + radA    # Radius for large wheel C
radD = 5                   # Radius for bevel gear D
radE = 2                   # Radius for pulley E

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,
                                  1000,
                                  True,
                                  False,
                                  mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Create the rotating bar support
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                                  1000,
                                  True,
                                  False,
                                  mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                         radA, 0.5,
                                         1000,
                                         True,
                                         False,
                                         mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add visual cylinder for the gear shaft
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape,
                        chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                      chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Create motor link for gear A
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Create the second gear
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                         radB, 0.4,
                                         1000,
                                         True,
                                         False,
                                         mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxisAB, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Create revolute joint between gear B and rotating bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                        chrono.ChFramed(chrono.ChVector3d(interaxisAB, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Create gear constraint between gears A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB,
                      chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Create gear constraint between gear B and truss (large wheel C)
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss,
                      chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Create bevel gear D
mbody_gearD = chrono.ChBodyEasyBevel(chrono.ChAxis_Y,
                                    radD, 0.5,
                                    1000,
                                    True,
                                    False,
                                    mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))  # 90 degrees rotation
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add visual shape for bevel gear
mshaft_shapeD = chrono.ChVisualShapeBevel(radD * 0.3, 10)
mbody_gearD.AddVisualShape(mshaft_shapeD,
                        chrono.ChFramed(chrono.ChVector3d(-10, 3, 0),
                                      chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Create pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                         radE, 0.5,
                                         1000,
                                         True,
                                         False,
                                         mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add visual shape for pulley
mshaft_shapeE = chrono.ChVisualShapeCylinder(radE * 0.3, 10)
mbody_pulleyE.AddVisualShape(mshaft_shapeE,
                        chrono.ChFramed(chrono.ChVector3d(-10, -11 + 5, 0),
                                      chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Create synchro belt constraint between gear D and pulley E
link_gearDE = chrono.ChLinkLockGear()
link_gearDE.Initialize(mbody_gearD, mbody_pulleyE,
                      chrono.ChFramed())
link_gearDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearDE.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearDE.SetTransmissionRatio(radD / radE)
link_gearDE.SetEpicyclic(True)
sys.AddLink(link_gearDE)

# Visualization enhancements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys with Bevel and Pulley')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Add simplified belt visual
belt_shape = chrono.ChVisualShapeCylinder(1, 10)
belt_frame = chrono.ChFramed(chrono.ChVector3d(-10, -5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2))
mbody_gearD.AddVisualShape(belt_shape, belt_frame)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)