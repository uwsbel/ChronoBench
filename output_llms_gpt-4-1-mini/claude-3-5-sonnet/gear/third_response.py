import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChMaterialSurfaceNSC()

# Create all rigid bodies with specific dimensions
radA = 1.5
radB = 3.5
radC = 2 * radB + radA
radD = 5  # New bevel gear radius
radE = 2  # New pulley radius

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetBodyFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))

# Create a revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                                                         chrono.Q_from_AngX(chrono.CH_C_PI_2)))

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.AddLink(link_motor)

# Create the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix second gear to the rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChCoordsysD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Create the gear constraint between the two gears, A and B
link_gearAB = chrono.ChLinkGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.CSYSNORM)
link_gearAB.SetTransmissionRatio(radA / radB)
sys.AddLink(link_gearAB)

# Create the gear constraint between second gear B and a large wheel C with inner teeth
link_gearBC = chrono.ChLinkGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.CSYSNORM)
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Create the new bevel gear D
mbody_gearD = chrono.ChBodyEasyCylinder(radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Create a revolute joint for gear D
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChCoordsysD(chrono.ChVectorD(-10, 0, -9), chrono.Q_from_AngZ(m.pi / 2)))
sys.AddLink(link_revoluteD)

# Create a gear constraint between gear A and gear D
link_gearAD = chrono.ChLinkGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.CSYSNORM)
link_gearAD.SetTransmissionRatio(1)  # 1:1 gear ratio
sys.AddLink(link_gearAD)

# Create the new pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(radE, 0.3, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Create a revolute joint for pulley E
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChCoordsysD(chrono.ChVectorD(-10, -11, -9), chrono.Q_from_AngZ(m.pi / 2)))
sys.AddLink(link_revoluteE)

# Create a synchro belt constraint between gear D and pulley E
link_beltDE = chrono.ChLinkPulley()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, 
                       chrono.ChVectorD(-10, 0, -9), chrono.ChVectorD(-10, -11, -9))
link_beltDE.SetPulleyRadii(radD, radE)
sys.AddLink(link_beltDE)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(12, 15, -20))
vis.AddTypicalLights()

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    # Add simplified visual representation of the belt
    start = mbody_gearD.GetPos()
    end = mbody_pulleyE.GetPos()
    vis.DrawLine(start, end, chrono.ChColor(0.7, 0.7, 0.7))
    
    vis.EndScene()
    sys.DoStepDynamics(1e-3)