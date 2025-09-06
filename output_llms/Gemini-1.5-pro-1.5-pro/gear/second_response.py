import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies with specific dimensions
radA = 1.5  # Modified: Changed radius for first gear
radB = 3.5  # Modified: Changed radius for second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)  # Modified: Changed truss dimensions
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))  # Correct position

# Create a revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0))) # Use ChCoordsysD
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2)) # Use Q_from_AngX

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)  # Modified: Changed visual shaft size
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVector3D(0, 3.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2))) # Use ChFrameD and Q_from_AngX


mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Impose rotation speed on the first gear
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0))) # Use ChCoordsysD
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))  # Modified: Changed rotation speed
sys.AddLink(link_motor)


# Create the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))  # Modified: Changed position of Gear B
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2)) #Use Q_from_AngX
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix second gear to the rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train, chrono.ChCoordsysD(chrono.ChVector3D(interaxis12, 0, 0))) #Use ChCoordsysD and ChVector3D
sys.AddLink(link_revolute)

# Create the gear constraint between the two gears, A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD()) # Use ChFrameD
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2))) # Use ChFrameD and Q_from_AngX
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2))) # Use ChFrameD and Q_from_AngX

link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)  # Correct method name
sys.AddLink(link_gearAB)

# Create the gear constraint between second gear B and a large wheel C (truss)
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD()) # Use ChFrameD
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2))) # Use ChFrameD and Q_from_AngX
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3D(0, 0, -4))) # Use ChFrameD and ChVector3D
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True) # Correct Method Name
sys.AddLink(link_gearBC)


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


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) # Correct enum


# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)