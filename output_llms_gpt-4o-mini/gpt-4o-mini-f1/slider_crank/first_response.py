import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Create the Chrono simulation system
sys = chrono.ChSystemNSC()

# Create a floor (truss) body
floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -0.25, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Create a crankshaft body
crankshaft = chrono.ChBodyEasyCylinder(0.1, 2, 1000, True, True)
crankshaft.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(crankshaft)

# Create a connecting rod body
connecting_rod = chrono.ChBodyEasyBox(0.1, 0.3, 1.5, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector3d(0, 0, 0))
connecting_rod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(connecting_rod)

# Create a piston body
piston = chrono.ChBodyEasyCylinder(0.2, 1, 1000, True, True)
piston.SetPos(chrono.ChVector3d(0, 0, 0))
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(piston)

# Create a revolute joint between the crankshaft and the connecting rod
crankshaft_connecting_rod_joint = chrono.ChLinkLockRevolute()
crankshaft_connecting_rod_joint.Initialize(crankshaft, connecting_rod, chrono.ChFramed(crankshaft.GetPos()))
sys.Add(crankshaft_connecting_rod_joint)

# Create a revolute joint between the connecting rod and the piston
connecting_rod_piston_joint = chrono.ChLinkLockRevolute()
connecting_rod_piston_joint.Initialize(connecting_rod, piston, chrono.ChFramed(connecting_rod.GetPos()))
sys.Add(connecting_rod_piston_joint)

# Create a prismatic joint between the piston and the floor
piston_floor_joint = chrono.ChLinkLockPrismatic()
piston_floor_joint.Initialize(piston, floor, chrono.ChFramed(piston.GetPos(), chrono.QUNIT))
sys.Add(piston_floor_joint)

# Create a motor to spin the crankshaft
crankshaft_motor = chrono.ChLinkMotorRotationSpeed()
crankshaft_motor.Initialize(crankshaft, floor, chrono.ChFramed(crankshaft.GetPos()))
crankshaft_motor.SetMotorFunction(chrono.ChFunctionConst(2 * chrono.CH_PI / 3))  # 2π/3 rad/s
sys.Add(crankshaft_motor)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1280, 720))
vis.SetWindowTitle('Crank-slider mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, 4.5, -2.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-2)