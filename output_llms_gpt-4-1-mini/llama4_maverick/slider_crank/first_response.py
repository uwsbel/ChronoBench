import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Create the Chrono system
sys = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -2.5, 0))
sys.Add(ground)

# Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(chrono.ChCoordsysD(chrono.ChVectorD(-2, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)), 0.1, 0.5, 1000)
sys.Add(crankshaft)

# Create the connecting rod
connecting_rod = chrono.ChBodyEasyBox(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 2, 0.1, 0.1, 1000)
connecting_rod.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(connecting_rod)

# Create the piston
piston = chrono.ChBodyEasyCylinder(chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)), 0.1, 0.2, 1000)
sys.Add(piston)

# Revolute joint between ground and crankshaft
rev_ground_crank = chrono.ChLinkRevolute()
rev_ground_crank.Initialize(ground, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(-2, 0, 0)))
sys.Add(rev_ground_crank)

# Revolute joint between crankshaft and connecting rod
rev_crank_conn = chrono.ChLinkRevolute()
rev_crank_conn.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(-1, 0, 0)))
sys.Add(rev_crank_conn)

# Revolute-Prismatic joint (or equivalent) between connecting rod and piston
rev_prism_conn_piston = chrono.ChLinkRevoluteSpherical()
rev_prism_conn_piston.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0)))
sys.Add(rev_prism_conn_piston)

# Prismatic joint between piston and ground
prism_piston_ground = chrono.ChLinkPrismatic()
prism_piston_ground.Initialize(piston, ground, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Y)))
sys.Add(prism_piston_ground)

# Motor to rotate the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, crankshaft, chrono.ChFrameD(chrono.ChVectorD(-2, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  # Rotate at 180 degrees per second
sys.Add(motor)

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-4, 3, -4), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)