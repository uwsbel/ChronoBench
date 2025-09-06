import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono environment
chrono.SetChronoDataPath('../../data/')

# Create a Chrono::Engine physical system
system = chrono.ChSystemNSC()

# Create the floor (truss)
floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
floor.SetBodyFixed(True)
system.Add(floor)

# Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0.25, 0))
system.Add(crankshaft)

# Create the connecting rod
connecting_rod = chrono.ChBodyEasyBox(0.1, 0.1, 1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
system.Add(connecting_rod)

# Create the piston
piston = chrono.ChBodyEasyCylinder(0.1, 0.2, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1, 0.25, 0))
system.Add(piston)

# Create revolute joint between floor and crankshaft
crank_joint = chrono.ChLinkRevolute()
crank_joint.Initialize(floor, crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0.25, 0)))
system.Add(crank_joint)

# Create revolute joint between crankshaft and connecting rod
rod_crank_joint = chrono.ChLinkRevolute()
rod_crank_joint.Initialize(crankshaft, connecting_rod, chrono.ChFrameD(chrono.ChVectorD(0.25, 0.25, 0)))
system.Add(rod_crank_joint)

# Create prismatic joint between connecting rod and piston
piston_joint = chrono.ChLinkPrismatic()
piston_joint.Initialize(connecting_rod, piston, chrono.ChFrameD(chrono.ChVectorD(0.75, 0.25, 0)))
system.Add(piston_joint)

# Apply a motor to the crankshaft
motor = chrono.ChFunction_Const(chrono.CH_C_PI / 4)  # Constant angular speed
driver = chrono.ChDriverCrank()
driver.Initialize(crank_joint, motor)
system.Add(driver)

# Set up the Irrlicht visualization
application = chronoirr.ChIrrApp(system, 'Crank-Slider Mechanism', chrono.ChVectorD(0.2, 0.2, 0.2))

application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_chrono_alpha.png')
application.AddTypicalFloor(chrono.ChCoordsysD(chrono.ChVectorD(0, -0.05, 0)))

application.AssetBindAll()
application.AssetUpdateAll()

# Set custom camera settings
application.SetCameraTarget(chrono.ChVectorD(0, 0.25, 0))
application.SetCameraDistance(2)
application.SetCameraAngles(30, 30)

# Run the simulation
application.SetTimestep(0.01)
application.SetTryRealTime(True)

# Run the application
application.Run()