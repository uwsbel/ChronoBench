import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create the simulation system
system = chrono.ChSystemNSC()

# Create the floor (truss)
floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetCollide(True)
floor.GetCollisionModel().ClearModel()
floor.GetCollisionModel().AddBox(10, 1, 10)  # Size of the floor
system.Add(floor)

# Visualization for floor
floor_shape = chrono.ChVisualShape()
floor_shape.SetBox(chrono.ChVectorD(10, 1, 10))
floor_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
floor_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
floor.GetAssets().push_back(floor_shape)

# Create crankshaft
crankshaft = chrono.ChBody()
crankshaft.SetMass(1.0)
crankshaft.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetCollide(True)
crankshaft.GetCollisionModel().ClearModel()
crankshaft.GetCollisionModel().AddCylinder(0.2, 0.1)  # Radius 0.2, length 0.1
system.Add(crankshaft)

# Visualization for crankshaft
crankshaft_shape = chrono.ChVisualShape()
crankshaft_shape.SetCylinder(0.2, 0.1)
crankshaft_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crankshaft_shape.SetTexture(chrono.GetChronoDataFile("textures/copper.jpg"))
crankshaft.GetAssets().push_back(crankshaft_shape)

# Create connecting rod
connecting_rod = chrono.ChBody()
connecting_rod.SetMass(0.5)
connecting_rod.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
connecting_rod.SetPos(chrono.ChVectorD(0.7, 0, 0))  # Centered between crank pin and piston
connecting_rod.SetCollide(True)
connecting_rod.GetCollisionModel().ClearModel()
connecting_rod.GetCollisionModel().AddBox(1.0, 0.05, 0.05)
system.Add(connecting_rod)

# Visualization for connecting rod
rod_shape = chrono.ChVisualShape()
rod_shape.SetBox(chrono.ChVectorD(1.0, 0.05, 0.05))
rod_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
rod_shape.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
connecting_rod.GetAssets().push_back(rod_shape)

# Create piston
piston = chrono.ChBody()
piston.SetMass(0.5)
piston.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
piston.SetPos(chrono.ChVectorD(1.2, 0, 0))
piston.SetCollide(True)
piston.GetCollisionModel().ClearModel()
piston.GetCollisionModel().AddBox(0.2, 0.2, 0.2)
system.Add(piston)

# Visualization for piston
piston_shape = chrono.ChVisualShape()
piston_shape.SetBox(chrono.ChVectorD(0.2, 0.2, 0.2))
piston_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
piston_shape.SetTexture(chrono.GetChronoDataFile("textures/metal.jpg"))
piston.GetAssets().push_back(piston_shape)

# Create joints and motor
# Motor between floor and crankshaft (rotates around Z-axis)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crankshaft, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(2 * chrono.CH_C_PI))  # 1 revolution per second
system.AddLink(motor)

# Revolute joint between crankshaft and connecting rod
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0), chrono.QUNIT))
system.AddLink(joint_crank_rod)

# Revolute joint between connecting rod and piston
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(1.2, 0, 0), chrono.QUNIT))
system.AddLink(joint_rod_piston)

# Prismatic joint between piston and floor (along X-axis)
prismatic_joint = chrono.ChLinkLockPrismatic()
prismatic_joint.Initialize(floor, piston, chrono.ChCoordsysD(piston.GetPos(), chrono.QUNIT))
system.AddLink(prismatic_joint)

# Set up Irrlicht visualization
application = chronoirr.ChIrrApp(system, 'Crank-Slider Mechanism', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(2, 1, -3), chronoirr.vector3df(0, 0, 0))

# Set ambient light and other settings
application.SetAmbientLight(chronoirr.SColorf(1, 1, 1, 1))

# Bind and update assets
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())