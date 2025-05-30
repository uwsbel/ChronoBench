import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ----------------------------
# Initialize the Chrono system
# ----------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# ----------------------------
# Create the floor (truss)
# ----------------------------
floor = chrono.ChBody()
floor.SetBodyFixed(True)
system.Add(floor)

# Floor shape and texture
floor_shape = chrono.ChBoxShape()
floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 0.5, 1)
floor.GetAssets().push_back(floor_shape)

texture_floor = chrono.ChTexture()
texture_floor.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
floor.GetAssets().push_back(texture_floor)

# ----------------------------
# Create the crankshaft
# ----------------------------
crank = chrono.ChBody()
crank.SetPos(chrono.ChVectorD(0, 0, 0))
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(crank)

# Crank shape and texture
cylinder_crank = chrono.ChCylinderShape()
cylinder_crank.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.5, 0, 0)
cylinder_crank.GetCylinderGeometry().p2 = chrono.ChVectorD(0.5, 0, 0)
cylinder_crank.GetCylinderGeometry().rad = 0.1
crank.GetAssets().push_back(cylinder_crank)

texture_crank = chrono.ChTexture()
texture_crank.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/blue.png')
crank.GetAssets().push_back(texture_crank)

# ----------------------------
# Create the connecting rod
# ----------------------------
rod = chrono.ChBody()
rod.SetPos(chrono.ChVectorD(1.0, 0, 0))  # Centered between crank and piston
rod.SetMass(1.0)
rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(rod)

# Rod shape and texture
cylinder_rod = chrono.ChCylinderShape()
cylinder_rod.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.5, 0, 0)
cylinder_rod.GetCylinderGeometry().p2 = chrono.ChVectorD(0.5, 0, 0)
cylinder_rod.GetCylinderGeometry().rad = 0.05
rod.GetAssets().push_back(cylinder_rod)

texture_rod = chrono.ChTexture()
texture_rod.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/red.png')
rod.GetAssets().push_back(texture_rod)

# ----------------------------
# Create the piston
# ----------------------------
piston = chrono.ChBody()
piston.SetPos(chrono.ChVectorD(1.5, 0, 0))
piston.SetMass(2.0)
piston.SetInertiaXX(chrono.ChVectorD(0.2, 0.2, 0.2))
system.Add(piston)

# Piston shape and texture
box_piston = chrono.ChBoxShape()
box_piston.GetBoxGeometry().Size = chrono.ChVectorD(0.25, 0.2, 0.2)
piston.GetAssets().push_back(box_piston)

texture_piston = chrono.ChTexture()
texture_piston.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/green.png')
piston.GetAssets().push_back(texture_piston)

# ----------------------------
# Create joints
# ----------------------------

# Revolute joint between floor and crank
revolute_floor_crank = chrono.ChLinkLockRevolute()
revolute_floor_crank.Initialize(floor, crank, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_floor_crank)

# Revolute joint between crank and rod
revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_crank_rod)

# Revolute joint between rod and piston
revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(1.5, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_rod_piston)

# Prismatic joint between piston and floor (along x-axis)
prismatic_piston_floor = chrono.ChLinkLockPrismatic()
prismatic_piston_floor.Initialize(floor, piston, chrono.ChCoordsysD(chrono.ChVectorD(1.5, 0, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.AddLink(prismatic_piston_floor)

# ----------------------------
# Create motor to drive crank
# ----------------------------
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(revolute_floor_crank, chrono.ChFrameD(), chrono.ChFrameD())
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_C_PI))  # 180 degrees per second
system.AddLink(motor)

# ----------------------------
# Setup Irrlicht visualization
# ----------------------------
application = chronoirr.ChIrrApp(system, 'Crank-Slider Mechanism', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 3, -6), chronoirr.vector3df(0, 1, 0))

application.AssetBindAll()
application.AssetUpdateAll()

# ----------------------------
# Simulation loop
# ----------------------------
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(1e-3)