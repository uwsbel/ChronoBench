import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the physics system
physics_system = chrono.ChSystemSMC()

# Initialize the Irrlicht visualization system
visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle("Crank-Slider Mechanism")
visualization.Initialize()
visualization.AddLogo(chronoirr.GetChronoDataFile('logo.png'))
visualization.AddSkyBox()
visualization.AddCamera(chronoirr.vector3df(0, 1.5, -3), chronoirr.vector3df(0, 0.5, 0))
visualization.AddTypicalLights()

# Create the floor (truss)
floor_material = chrono.ChMaterialSurfaceSMC()
floor_material.SetFriction(0.3)
floor_body = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, floor_material)
floor_body.SetPos(chrono.ChVectorD(0, -0.05, 0))
physics_system.Add(floor_body)

# Create the crankshaft
crankshaft_material = chrono.ChMaterialSurfaceSMC()
crankshaft_material.SetFriction(0.3)
crankshaft_body = chrono.ChBodyEasyCylinder(chrono.ChAxis.Y, 0.05, 0.1, 1000, True, True, crankshaft_material)
crankshaft_body.SetPos(chrono.ChVectorD(0, 0.05, 0))
physics_system.Add(crankshaft_body)

# Create the connecting rod
connecting_rod_material = chrono.ChMaterialSurfaceSMC()
connecting_rod_material.SetFriction(0.3)
connecting_rod_body = chrono.ChBodyEasyCylinder(chrono.ChAxis.Y, 0.02, 0.2, 1000, True, True, connecting_rod_material)
connecting_rod_body.SetPos(chrono.ChVectorD(0, 0.15, 0))
physics_system.Add(connecting_rod_body)

# Create the piston
piston_material = chrono.ChMaterialSurfaceSMC()
piston_material.SetFriction(0.3)
piston_body = chrono.ChBodyEasyCylinder(chrono.ChAxis.Y, 0.05, 0.1, 1000, True, True, piston_material)
piston_body.SetPos(chrono.ChVectorD(0, 0.35, 0))
physics_system.Add(piston_body)

# Create joints
crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft_body, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.05, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
physics_system.Add(crank_joint)

connecting_rod_joint_1 = chrono.ChLinkLockRevolute()
connecting_rod_joint_1.Initialize(crankshaft_body, connecting_rod_body, chrono.ChCoordsysD(chrono.ChVectorD(0.05, 0.05, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
physics_system.Add(connecting_rod_joint_1)

connecting_rod_joint_2 = chrono.ChLinkLockRevolute()
connecting_rod_joint_2.Initialize(connecting_rod_body, piston_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
physics_system.Add(connecting_rod_joint_2)

# Add a motor to spin the crankshaft
motor = chrono.ChFunction_Const(chrono.CH_C_PI)
crank_joint.SetMotion_function(motor)

# Set visualization assets
floor_body.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))
crankshaft_body.AddAsset(chrono.ChColorAsset(0.8, 0.8, 0.8))
connecting_rod_body.AddAsset(chrono.ChColorAsset(0.8, 0.8, 0.8))
piston_body.AddAsset(chrono.ChColorAsset(0.8, 0.8, 0.8))

# Simulation loop
while visualization.GetDevice().run():
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()

    physics_system.DoStepDynamics(0.01)
    visualization.Sync()