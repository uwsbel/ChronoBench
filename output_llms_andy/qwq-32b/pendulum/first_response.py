import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
my_system = chrono.ChSystemNSC()
my_system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Create ground body (pivot point)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChSphereShape()
ground_shape.GetSphereGeometry().rad = 0.1
ground.AddAsset(ground_shape)
ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.3, 0.3, 0.3)))
my_system.Add(ground)

# Create pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum_shape = chrono.ChSphereShape()
pendulum_shape.GetSphereGeometry().rad = 0.2
pendulum.AddAsset(pendulum_shape)
pendulum.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.2, 0.2)))
pendulum.SetPos(chrono.ChVectorD(0, -1, 0))  # Start 1m below pivot along Y-axis
my_system.Add(pendulum)

# Create revolute joint with rotation axis along X (swing in Y-Z plane)
rotation = chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))  # Rotate around Y to align axis with X
revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), rotation))
my_system.AddLink(revolute)

# Visualization setup
vis = chronoirr.ChIrrApp(my_system, 'Pendulum Simulation', chronoirr.dimension2du(800, 600))
vis.SetCameraPosition(chrono.ChVectorD(3, 3, 3))  # Camera position
vis.SetCameraVertical(chrono.ChVectorD(0, 1, 0))  # Y-up orientation
vis.AddTypicalLights()
vis.AddTypicalLogo()
vis.AssetBind()
vis.AssetUpdate()
vis.SetTimestep(0.005)  # Time step of 5ms

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation step
    my_system.DoStepDynamics()

    # Log position and velocity
    current_time = my_system.GetChTime()
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Time: {current_time:.2f}, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")