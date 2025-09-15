import pychrono as chrono
import pychrono.irrlicht as chronoirr
from pychrono import ChCoordsysD, ChVectorD, Q_from_AngZ

# 1. Initialize Chrono system with gravity
system = chrono.ChSystemNSC()
system.Set_GravitationalAcceleration(ChVectorD(0, -9.81, 0))

# 2. Create rigid terrain
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(10, 0.1, 10)  # Half dimensions
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)

# Add visualization assets
ground_vis = chrono.ChBoxShape()
ground_vis.GetBoxGeometry().Size = ChVectorD(10, 0.1, 10)
ground.AddAsset(ground_vis)
system.Add(ground)

# 3. Initialize Turtlebot at specified position/orientation
init_pos = ChVectorD(0, 0.2, 0)  # Start slightly above ground
init_rot = Q_from_AngZ(0)        # Initial orientation (Z-up)
turtlebot = chrono.ChTurtleBot(system)
turtlebot.Initialize(ChCoordsysD(init_pos, init_rot))

# 4. Set up Irrlicht visualization
app = chronoirr.ChIrrApp(system, "Turtlebot Simulation", 800, 600)
app.AddTypicalCamera(chronoirr.vector3df(0, 2, -3))  # Camera position
app.AddTypicalLights()
app.AddShadowAll()
app.AssetBindAll()
app.AssetUpdateAll()

# Get driver interface and set time step
driver = turtlebot.GetDriver()
time_step = 0.01

# Simulation loop
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    current_time = system.GetChTime()
    
    # Motor control logic
    if current_time >= 2.0 and current_time < 4.0:
        driver.SetSpeeds(5, 3)  # Turn left
    elif current_time >= 4.0:
        driver.SetSpeeds(5, 5)  # Straight forward
    
    # Update robot and simulation
    turtlebot.Update()
    system.DoStepDynamics(time_step)