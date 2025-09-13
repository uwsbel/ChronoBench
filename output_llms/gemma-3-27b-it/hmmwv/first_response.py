import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# --- Create the simulation system ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# --- Create the terrain ---
terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
terrain.SetMass(0)
terrain.SetCollide(True)
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.1)

# Terrain dimensions
terrain_size_x = 100
terrain_size_z = 100
terrain.SetBox(terrain_size_x, 1, terrain_size_z)
terrain.SetPos(chrono.ChVectorD(0, -0.1, 0))

# Add terrain to system
system.Add(terrain)

# --- Create the HMMWV vehicle ---
hmmwv = veh.HMMWV()

# Set initial vehicle position and orientation
initial_x = 0
initial_z = 0
initial_yaw = 0  # Radians
hmmwv.SetPos(chrono.ChVectorD(initial_x, 0.5, initial_z))
hmmwv.SetRotation(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), initial_yaw))

# Set contact method
hmmwv.SetContactMethod(veh.ChContactMethod.SMC)

# Set tire model
hmmwv.SetTireModel(veh.ChTireModel.TMEASY)

# Use primitive visualization
hmmwv.SetChassisVisualizationType(veh.ChVisualizationType.MESH)
hmmwv.SetWheelVisualizationType(veh.ChVisualizationType.MESH)
hmmwv.SetSuspensionVisualizationType(veh.ChVisualizationType.MESH)

# Add vehicle to system
system.Add(hmmwv)

# --- Create the interactive driver ---
driver = veh.ChInteractiveDriver(hmmwv)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)
driver.SetHandBrakeDelta(0.1)

# --- Create the Irrlicht application ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))
vis.AddTypicalLights()

# --- Simulation loop ---
time_step = 0.005
vis.SetUpdateInterval(time_step)
vis.SetRenderMode(chronoirr.ChRenderMode.REGISTERED)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the system
    system.DoStepDynamics(time_step)

    # Update the driver
    driver.Synchronize(time_step)