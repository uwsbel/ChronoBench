import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as chronoveh

# --- Simulation setup ---
chrono.SetChronoDataPath("../data/")  # Adjust path as needed

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# --- Terrain ---
# Load the collision and visual meshes for the terrain
terrain_col_mesh = chrono.ChTriangleMeshConnected()
terrain_col_mesh.LoadFromFile("Highway_col.obj")
terrain_col_mesh.SetMutable(True)  # Allow modifications for collision

terrain_vis_mesh = chrono.ChTriangleMeshConnected()
terrain_vis_mesh.LoadFromFile("Highway_vis.obj")
terrain_vis_mesh.SetMutable(True)

# Create a terrain object
terrain = chrono.ChTerrain()
terrain.SetMesh(terrain_col_mesh)
terrain.SetVisualMesh(terrain_vis_mesh)
terrain.SetContactMethod(chrono.ChTerrain.ContactMethod.HYBRID)
system.Add(terrain)

# --- Vehicle ---
# Create the HMMWV vehicle
hmmwv = chronoveh.HMMWV()

# Set initial vehicle position and orientation
hmmwv.SetPos(chrono.ChVectorD(0, 1, 0))
hmmwv.SetTransform(chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), chrono.Q_from_Ang3(0, 0, 0)))

# Set the contact method for the vehicle
hmmwv.SetContactMethod(chrono.ChVehicle.ContactMethod.HYBRID)

# Set the tire model
hmmwv.SetTireModel(chronoveh.HMMWV.TireModel.TMEASY)

# Enable mesh visualization for all vehicle components
for wheel in hmmwv.GetWheels():
    wheel.SetCollide(True)
    wheel.SetVisualizationType(chrono.ChVehicle.VisualizationType.MESH)
    wheel.GetVisualShape().SetMeshFile("wheel.obj")  # Replace with actual wheel mesh file

hmmwv.SetCollide(True)
hmmwv.SetVisualizationType(chrono.ChVehicle.VisualizationType.MESH)
hmmwv.GetVisualShape().SetMeshFile("hmmwv.obj") # Replace with actual HMMWV mesh file

system.Add(hmmwv)

# --- Driver system ---
# Create an interactive driver system
driver = chronoveh.ChIrrAppDriver(hmmwv)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)

# --- Visualization ---
# Create an Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 10, -15))
vis.AddTypicalLights()

# --- Simulation loop ---
# Set the simulation step size and target frame rate
time_step = 0.005
target_fps = 50

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the simulation
    system.DoStepDynamics(time_step)

    # Update the driver input
    driver.Synchronize(time_step)

    # Limit frame rate
    chrono.ChThread::Sleep(1.0 / target_fps)