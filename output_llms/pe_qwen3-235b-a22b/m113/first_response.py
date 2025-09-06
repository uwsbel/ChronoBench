import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set Chrono data path (adjust based on your installation)
chrono.SetChronoDataPath("C:/Chrono/Data/")

# Create the physical system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create rigid terrain with custom material
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)      # Set friction coefficient
patch_mat.SetRestitution(0.01)  # Set restitution coefficient
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         200, 100)  # Terrain dimensions (length, width)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()

# Initialize M113 vehicle
m113 = veh.M113(sys)  # Create M113 vehicle
m113.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 1, 0),  # Initial position (x, y, z)
    chrono.QuatFromAngleZ(0)     # Initial orientation (yaw)
))
m113.Initialize()
# Set visualization types for vehicle components
m113.SetChassisVisualizationType(veh.VisualizationType_MESH)
m113.SetSprocketVisualizationType(veh.VisualizationType_MESH)
m113.SetIdlerVisualizationType(veh.VisualizationType_MESH)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)

# Create driver system
driver = veh.ChIrrGuiDriver(m113)  # Interactive Irrlicht GUI driver
driver.Initialize()                # Initialize driver inputs

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -6))  # Camera position
vis.AddTypicalLights()

# Simulation loop parameters
time_step = 0.01  # Simulation time step (seconds)

# Run simulation
while vis.Run():
    time = sys.GetChTime()  # Current simulation time
    
    # Synchronize components
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver.GetInputs(), terrain)
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()