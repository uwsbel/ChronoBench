import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# Set the Chrono data path (adjust according to your installation)
chrono.SetChronoDataPath('/path/to/chrono/data')

# Initialize the vehicle system
veh.InitLog()

# Create the CityBus vehicle
contact_method = chrono.ChContactMethod_SMC
vehicle = veh.CityBus(veh.GetDataFile("CityBus/vehicle/CityBus.json"), contact_method)

# Set the initial position and orientation of the vehicle
initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation

# Initialize the vehicle
vehicle.Initialize(chrono.ChCoordsysd(initLoc, initRot))

# Set tire model (optional, adjust according to available models)
tire_model = veh.TireModelType_TMEasy
vehicle.SetTireType(tire_model)

# Get the vehicle system
sys = vehicle.GetSystem()

# Disable rigid body gravity for the terrain (if applicable)
sys.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))

# Create the terrain
terrain = veh.RigidTerrain(sys)

# Define the terrain patch dimensions and material
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrainLength = 100.0
terrainWidth = 100.0

patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)

# Set visualization type for the terrain (MESH or PRIMITIVES)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Initialize the terrain
terrain.Initialize()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 3), vehicle.GetChassis().GetPos())
vis.AddTypicalLights()

# Set visualization types for vehicle parts
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vis)

# Set the initial driver inputs
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(1.0 / 60)
driver.SetBrakingDelta(1.0 / 60)

# Initialize the driver inputs
driver.Initialize()

# Simulation loop parameters
fps = 50
time_step = 1 / fps
time_end = 1000

# Simulation loop
while vis.Run() and sys.GetChTime() < time_end:
    time = sys.GetChTime()

    # Update driver inputs
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)

    # Update the vehicle dynamics
    vehicle.Update(time_step)

    # Update the terrain
    terrain.Update(time)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation
    sys.DoStepDynamics(time_step)