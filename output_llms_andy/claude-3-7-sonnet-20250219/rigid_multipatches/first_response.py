import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math
import os

# ---------------------------------------------------------------------
# Initialize the PyChrono system and simulation

print("Create the simulation system and solver")
chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))

# Create a ChronoEngine system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create a rigid terrain with multiple patches

print("Creating the terrain...")

terrain = veh.RigidTerrain(my_system)

# Create a flat patch with default material
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)

# Add flat ground patch
terrain_params = veh.RigidTerrainToMeshDesc()
terrain_params.m_mesh_filename = "terrain/RigidPlane.obj"
terrain_params.m_mesh_scale = chrono.ChVectorD(20, 1, 20)
terrain_params.m_friction = 0.8
terrain_params.m_color = chrono.ChColor(0.5, 0.8, 0.5)
terrain.AddPatch(terrain_params)

# Add a bump using mesh patch
bump_params = veh.RigidTerrainToMeshDesc()
bump_params.m_mesh_filename = "terrain/bump.obj"
bump_params.m_mesh_scale = chrono.ChVectorD(10, 0.5, 10)
bump_params.m_mesh_position = chrono.ChVectorD(0, 0, 20)
bump_params.m_friction = 0.9
bump_params.m_color = chrono.ChColor(0.5, 0.5, 0.8)
terrain.AddPatch(bump_params)

# Add a heightmap-based patch for varying elevation
height_params = veh.RigidTerrainToHeightMapDesc()
height_params.m_height_map_size = chrono.ChVectorD(50, 2, 50)
height_params.m_height_map_position = chrono.ChVectorD(50, -1, 0)
height_params.m_height_map_data_file = "terrain/heightmap.dat"
height_params.m_friction = 0.9
height_params.m_color = chrono.ChColor(0.8, 0.8, 0.5)
terrain.AddPatch(height_params)

terrain.Initialize()

# ---------------------------------------------------------------------
# Create the HMMWV vehicle model

print("Creating the HMMWV vehicle model...")

# Set vehicle parameters
vehicle_params = veh.HMMWV_VehicleParams()
powertrain_params = veh.HMMWV_PowertrainParams()
tire_params = veh.HMMWV_TireParams()

# Initial vehicle position and orientation
initPos = chrono.ChVectorD(0, 0.6, 0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Create the vehicle
hmmwv = veh.HMMWV_Full(my_system)
hmmwv.SetInitPosition(chrono.ChCoordsysD(initPos, initRot))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()

# Set visualization properties
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# ---------------------------------------------------------------------
# Create the driver system

print("Creating the driver system...")

# Create the interactive driver
driver = veh.ChIrrGuiDriver(hmmwv.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# ---------------------------------------------------------------------
# Create the Irrlicht visualization

print("Creating the Irrlicht visualization...")

vis = chronoirr.ChIrrApp(my_system, "HMMWV on Complex Terrain", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-5, 2, 0), chrono.ChVectorD(0, 0, 0))
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.SetSymbolScale(0.1)
vis.SetShowInfos(True)

# Bind visualization assets
vis.AssetBindAll()
vis.AssetUpdateAll()

# ---------------------------------------------------------------------
# Simulation parameters

print("Simulation starting...")

step_size = 1e-3  # Simulation step size
render_step_size = 1.0 / 60  # Time interval between visualization frames

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_time = 0

while vis.GetDevice().run():
    # Render scene and output POV-Ray data
    time = my_system.GetChTime()
    
    if time >= render_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_time += render_step_size
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)
    
    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    my_system.DoStepDynamics(step_size)
    vis.Advance(step_size)
    
    # Increment step number
    step_number += 1
    
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Print out vehicle speed
    if step_number % 500 == 0:
        speed_kmh = hmmwv.GetVehicle().GetVehicleSpeed() * 3.6
        print(f"Time: {time:.1f} s    Speed: {speed_kmh:.1f} km/h")

print("Simulation ended.")