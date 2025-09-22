import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# ---------------------------------------------------------------------
# Initialize the simulation

# Set the path to Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))

# Create a Chrono system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Terrain

# Create the terrain
terrain = veh.RigidTerrain(system)

# Create a flat patch for the terrain
terrain_patch_dims = chrono.ChVectorD(100, 0.2, 100)
terrain_mat = chrono.ChMaterialSurfaceSMC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

patch_params = veh.RigidTerrainGeometry.Patch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, -0.1, 0), chrono.QUNIT),
    terrain_patch_dims
)
patch_params.m_material = terrain_mat

# Add a texture for the terrain
terrain_tex = veh.RigidTerrainVisualMesh(
    chrono.GetChronoDataFile("terrain/textures/dirt.jpg"),
    chrono.ChVectorD(terrain_patch_dims.x, 0, terrain_patch_dims.z),
    chrono.ChVectorD(5, 5, 5)
)
patch_params.m_visual_mesh = terrain_tex

# Add the terrain patch to the system
terrain.AddPatch(patch_params)
terrain.Initialize()

# ---------------------------------------------------------------------
# Vehicle

# Create the Gator vehicle from JSON specification file
gator_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                          "data/vehicle/Gator/vehicle/Gator.json")

gator = veh.WheeledVehicle(system, gator_file)

# Set the vehicle position and orientation
init_pos = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # Rotation around z-axis
gator.Initialize(chrono.ChCoordsysD(init_pos, init_rot))

# Set visualization type for vehicle components to MESH
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)

# ---------------------------------------------------------------------
# Powertrain

# Create the powertrain system
powertrain_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                              "data/vehicle/Gator/powertrain/Gator_SimpleMapPowertrain.json")
powertrain = veh.SimpleMapPowertrain(powertrain_file)
gator.InitializePowertrain(powertrain)

# ---------------------------------------------------------------------
# Tires

# Create tires using TMEASY tire model
tire_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                        "data/vehicle/Gator/tire/Gator_TMeasyTire.json")

for axle in range(gator.GetNumberOfAxles()):
    for wheel in range(2):
        tire = veh.TMeasyTire(tire_file)
        tire.SetVisualizationType(veh.VisualizationType_MESH)
        gator.InitializeTire(tire, axle, wheel, veh.WheelState_ROTATION)

# ---------------------------------------------------------------------
# Interactive driver

# Create the interactive driver
driver = veh.ChIrrGuiDriver(gator.GetVehicle())
driver.SetSteeringDelta(1.0 / 50)      # Steering increments
driver.SetThrottleDelta(1.0 / 50)      # Throttle increments
driver.SetBrakingDelta(1.0 / 50)       # Braking increments
driver.Initialize()

# ---------------------------------------------------------------------
# Visualization

# Create the vehicle Irrlicht application
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(3.5, 0.5, 1.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())

# ---------------------------------------------------------------------
# Simulation loop

# Set real-time simulation
realtime_timer = chrono.ChRealtimeStepTimer()
step_size = 1e-3  # Time step size

# Initialize simulation time and step counter
time = 0
step_number = 0

# Maximum simulation duration and data collection frequency
t_end = 1000  # Simulation end time
render_step_size = 1.0 / 50  # Render at 50 FPS
render_steps = math.ceil(render_step_size / step_size)

print("Vehicle mass: " + str(gator.GetVehicle().GetMass()))

# Simulation loop
while vis.Run() and time < t_end:
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules (vehicle, tires, powertrain, etc.)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)
    
    # Increment time and step counter
    time += step_size
    step_number += 1
    
    # Enforce real-time simulation
    realtime_timer.Spin(step_size)

# ---------------------------------------------------------------------
# Clean up
print("Simulation ended at time: " + str(time))