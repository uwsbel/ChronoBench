import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization types
vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Track point for camera
trackPoint = chrono.ChVector3d(-15.0, 10.0, 5.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50  # FPS = 50

# Create the vehicle (CityBus)
vehicle = veh.CityBus()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

# Use Bullet collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('City Bus Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 3.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create data-driven driver system
# Define driver input sequence with timestamps
# Using veh.vector_Entry and veh.DataDriverEntry

# Entry at time 0.0: throttle=0.0, steering=0.0, braking=0.0
entry1 = veh.vector_Entry()
entry1.push_back(veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0))

# Entry at time 0.1: throttle=1.0, steering=0.0, braking=0.0
entry2 = veh.vector_Entry()
entry2.push_back(veh.DataDriverEntry(0.1, 1.0, 0.0, 0.0))

# Entry at time 0.5: throttle=1.0, steering=0.7, braking=0.0
entry3 = veh.vector_Entry()
entry3.push_back(veh.DataDriverEntry(0.5, 1.0, 0.7, 0.0))

# Combine entries into a vector
driver_sequence = [entry1, entry2, entry3]

# Initialize the data driver with the sequence
driver = veh.ChDataDriver()
driver.Initialize(vehicle.GetVehicle(), driver_sequence)

# Remove time response settings as per instructions
# (No code needed here since they are not included)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation parameters
render_steps = math.ceil(render_step_size / step_size)

# Initialize real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

# Main simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified intervals
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs from data driver
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame counter
    step_number += 1

    # Keep real-time pace
    realtime_timer.Spin(step_size)