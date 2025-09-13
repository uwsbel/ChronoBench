import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Updated initial location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.5)  # Changed from (0,0,0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # Unchanged rotation

# Visualization and collision settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Rigid terrain with corrected height
terrainHeight = 0.5  # Corrected to match vehicle height
terrainLength = 100.0
terrainWidth = 100.0

# Updated camera track point and chase distance
trackPoint = chrono.ChVector3d(3, 0, 2.1)  # Changed from (0,0,2.1)
chase_dist = 25.0
chase_height = 10.5  # Changed from 1.5

# Simulation parameters
contact_method = chrono.ChContactMethod_NSC
contact_vis = False
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize vehicle
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireStepSize(tire_step_size)  # Initialize tire step size
vehicle.InitializeTires()  # Initialize tires with specified model

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight),  # Use terrainHeight
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, chase_dist, chase_height)  # Updated parameters
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())

# Create driver with double lane change maneuver
driver = veh.ChDriver(vehicle.GetTractor())
# Double lane change parameters
lc_start = 1.0
lc_length = 40.0
lc_width = 3.5
lc_speed = 10.0
driver = veh.ChDoubleLaneChangeDriver(vehicle.GetTractor(), lc_speed, lc_length, lc_width, lc_start)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)