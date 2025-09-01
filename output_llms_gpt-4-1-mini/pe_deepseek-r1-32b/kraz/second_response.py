import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.5)  # Changed from (0,0,0.5) to (-15,0,0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Track point for camera (changed from (0,0,2.1) to (3,0,2.1))
trackPoint = chrono.ChVector3d(3, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the kraz vehicle, set parameters, and initialize
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
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

# Create the vehicle Irrlicht interface
vis = irr.ChVisualSystemIrrlicht()  # Fixed: Changed from veh.ChWheeledVehicleVisualSystemIrrlicht to irr.ChVisualSystemIrrlicht
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)  # Adjusted chase camera distance to 10.5
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())

# Create the driver system with lane change maneuver
driver_inputs = veh.ChDriverInputs()

# Define lane change maneuver sequence
def lane_change_maneuver(time):
    # Double lane change sequence
    if time < 2.0:
        return 0.0  # Straight
    elif 2.0 <= time < 4.0:
        return math.sin((time - 2.0) * math.pi)  # First lane change
    elif 4.0 <= time < 6.0:
        return -math.sin((time - 4.0) * math.pi)  # Second lane change
    else:
        return 0.0  # Straight

# Set the time response for steering and throttle inputs
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

# Initialize simulation variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get driver inputs
    driver_inputs.steering = lane_change_maneuver(time)  # Automated steering input
    driver_inputs.throttle = 0.5  # Constant throttle
    driver_inputs.braking = 0.0

    # Render scene
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update modules
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)