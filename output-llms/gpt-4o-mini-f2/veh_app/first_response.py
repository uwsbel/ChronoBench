import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH
# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0, 0, 1.54)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 20  # FPS = 50

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()

# Create the driver system
driver = veh.ChInteractiveDriverChatty(vehicle.GetChassis())

# Create a sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Create two point lights
intensity = 0.1
light = sens.ChPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
light.Attach(manager)
manager.AddSensor(light)

light2 = sens.ChPointLight(chrono.ChVector3f(8, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
light2.Attach(manager)
manager.AddSensor(light2)

# Create a camera and add it to the manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(0.0, -6.0, 2.5), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    vehicle.GetChassis(),             # Body camera is attached to
    update_rate,          # Image update rate in Hz
    offset_pose,          # Offset pose
    1280, 720,            # Image width and height
    1.57, 0.78             # Horizontal and vertical fields of view
)
cam.SetName("First Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
manager.AddSensor(cam)

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while (True):
    time = vehicle.GetSystem().GetChTime()

    # Render the scene and output POV-Ray data
    if (step_number % render_steps == 0):
        # Update sensor manager
        manager.Update()
        # Renderscene()
        render_frame += 1

    # Get the driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    manager.Synchronize(time)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    manager.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)