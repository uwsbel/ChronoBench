import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set the path for Chrono data
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation for vehicle 1
initLoc1 = chrono.ChVector3d(0, 0, 0.5)
initRot1 = chrono.ChQuaterniond(1, 0, 0, 0)

# Initial vehicle location and orientation for vehicle 2
initLoc2 = chrono.ChVector3d(0, 0, 0.5)
initRot2 = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# --------------
# Create systems
# --------------

# Create the vehicles
vehicle1 = veh.BMW_E90()
vehicle2 = veh.Truck()

# Initialize vehicle parameters
def InitializeVehicle(vehicle, initLoc, initRot):
    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(chassis_collision_type)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(vis_type)
    vehicle.SetSuspensionVisualizationType(vis_type)
    vehicle.SetSteeringVisualizationType(vis_type)
    vehicle.SetWheelVisualizationType(vis_type)
    vehicle.SetTireVisualizationType(vis_type)
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Initialize both vehicles
InitializeVehicle(vehicle1, initLoc1, initRot1)
InitializeVehicle(vehicle2, initLoc2, initRot2)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle1.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interfaces
vis1 = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis2 = veh.ChWheeledVehicleVisualSystemIrrlicht()

vis1.SetWindowTitle('Vehicle 1')
vis1.SetWindowSize(1280, 1024)
vis1.SetChaseCamera(trackPoint, 6.0, 0.5)
vis1.Initialize()
vis1.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis1.AddLightDirectional()
vis1.AddSkyBox()
vis1.AttachVehicle(vehicle1.GetVehicle())

vis2.SetWindowTitle('Vehicle 2')
vis2.SetWindowSize(1280, 1024)
vis2.SetChaseCamera(trackPoint, 6.0, 0.5)
vis2.Initialize()
vis2.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis2.AddLightDirectional()
vis2.AddSkyBox()
vis2.AttachVehicle(vehicle2.GetVehicle())

# Create the drivers
driver1 = veh.ChInteractiveDriverIRR(vis1)
driver2 = veh.ChInteractiveDriverIRR(vis2)

# Set time responses for steering and throttle
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta(render_step_size / braking_time)

driver2.SetSteeringDelta(render_step_size / steering_time)
driver2.SetThrottleDelta(render_step_size / throttle_time)
driver2.SetBrakingDelta(render_step_size / braking_time)

# Initialize drivers
driver1.Initialize()
driver2.Initialize()

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass (for vehicle 1)
print("VEHICLE MASS: ", vehicle1.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis1.Run() or vis2.Run():
    time = vehicle1.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis1.BeginScene()
        vis1.Render()
        vis2.BeginScene()
        vis2.Render()
        vis1.EndScene()
        vis2.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs1 = driver1.GetInputs()
    driver_inputs2 = driver2.GetInputs()

    # Update modules
    driver1.Synchronize(time)
    driver2.Synchronize(time)
    vehicle1.Synchronize(time, driver_inputs1, terrain)
    vehicle2.Synchronize(time, driver_inputs2, terrain)
    vis1.Synchronize(time, driver_inputs1)
    vis2.Synchronize(time, driver_inputs2)

    # Advance simulation for one timestep for all modules
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    terrain.Advance(step_size)
    vis1.Advance(step_size)
    vis2.Advance(step_size)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Generate sinusoidal steering input
    def generate_steering(angle):
        return int(math.sin(time * 2) * 0.1)  # Scale the sinusoid to vehicle control

    # Apply sinusoidal steering to both vehicles
    steering_angle1 = generate_steering(time)
    steering_angle2 = generate_steering(time)

    # Update vehicle controls
    if abs(steering_angle1) < 1.0:
        driver1.SetSteeringInput(steering_angle1)
    if abs(steering_angle2) < 1.0:
        driver2.SetSteeringInput(steering_angle2)

# End of simulation
print("Simulation ended.")