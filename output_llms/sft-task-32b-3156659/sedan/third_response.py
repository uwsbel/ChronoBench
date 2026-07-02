import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted)
initLoc = chrono.ChVector3d(0, -10, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method and visualization
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes (decreased for finer control)
step_size = 5e-4  # 0.0005
tire_step_size = step_size
render_step_size = 0.01  # 100 FPS

# Create the vehicle system
vehicle = veh.BMW_E90()
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

# Create highway mesh terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.MeshTerrain(vehicle.GetSystem())
terrain.SetContactMaterial(patch_mat)
terrain.AddMesh(veh.GetDataFile("terrain/meshes/highway.obj"), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 1, 1))
terrain.Initialize()

# Create visualization interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan with PID Throttle Control')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)

# Adjusted steering response time (5 seconds)
steering_time = 5.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Reference speed and PID controller parameters
ref_speed = 20.0  # m/s
Kp = 0.1
Ki = 0.01
Kd = 0.05
integral = 0.0
previous_error = 0.0

# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Simulation loop parameters
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

    # PID controller for throttle based on speed error
    current_speed = vehicle.GetVehicle().GetSpeed()
    error = ref_speed - current_speed
    integral += error * step_size
    derivative = (error - previous_error) / step_size
    throttle = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    
    # Override throttle input with PID output
    driver_inputs.throttle = max(0.0, min(1.0, throttle))  # Clamp between 0 and 1

    # Synchronize modules
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