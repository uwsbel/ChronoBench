import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set this path before running the demo
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation adjusted
initLoc = chrono.ChVectorD(5, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-4
tire_step_size = step_size
render_step_size = 0.01
trackPoint = chrono.ChVectorD(-5.0, 0.0, 1.8)

# PID controller parameters
desired_speed = 10.0  # m/s
kp = 0.1
ki = 0.01
kd = 0.05
error_integral = 0.0
previous_error = 0.0

# Create vehicle
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType.BULLET)

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Simulation loop variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("VEHICLE MASS: ", vehicle.GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Compute PID control
    chassis = vehicle.GetChassis()
    current_speed = chassis.GetPos_dt().x
    error = desired_speed - current_speed
    error_integral += error * step_size
    derivative = (error - previous_error) / step_size
    pid_output = kp * error + ki * error_integral + kd * derivative
    throttle = max(0.0, min(1.0, pid_output))
    steering = 0.0  # Straight path
    braking = 0.0

    driver_inputs = veh.DriverInputs()
    driver_inputs.throttle = throttle
    driver_inputs.steering = steering
    driver_inputs.braking = braking
    previous_error = error

    # Update and advance modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set this path before running the demo
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation adjusted
initLoc = chrono.ChVectorD(5, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-4
tire_step_size = step_size
render_step_size = 0.01
trackPoint = chrono.ChVectorD(-5.0, 0.0, 1.8)

# PID controller parameters
desired_speed = 10.0  # m/s
kp = 0.1
ki = 0.01
kd = 0.05
error_integral = 0.0
previous_error = 0.0

# Create vehicle
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType.BULLET)

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Simulation loop variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("VEHICLE MASS: ", vehicle.GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Compute PID control
    chassis = vehicle.GetChassis()
    current_speed = chassis.GetPos_dt().x
    error = desired_speed - current_speed
    error_integral += error * step_size
    derivative = (error - previous_error) / step_size
    pid_output = kp * error + ki * error_integral + kd * derivative
    throttle = max(0.0, min(1.0, pid_output))
    steering = 0.0  # Straight path
    braking = 0.0

    driver_inputs = veh.DriverInputs()
    driver_inputs.throttle = throttle
    driver_inputs.steering = steering
    driver_inputs.braking = braking
    previous_error = error

    # Update and advance modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)