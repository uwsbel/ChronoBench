import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set paths before running
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted)
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Simulation parameters
step_size = 0.0005
tire_step_size = step_size
render_step_size = 0.01  # 100 FPS
contact_method = chrono.ChContactMethod_NSC

# Create vehicle
vehicle = veh.Sedan()  # Corrected vehicle class
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
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength=100.0, terrainWidth=100.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = irr.ChIrrApp(vehicle.GetSystem())
vis.SetWindowSize(1280, 1024)
vis.SetWindowTitle('Sedan Simulation')
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.SetChaseCamera(chrono.ChVectorD(-5.0, 0.0, 1.8), 6.0, 0.5)
vis.AssetBind()
vis.AssetUpdate()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 3, 10))
vis.SetTimestep(step_size)
vis.Initialize()

# PID Controller Setup
desired_speed = 10.0  # m/s
kp = 0.5
ki = 0.01
kd = 0.1
error_prev = 0.0
integral = 0.0
driver = veh.ChDriver()  # Simple driver for PID control

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # PID Controller Logic
    current_speed = vehicle.GetSpeed()
    error = desired_speed - current_speed
    integral += error * step_size
    derivative = (error - error_prev) / step_size if step_size != 0 else 0
    output = kp * error + ki * integral + kd * derivative
    throttle = max(min(output, 1.0), 0.0)
    brake = 0.0
    
    driver.SetThrottle(throttle)
    driver.SetBraking(brake)
    error_prev = error
    
    driver_inputs = driver.GetInputs()
    
    # Update modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Advance simulation
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance()
    
    step_number += 1
    realtime_timer.Spin(step_size)

# Cleanup
vis.Close()