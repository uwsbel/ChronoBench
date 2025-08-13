import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
print(veh)

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  






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


vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(10, 0, 0.5), chrono.QUNIT))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()
vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


vis2 = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis2.SetWindowTitle('Sedan 2')
vis2.SetWindowSize(1280, 1024)
vis2.SetChaseCamera(trackPoint, 6.0, 0.5)
vis2.Initialize()
vis2.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis2.AddLightDirectional()
vis2.AddSkyBox()
vis2.AttachVehicle(vehicle2.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


driver2 = veh.ChInteractiveDriverIRR(vis2)
driver2.SetSteeringDelta(render_step_size / 1.0)
driver2.SetThrottleDelta(render_step_size / 1.0)
driver2.SetBrakingDelta(render_step_size / 0.3)
driver2.Initialize()

steering_input_1 = 0.0
steering_input_2 = 0.0
throttle_input_1 = 0.0
throttle_input_2 = 0.0
braking_input_1 = 0.0
braking_input_2 = 0.0

steering_input_1 = math.sin(time * 0.5) * 0.8
steering_input_2 = math.sin(time * 0.7) * 0.8
throttle_input_1 = math.sin(time * 0.3) * 0.2
throttle_input_2 = math.sin(time * 0.4) * 0.2
braking_input_1 = math.sin(time * 0.9) * 0.2
braking_input_2 = math.sin(time * 1.0) * 0.2

driver_inputs = driver.GetInputs()
driver_inputs.m_steering = steering_input_1
driver_inputs.m_throttle = throttle_input_1
driver_inputs.m_braking = braking_input_1

driver_inputs2 = driver2.GetInputs()
driver_inputs2.m_steering = steering_input_2
driver_inputs2.m_throttle = throttle_input_2
driver_inputs2.m_braking = braking_input_2






print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()
    driver_inputs.m_steering = steering_input_1
    driver_inputs.m_throttle = throttle_input_1
    driver_inputs.m_braking = braking_input_1

    driver_inputs2 = driver2.GetInputs()
    driver_inputs2.m_steering = steering_input_2
    driver_inputs2.m_throttle = throttle_input_2
    driver_inputs2.m_braking = braking_input_2

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    vehicle2.Synchronize(time, driver_inputs2, terrain)
    vis2.Synchronize(time, driver_inputs2)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle2.Advance(step_size)
    vis2.Advance(step_size)
    
    step_number += 1

    
    realtime_timer.Spin(step_size)