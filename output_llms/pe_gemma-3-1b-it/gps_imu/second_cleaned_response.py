import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math


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


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)
trackPoint.SetPosition(chrono.ChVector3d(0, 0, 0))
trackPoint.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))


collision_material = chrono.ChContactMaterialNSC()
collision_material.SetFriction(0.9)
collision_material.SetRestitution(0.01)


chassis_collision_model = chrono.ChCollisionModel.Type_BULLET


vehicle_collision_type = veh.CollisionType_BULLET


vehicle_collision_params = chrono.ChCollisionParams()
vehicle_collision_params.SetCollisionModel(chassis_collision_model)
vehicle_collision_params.SetCollisionType(vehicle_collision_type)


vehicle_collision_params.SetCollisionEnvelope(chrono.ChVector3d(0, 0, 0))


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(collision_material)
vehicle.SetChassisCollisionType(vehicle_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))
vehicle.SetInitialAcceleration(chrono.ChVector3d(0, 0, 0))
vehicle.SetFinalVelocity(chrono.ChVector3d(0, 0, 0))
vehicle.SetFinalAcceleration(chrono.ChVector3d(0, 0, 0))
vehicle.SetInitialRotation(chrono.ChQuaterniond(1, 0, 0, 0))
vehicle.SetDynamics(chrono.ChDynamics_None)


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1024, 768)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


manager = sens.ChSensorManager(vehicle.GetSystem())


offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1), chrono.QUNIT)
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),                     
                                 10,        
                                 offset_pose,          
                                 sens.ChNoiseNone())   
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)

imu.PushFilter(sens.ChFilterAccelAccess())

manager.AddSensor(imu)


gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                     
                                 10,        
                                 offset_pose,          
                                 chrono.ChVector3d(-89.400, 43.070, 260.0),  
                                 sens.ChNoiseNone())   
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)





print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time, driver_inputs, terrain)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    manager.Update()
    
    
    step_number += 1

    
    realtime_timer.Spin(step_size)