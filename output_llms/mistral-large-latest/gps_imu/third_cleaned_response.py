import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import matplotlib.pyplot as plt


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


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  
log_step_size = 1.0 / 20    


vehicle = veh.HMMWV_Full() 
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


patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
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


offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
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

gps.PushFilter(sens.ChFilterGPSAccess())

manager.AddSensor(gps)






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)
log_steps = math.ceil(log_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

gps_data = []

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    if step_number % log_steps == 0:
        
        gps_coor = gps.GetMostRecentGPSBuffer().GetGPSData()
        gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])
    
    driver.SetThrottle(0.5)
    driver.SetSteering(0.6)
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    manager.Update()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


latitudes = [coor[0] for coor in gps_data]
longitudes = [coor[1] for coor in gps_data]
plt.plot(longitudes, latitudes, marker='o')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('GPS Trajectory')
plt.grid(True)
plt.show()

print("GPS Data: ", gps_data)