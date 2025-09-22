import math
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens





chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type              = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE
tire_model             = veh.TireModelType_TMEASY


terrain_height = 0
terrain_length = 100.0
terrain_width  = 100.0


contact_method = chrono.ChContactMethod_NSC


trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)


step_size       = 1e-3
tire_step_size  = step_size
render_step_size = 1.0 / 50.0     
log_step_size    = 1.0 / 20.0     





vehicle = veh.HMMWV_Full()
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





patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch   = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Demo")
vis.SetWindowSize(1280, 1024)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AttachVehicle(vehicle.GetVehicle())





driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0)   
driver.SetThrottleDelta(0)
driver.SetBrakingDelta(0)
driver.Initialize()


CONST_STEERING = 0.6
CONST_THROTTLE = 0.5





manager = sens.ChSensorManager(vehicle.GetSystem())


imu_offset_pose = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 1),            
    chrono.ChQuaternionD(1, 0, 0, 0)
)
imu = sens.ChAccelerometerSensor(
    vehicle.GetChassisBody(),
    10,                                   
    imu_offset_pose,
    sens.ChNoiseNone()
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)


gps_offset_pose = imu_offset_pose                                  
gps_ref = chrono.ChVectorD(-89.400, 43.070, 260.0)                 
gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),
    10,                                                            
    gps_offset_pose,
    gps_ref,
    sens.ChNoiseNone()
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)





print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
log_steps    = math.ceil(log_step_size    / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

gps_data = []          

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    
    
    driver.SetSteering(CONST_STEERING)
    driver.SetThrottle(CONST_THROTTLE)
    driver_inputs = driver.GetInputs()

    
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    
    if step_number % log_steps == 0:
        buf = gps.GetMostRecentGPSBuffer()
        if buf is not None:
            coor = buf.GetGPSData()      
            gps_data.append([coor[0], coor[1], coor[2]])

    
    
    
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




print("Simulation finished – plotting GPS trajectory.")

if gps_data:
    lons = [p[0] for p in gps_data]
    lats = [p[1] for p in gps_data]

    plt.figure(figsize=(8, 6))
    plt.plot(lons, lats, 'b-', lw=2, label="GPS trajectory")
    plt.xlabel("Longitude [deg]")
    plt.ylabel("Latitude  [deg]")
    plt.title("Vehicle GPS trajectory")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print("No GPS data collected.")