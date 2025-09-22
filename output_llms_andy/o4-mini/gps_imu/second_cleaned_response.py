import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type              = veh.VisualizationType_MESH
chassis_collision     = veh.CollisionType_NONE
tire_model            = veh.TireModelType_TMEASY


terrainHeight         = 0
terrainLength         = 100.0
terrainWidth          = 100.0


trackPoint            = chrono.ChVectorD(-3.0, 0.0, 1.1)


contact_method        = chrono.ChContactMethod_NSC


step_size             = 1e-3
tire_step_size        = step_size
render_fps            = 50
render_step_size      = 1.0 / render_fps




log_step_size = 0.1                             
log_steps      = math.ceil(log_step_size/step_size)
gps_data       = []                             




vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision)
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




patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch   = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)),
    terrainLength, terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()




manager = sens.ChSensorManager(vehicle.GetSystem())


imu_pose = chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 1.0),
                           chrono.ChQuaternionD(1,0,0,0))
imu = sens.ChAccelerometerSensor(
    vehicle.GetChassisBody(),
    100,                
    imu_pose,
    sens.ChNoiseNone()
)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)


gps_pose = chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 1.0),
                           chrono.ChQuaternionD(1,0,0,0))
gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),
    10,                 
    gps_pose,
    chrono.ChVectorD(-89.400, 43.070, 260.0),
    sens.ChNoiseNone()
)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)




print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    
    driver.Synchronize(t)
    inputs = driver.GetInputs()

    
    
    
    
    if t < 3.0:
        inputs.m_throttle = 0.5
        inputs.m_steering = 0.1
        inputs.m_braking  = 0.0
    elif t < 6.0:
        inputs.m_throttle = 1.0
        inputs.m_steering = 0.0
        inputs.m_braking  = 0.0
    else:
        inputs.m_throttle = 0.0
        inputs.m_steering = 0.0
        inputs.m_braking  = 0.5

    
    
    
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    
    if step_number % log_steps == 0:
        buf = gps.GetMostRecentGPSBuffer()
        data = buf.GetGPSData()     
        gps_data.append((t, data))

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    manager.Update()

    
    realtime_timer.Spin(step_size)
    step_number += 1




print("GPS Data (time, (lat, lon, alt)):")
for entry in gps_data:
    print(entry)