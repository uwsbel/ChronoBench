import pychrono as chrono
import pychrono_vehicle as veh
import pychrono_sensor




chrono.SetChronoDataPath(chrono.GetChronoDataPath())  


system = chrono.ChSystemNSC()



vehicle = veh.HMMWV_Full(system, chrono.vehicle.CollideType_NONE)


init_loc = chrono.ChVectorD(0, 0, 1.0)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))


vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
                         150.0, 150.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)
patch.SetContactFriction(0.9f)
patch.SetContactRestitution(0.01f)
patch.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize()


time_step = 1e-3
driver = veh.HMMWV_Driver(vehicle, veh.GetDataFile("vehicle/hmmwv/driver/HMMWV_acc_very_smooth.json"))
driver.Initialize()




sensor_manager = pychrono_sensor.SensorManager(system)
sensor_manager.SetVerbose(False)


imu_offset_pose = chrono.ChFrameD()  
imu_sensor = pychrono_sensor.ChSensor(
    vehicle.GetChassisBody(), 
    chrono.ChFrameD(imu_offset_pose.GetPos(), imu_offset_pose.GetRot()),
    1.0 / 60.0,  
    True,        
)
imu_sensor.PushFilter(pychrono_sensor.ChFilterAccelNoise(0.05))  
imu_sensor.PushFilter(pychrono_sensor.ChFilterGyroNoise(0.01))
sensor_manager.AddSensor(imu_sensor)


gps_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1))  
gps_sensor = pychrono_sensor.ChSensor(
    vehicle.GetChassisBody(),
    gps_offset_pose,
    10.0,  
    True,
)
gps_sensor.PushFilter(pychrono_sensor.ChFilterGPSNoise(1.5))  
sensor_manager.AddSensor(gps_sensor)




app = veh.ChIrrApp(vehicle.GetVehicle(), "HMMWV Simulation with Sensors", chrono.dimension2du(1280, 720))

app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(-8, -11, 4), chrono.ChVectorD(0, 0, 1))
app.SetTimestep(time_step)
app.AssetBindAll()
app.AssetUpdateAll()


end_time = 20.0




while app.GetDevice().run() and system.GetChTime() < end_time:
    
    driver_inputs = driver.GetInputs()

    
    time = system.GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    sensor_manager.Update()  
    app.BeginScene()
    app.DrawAll()

    
    print(f"Time: {time:.3f}, Vehicle Mass: {vehicle.GetVehicle().GetMass():.1f} kg")

    
    driver.Advance(time_step)
    vehicle.Advance(time_step)
    terrain.Advance(time_step)
    system.DoStepDynamics(time_step)
    app.EndScene()