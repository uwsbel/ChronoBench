import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors


chrono.ChSystemNSC my_system()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.ChVehicleSystemNSC()
vehicle.Initialize(chrono.GetChronoDataPath() + 'vehicle/hmmwv/hmmwv_chassis.hrb')


terrain = veh.ChTerrainNSC()
terrain.SetContactMaterial(0.8, 0.2, 0.01, 0.01, 0.01, 3.4, 0.005, 0.01)
terrain.Initialize(chrono.GetChronoDataPath() + 'terrain/flat.ter')
terrain.SetTexture(chrono.GetChronoDataPath() + 'terrain/texture.jpg', True, 100)
terrain.SetOrigin(chrono.ChVectorD(0, 0, 0))
terrain.SetUpVector(chrono.ChVectorD(0, 1, 0))
terrain.SetScaleFactor(chrono.ChVectorD(1, 1, 1))


my_system.Add(terrain)


my_system.Add(vehicle)


driver = veh.ChDriverNSC()
driver.Initialize(vehicle, chrono.ChFileName(chrono.GetChronoDataPath() + 'vehicle/hmmwv/driver_hmmwv.txt'))
driver.SetSteeringController(veh.ChDriverSteeringControllerPID(2.0, 0.1, 0.01, 0))
driver.SetThrottleController(veh.ChDriverThrottleControllerPID(0.2, 0.1, 0.01, 0))


imu_sensor = sensors.ChSensorIMU()
imu_sensor.Initialize(vehicle.GetChassisBody(), my_system)
imu_sensor.SetName("IMU Sensor")
imu_sensor.SetUpdateMode(sensors.ChSensor::UPD_MODE_ON_DEMAND)
my_system.Add(imu_sensor)

gps_sensor = sensors.ChSensorGPS()
gps_sensor.Initialize(vehicle.GetChassisBody(), my_system)
gps_sensor.SetName("GPS Sensor")
gps_sensor.SetUpdateMode(sensors.ChSensor::UPD_MODE_ON_DEMAND)
my_system.Add(gps_sensor)


application = chronoirr.ChIrrApp(my_system, 'HMMWV Simulation', chronoirr.dimension2du(1280, 720))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_white.png')
application.AddLightWithShadow(chrono.ChVectorD(1.0, 1.0, 1.0), chrono.ChVectorD(1.0, -1.0, 1.0), 1.0, 1.5)
application.SetChaseCamera(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, -3), chrono.ChVectorD(0, 1, 0))

application.AssetBindAll()
application.AssetUpdateAll()


my_system.SetTimestepperType(chrono.ChTimestepper.HBDF)
my_system.SetMaxPenetrationRecoverySpeed(2.0)

step_size = 1e-3
time = 0.0
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    
    imu_sensor.Update()
    gps_sensor.Update()

    
    print(f"Time: {time:.2f} s, Vehicle Mass: {vehicle.GetChassisBody().GetMass()} kg")

    
    my_system.DoStepDynamics(step_size)
    time += step_size


application.GetDevice().closeDevice()