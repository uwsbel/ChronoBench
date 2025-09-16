import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import numpy as np


chrono.SetChronoOutputMode(chrono.ChOutputMode.CONSOLE)
chrono.SetVerboseMode(False)


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




terrain = veh.Terrain(my_system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.SetContactForceModel(chrono.ChSystemNSC.ContactForceModel::Hertz)
terrain.Initialize(100, 100, 0.5, -2)


vehicle = veh.HMMWV(my_system)
vehicle.Initialize(chrono.ChCoordinator::Static, chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetChassisFixed(False)
vehicle.SetTireType(veh.Tire::TMeasy)
vehicle.SetTireStepSize(1e-3)
vehicle.SetEngineType(veh.EngineModelType::SHARED)
vehicle.SetDriveType(veh.DrivelineType::AWD)
vehicle.SetTransmissionType(veh.TransmissionModelType::AUTOMATIC)


driver = veh.ChDriverHMMWV(my_system)
driver.Initialize(vehicle)
driver.SetSteeringControllerType(veh.ChSteeringController::Type::STEERING_CONTROLLER_PATH)
driver.SetThrottleControllerType(veh.ChThrottleController::Type::THROTTLE_CONTROLLER_CONSTANT_SPEED)
driver.SetSteeringDelta(0.1)
driver.SetTargetSpeed(10)



imu_sensor = sensor.ChIMUSensor(vehicle.GetChassisBody())
imu_sensor.SetName("IMU_Sensor")
imu_sensor.SetUpdateRate(100)  
imu_sensor.SetNoise(0.01)  
imu_sensor.SetDrift(0.001)  
vehicle.GetChassisBody().AddSensor(imu_sensor)


gps_sensor = sensor.ChGPSSensor(vehicle.GetChassisBody())
gps_sensor.SetName("GPS_Sensor")
gps_sensor.SetUpdateRate(10)  
gps_sensor.SetNoise(0.1)  
vehicle.GetChassisBody().AddSensor(gps_sensor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 3, 2))
vis.AddTypicalLights()
vis.SetCameraTracking(vehicle.GetChassisBody(), chrono.ChVectorD(0, 1.5, 0), 5)
vis.SetSymbolScale(0.01)


time_step = 1e-3
sim_time = 0
output_step = 10  

while sim_time < 10:  
    
    imu_sensor.Update()
    gps_sensor.Update()

    
    if my_system.GetChTime() % 0.1 < time_step:  
        print(f"Time: {my_system.GetChTime():.2f}s")
        print(f"Vehicle mass: {vehicle.GetVehicleMass():.2f} kg")
        print(f"IMU - Acceleration: {imu_sensor.GetAcceleration().Length()}")
        print(f"GPS - Position: {gps_sensor.GetPosition()}")

    
    driver.Synchronize(sim_time)

    
    vehicle.Synchronize(sim_time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    
    terrain.Synchronize(sim_time)

    
    my_system.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sim_time += time_step

print("Simulation completed.")