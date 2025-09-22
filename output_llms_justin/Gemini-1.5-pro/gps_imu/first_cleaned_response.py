import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetStep(1e-3)


vehicle_file = "hmmwv/hmmwv_Vehicle.xml"
tire_file = "hmmwv/hmmwv_TMeasy.xml"
powertrain_file = "hmmwv/hmmwv_PowertrainSimple.xml"


vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))


tire_front_left = veh.TMeasy(tire_file)
tire_front_right = veh.TMeasy(tire_file)
tire_rear_left = veh.TMeasy(tire_file)
tire_rear_right = veh.TMeasy(tire_file)
vehicle.SetTire(tire_front_left,  veh.VehicleSide.FRONT_LEFT)
vehicle.SetTire(tire_front_right, veh.VehicleSide.FRONT_RIGHT)
vehicle.SetTire(tire_rear_left, veh.VehicleSide.REAR_LEFT)
vehicle.SetTire(tire_rear_right, veh.VehicleSide.REAR_RIGHT)


powertrain = veh.SimplePowertrain(powertrain_file)
vehicle.SetPowertrain(powertrain)


driver = veh.ChVehicleDriverSimple(vehicle)
driver.SetSteeringDelta(0)
driver.SetThrottle(0)
driver.SetBraking(0)




terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(0.8, 0.1, 10000), 
                         chrono.ChVectorD(-50, -5, 0), chrono.ChVectorD(50, -5, 0), 
                         chrono.ChVectorD(0, 10, 0), 1, 1)




sensor_manager = veh.ChSensorManager(system)
imu_sensor = sensor_manager.AddSensor(veh.ChSensorImu(vehicle.GetChassis(), chrono.ChVectorD(0, 0, 0)))
gps_sensor = sensor_manager.AddSensor(veh.ChSensorGPS(vehicle.GetChassis(), chrono.ChVectorD(0, 0, 0)))




application = veh.ChVehicleIrrApp(vehicle)
application.SetSkyBox()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(10, 10, 5), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()




time = 0
while application.GetDevice().run():
    time += system.GetStep()

    
    driver.SetSteeringDelta(math.sin(time) * 0.5)
    driver.SetThrottle(0.5)

    
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking(), terrain)
    terrain.Synchronize(time)
    sensor_manager.Update(time) 

    
    system.DoStepDynamics(system.GetStep())

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    imu_data = imu_sensor.Get_Data()
    gps_data = gps_sensor.Get_Data()

    
    print("Vehicle Mass:", vehicle.GetVehicleMass())