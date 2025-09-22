import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens


print('Copyright (c) 2023 Project Chrono')


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -5, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetInitFwdVel(0)
vehicle.SetInitWheelAngVel(chrono.Vect_4(0, 0, 0, 0))
vehicle.Initialize()


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e7, 0.01, 2e7, 0.01, 1.7e7, 0.01, 0.6e7, 0.006, 1.2, 0.4)
terrain.Initialize(terrain_length, terrain_width, terrain_grid_spacing)


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV SCM Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


imu_pos = chrono.ChVector3d(0, 0, 1)
gps_pos = chrono.ChVector3d(0, 0, 1)
imu = sens.ChSensorIMU(imu_pos, chrono.ChQuaterniond(1, 0, 0, 0), vehicle.GetChassisBody(), 100)
gps = sens.ChSensorGPS(gps_pos, chrono.ChQuaterniond(1, 0, 0, 0), vehicle.GetChassisBody(), 10)


vehicle.GetChassisBody().AddSensor(imu)
vehicle.GetChassisBody().AddSensor(gps)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)
    
    
    imu.Update()
    gps.Update()
    
    
    imu_data = imu.GetMostRecentBuffer()
    gps_data = gps.GetMostRecentBuffer()
    
    
    print(f"Vehicle mass: {vehicle.GetVehicle().GetMass()}")
    
    
    vehicle.Advance(0.01)
    terrain.Advance(0.01)
    vis.Advance(0.01)
    
    
    vis.Render()