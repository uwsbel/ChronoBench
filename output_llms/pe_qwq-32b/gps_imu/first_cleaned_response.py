import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.robot as robot


chrono.SetChronoDataPath(chrono.GetChronoDataFile(''))


system = chrono.ChSystemNSC()
application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.SetCamPosition(chrono.ChVectorD(0, 3, 15))
application.SetCamRotation(chrono.Q_from_AngAxis(-chrono.CH_C_PI/4, chrono.VECT_Y))
application.AddAsset(chrono.ChColorAsset(chrono.ChVectorf(0.5, 0.5, 0.5)))


vehicle = veh.HMMWV(vehicle.GetSystem(), "vehicle/HMMWV")  
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
vehicle.Initialize()


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(material, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
terrain.Initialize()
application.Add(terrain.GetGroundBody())


application.Add(vehicle.GetVehicle())


driver = veh.ChDriver()
vehicle.SetDriver(driver)
driver.SetThrottle(1.0)  
driver.SetSteering(0.0)   


imu = robot.ChIMUSensor(vehicle.GetChassis(), chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))
imu.Initialize()
system.AddSensor(imu)

gps = robot.ChGPSSensor(vehicle.GetChassis(), chrono.ChVectorD(0, 0, 0))
gps.Initialize()
system.AddSensor(gps)


step_size = 0.01
time = 0


while application.GetDevice().run():

    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver)

    
    system.DoStepDynamics(step_size)

    
    imu_data = imu.GetReading()
    gps_data = gps.GetReading()

    
    print(f"Time {time:.2f}s | Mass: {vehicle.GetChassis().GetMass():.2f} kg | GPS: {gps_data}")

    time += step_size