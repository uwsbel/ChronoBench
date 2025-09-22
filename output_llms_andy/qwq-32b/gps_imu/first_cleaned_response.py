import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


app = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddTypicalSky()
app.SetChaseCamera(chrono.ChVectorD(0, 0.5, 0), 6)


terrain = veh.ChTerrain()
terrain.SetHeight(0)  
terrain.SetSize(200, 200)  
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.png"), 200, 200)
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.SetFriction(0.9)
system.Add(terrain)
app.Add(terrain)


vehicle = veh.HMMWV()
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))  
vehicle.Initialize(system, terrain, veh.ChTireModelType.RIGID)
app.Add(vehicle.GetVehicle())


driver = veh.ChDriver()
driver.SetThrottle(0.5)  
driver.SetSteering(0)
driver.Initialize(vehicle.GetVehicle())


imu = veh.ChImu()
imu.Initialize(vehicle.GetChassis(), vehicle.GetChassis().GetFrame_REF_to_abs(), 0.01)
vehicle.AddSensors(imu)

gps = veh.ChGPS()
gps.Initialize(vehicle.GetChassis(), vehicle.GetChassis().GetFrame_REF_to_abs(), 0.01)
vehicle.AddSensors(gps)


while app.GetDevice().run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    vehicle.Synchronize(time)
    
    
    app.DoStepIfNotPaused()
    
    
    imu.Update()
    gps.Update()
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    print(f"Time: {time:.2f}s | IMU Acceleration: {imu.GetAcceleration()} | GPS Position: {gps.GetPos()}")
    
    
    if time < 0.1:
        print(f"Vehicle Chassis Mass: {vehicle.GetChassis().GetMass():.2f} kg")


app.GetDevice().closeGraphicalWindow()