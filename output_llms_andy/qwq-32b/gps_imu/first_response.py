import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the Irrlicht visualization
app = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddTypicalSky()
app.SetChaseCamera(chrono.ChVectorD(0, 0.5, 0), 6)

# Create terrain
terrain = veh.ChTerrain()
terrain.SetHeight(0)  # Terrain plane at Y=0
terrain.SetSize(200, 200)  # X and Z dimensions
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.png"), 200, 200)
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.SetFriction(0.9)
system.Add(terrain)
app.Add(terrain)

# Create HMMWV vehicle
vehicle = veh.HMMWV()
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))  # Start above terrain
vehicle.Initialize(system, terrain, veh.ChTireModelType.RIGID)
app.Add(vehicle.GetVehicle())

# Create driver
driver = veh.ChDriver()
driver.SetThrottle(0.5)  # Forward motion
driver.SetSteering(0)
driver.Initialize(vehicle.GetVehicle())

# Add IMU and GPS sensors
imu = veh.ChImu()
imu.Initialize(vehicle.GetChassis(), vehicle.GetChassis().GetFrame_REF_to_abs(), 0.01)
vehicle.AddSensors(imu)

gps = veh.ChGPS()
gps.Initialize(vehicle.GetChassis(), vehicle.GetChassis().GetFrame_REF_to_abs(), 0.01)
vehicle.AddSensors(gps)

# Simulation loop
while app.GetDevice().run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Update vehicle and driver
    vehicle.Synchronize(time)
    
    # Advance simulation step
    app.DoStepIfNotPaused()
    
    # Update sensors
    imu.Update()
    gps.Update()
    
    # Render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    # Output sensor data
    print(f"Time: {time:.2f}s | IMU Acceleration: {imu.GetAcceleration()} | GPS Position: {gps.GetPos()}")
    
    # Output vehicle mass once
    if time < 0.1:
        print(f"Vehicle Chassis Mass: {vehicle.GetChassis().GetMass():.2f} kg")

# End simulation
app.GetDevice().closeGraphicalWindow()