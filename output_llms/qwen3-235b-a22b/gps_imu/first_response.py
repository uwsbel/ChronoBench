import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrlicht
import pychrono.sensor as sensor

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create and initialize the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create the terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(200, 1, 200), 0.8)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create Irrlicht visualization
app = irrlicht.ChIrrApp(system, 'HMMWV Simulation', irrlicht.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 1, 0))
app.AssetBindAll()
app.AssetUpdateAll()

# Create the driver
driver = veh.ChIrrGuiDriver(app)
driver.Initialize()

# Create and add IMU and GPS sensors
imu = sensor.ChIMU(vehicle.GetChassisBody(), 100, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
imu.SetName("IMU Sensor")
gps = sensor.ChGPS(vehicle.GetChassisBody(), 10, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
gps.SetName("GPS Sensor")

# Create sensor manager and add sensors
manager = sensor.ChSensorManager(system)
manager.AddSensor(imu)
manager.AddSensor(gps)

# Output vehicle mass
print(f"Vehicle mass: {vehicle.GetVehicle().GetMass()} kg")

# Simulation loop
step_size = 1e-3  # Simulation step size
time = 0.0

while app.GetDevice().run():
    # Get current time
    time = system.GetChTime()
    
    # Get current driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    vehicle.GetVehicle().Synchronize(driver_inputs)
    terrain.Synchronize(driver_inputs)
    driver.Synchronize("")
    manager.Synchronize("", time)

    # Advance all modules by one step
    vehicle.GetVehicle().Advance(step_size)
    terrain.Advance(step_size)
    driver.Advance(step_size)
    manager.Advance(step_size)

    # Advance the system dynamics
    system.DoStepDynamics(step_size)

    # Update sensors
    manager.Update()

    # Render the scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    # Output sensor data periodically (every 0.1 seconds)
    if int((time / 0.1) * 10) % 10 == 0:
        print(f"IMU Acceleration: {imu.GetMostRecentAccel()}")
        print(f"GPS Position: {gps.GetMostRecentPos()}")