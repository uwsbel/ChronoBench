import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sensor

# --- Initialization ---
chrono.SetChronoDataPath("./")  # Set the path to the Chrono data directory

# Create a Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# --- Terrain ---
terrain = chrono.ChTerrain()
terrain.Create(system, "terrain", 100, 100, 10, 10)
terrain.SetTexture(chrono.GetChronoDataPath() + "textures/terrain/grass.png", 0)
terrain.SetHeightFunction(chrono.ChFunctionToboggan())

# --- Vehicle Setup ---
hmmwv = veh.HMMWV(system)
hmmwv.SetVehicleModel("hmmwv")
hmmwv.SetTerrain(terrain)
hmmwv.SetTireForceModel(veh.TireForceModelType.MTM)
hmmwv.SetTireRadius(0.4)
hmmwv.SetTireWidth(0.3)
hmmwv.SetTirePressure(1.2)
hmmwv.SetEngineType(veh.EngineModelType.SIMPLE)
hmmwv.SetTransmissionType(veh.TransmissionModelType.SIMPLE)
hmmwv.SetDifferentialType(veh.DifferentialModelType.SIMPLE)
hmmwv.SetChassisHeight(0.7)
hmmwv.SetInitialPosition(chrono.ChVectorD(0, 2, 0))
hmmwv.SetInitialVelocity(chrono.ChVectorD(0, 0, 0))

# --- Driver Input ---
driver = veh.SimpleDriver(hmmwv)
driver.SetInput(veh.DriverInputs())
driver.SetTargetSpeed(20)
driver.SetTargetDirection(0)

# --- Sensors ---
# IMU Sensor
imu = sensor.ChIMUSensor()
imu.Attach(hmmwv.GetChassis())
imu.SetFrequency(60)  # Hz
imu.SetPosition(chrono.ChVectorD(0, 0.5, 0))
imu.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))

# GPS Sensor
gps = sensor.ChGPSSensor()
gps.Attach(hmmwv.GetChassis())
gps.SetFrequency(1)  # Hz
gps.SetPosition(chrono.ChVectorD(0, 0.5, 0))
gps.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))

# --- Visual System ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddTypicalLights()

# --- Simulation Loop ---
time_step = 0.01
total_time = 60

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update driver input
    driver.Update(time_step)

    # Advance the system
    system.DoStepDynamics(time_step)

    # Update sensor data
    imu.Update()
    gps.Update()

    # Output vehicle mass
    vehicle_mass = hmmwv.GetVehicleMass()
    print(f"Vehicle Mass: {vehicle_mass:.2f} kg")

    # Process IMU data (example)
    if imu.HasNewData():
        imu_data = imu.GetLatestData()
        #print(f"IMU Data: Acceleration={imu_data.acceleration}, Angular Velocity={imu_data.angular_velocity}")

    # Process GPS data (example)
    if gps.HasNewData():
        gps_data = gps.GetLatestData()
        #print(f"GPS Data: Latitude={gps_data.latitude}, Longitude={gps_data.longitude}, Altitude={gps_data.altitude}")