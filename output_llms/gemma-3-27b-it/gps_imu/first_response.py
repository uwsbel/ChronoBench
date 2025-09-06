import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sensor

# --- Simulation Setup ---
# Create a Chrono system
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("../data/")  # Adjust path as needed

# --- Terrain Creation ---
terrain = veh.RigidTerrain(sys)
terrain.SetMeshFile("terrain/flat_terrain.obj")
terrain.SetContactMaterial(chrono.ChContactMaterialSMC())
terrain.SetContactOffset(0.01)
terrain.Initialize()

# --- Vehicle Initialization ---
hmmwv = veh.HMMWV(sys)
hmmwv.SetContactMethod(veh.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.ChChassisCollision_AUTO)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
hmmwv.Initialize()

# --- Driver Input ---
driver = veh.HMMWV_Driver(hmmwv)
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)

# --- Sensor Initialization ---
# IMU Sensor
imu = sensor.ChIMUSensor()
imu.SetFrequency(100.0)  # Hz
imu.Attach(hmmwv.GetChassis())
imu.SetPos(chrono.ChVector3d(0, 0.5, 0))  # Position relative to chassis
imu.SetNoise(0.01, 0.01, 0.01) #Noise for accelerometer and gyroscope

# GPS Sensor
gps = sensor.ChGPSSensor()
gps.SetFrequency(10.0)  # Hz
gps.Attach(hmmwv.GetChassis())
gps.SetPos(chrono.ChVector3d(0, 1.5, 0))  # Position relative to chassis
gps.SetNoise(0.1, 0.1, 0.1) #Noise for latitude, longitude, altitude

# --- Visualization Setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation with Sensors')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(10, 5, -10))
vis.AddTypicalLights()
vis.AddSkyBox()

# --- Simulation Loop ---
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update driver inputs
    if driver.GetThrottle() < 1:
        driver.SetThrottle(driver.GetThrottle() + driver.GetThrottleDelta())
    if driver.GetSteering() < 1:
        driver.SetSteering(driver.GetSteering() + driver.GetSteeringDelta())

    # Advance the simulation
    sys.DoStepDynamics(time_step)

    # Update sensor data
    imu.Update()
    gps.Update()

    # Output vehicle mass and sensor data
    print(f"Vehicle Mass: {hmmwv.GetMass():.2f}")
    print(f"IMU Acceleration: {imu.GetAcc():.2f}")
    print(f"GPS Latitude: {gps.GetLatitude():.6f}")

    # Add a visual marker for the IMU and GPS sensor positions
    vis.GetSceneManager().addBillboardSceneNode(
        chrono.ChVector3d(imu.GetPos().x, imu.GetPos().y, imu.GetPos().z),
        chrono.ChVector3d(0.1, 0.1, 0.1),
        vis.GetSceneManager().getRootSceneNode(),
        "media/arrow.png",
        chrono.ChColor(1, 0, 0)  # Red
    )

    vis.GetSceneManager().addBillboardSceneNode(
        chrono.ChVector3d(gps.GetPos().x, gps.GetPos().y, gps.GetPos().z),
        chrono.ChVector3d(0.1, 0.1, 0.1),
        vis.GetSceneManager().getRootSceneNode(),
        "media/arrow.png",
        chrono.ChColor(0, 0, 1)  # Blue
    )