import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# ---------------------
# Create the Chrono system
# ---------------------
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# ---------------------
# Create the HMMWV vehicle
# ---------------------
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.6)))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
hmmwv.SetDriveType(veh.DrivelineTypeWV_RWD)
hmmwv.Initialize()

# Add vehicle to the system
system.Add(hmmwv.GetVehicle())

# ---------------------
# Create the terrain
# ---------------------
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(-50, -50, 0), 
                         chrono.ChVectorD(50, 50, 0), 
                         0.1)
terrain.Initialize()

# ---------------------
# Create the driver
# ---------------------
driver = veh.ChAIDriver()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
driver.Initialize()

# ---------------------
# Create IMU and GPS sensors
# ---------------------
imu_sensor = chrono.ChSensorIMU(hmmwv.GetChassisBody())
imu_sensor.SetName("IMU Sensor")
imu_sensor.SetUpdateInterval(0.01)
system.AddSensor(imu_sensor)

gps_sensor = chrono.ChSensorGPS(hmmwv.GetChassisBody())
gps_sensor.SetName("GPS Sensor")
gps_sensor.SetUpdateInterval(0.01)
system.AddSensor(gps_sensor)

# ---------------------
# Set up the Irrlicht visualization
# ---------------------
application = irr.ChIrrApp(system, "HMMWV Simulation", irr.dimension2du(1024,768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(10, 10, 6), irr.vector3df(0, 0, 1))
application.SetChaseCamera(hmmwv.GetChassisBody(), 6.0, 0.5)
application.AssetBindAll()
application.AssetUpdateAll()

# ---------------------
# Simulation loop
# ---------------------
time_step = 0.01
time_end = 10

while application.GetDevice().run():
    time = system.GetChTime()

    # Driver inputs
    driver.SetSteering(0.5 * math.sin(time))
    driver.SetThrottle(0.2)
    driver.Synchronize(time)

    # Update vehicle and terrain
    hmmwv.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking(), terrain)
    terrain.Synchronize(time)

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update visualization
    application.DrawAll()

    # Output vehicle mass and sensor data
    print(f"Time: {time}, Vehicle Mass: {hmmwv.GetVehicle().GetMass()}")
    # print(f"IMU Data: {imu_sensor.Get_acc_loc()}")
    # print(f"GPS Data: {gps_sensor.Get_pos_ref()}")

    # End simulation
    if time >= time_end:
        break