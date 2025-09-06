import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))

# Initialize system
sys = chrono.ChSystemNSC()

# Create HMMWV vehicle
vehicle = veh.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_ALL)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize(sys)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 200, 100)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()

# Create driver
driver = veh.ChDataDriver(vehicle)
driver.Initialize()

# Add sensors
imu = sens.ChIMUSensor(
    vehicle.GetChassisBody(),
    100,  # Update rate (Hz)
    chrono.ChVector3d(0, 0, 0),  # Position on chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  # Orientation
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetNoise(0.01)
sys.AddSensor(imu)

gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),
    10,  # Update rate (Hz)
    chrono.ChVector3d(0, 0, 0),  # Position on chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  # Orientation
)
gps.SetName("GPS Sensor")
gps.SetReferenceLocation(chrono.ChVector3d(0, 0, 0))
sys.AddSensor(gps)

# Create sensor manager
manager = sens.ChSensorManager(sys)
manager.AddSensor(imu)
manager.AddSensor(gps)

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation with Sensors')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
time_step = 1e-3
while vis.Run():
    time = sys.GetChTime()
    
    # Synchronize modules
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    # Update sensors
    manager.Update()
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Output vehicle mass
    print(f"Time: {time:.3f} s, Vehicle mass: {vehicle.GetTotalMass():.2f} kg")
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()