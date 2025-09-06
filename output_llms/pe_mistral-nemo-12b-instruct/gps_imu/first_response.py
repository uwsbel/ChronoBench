import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set Chrono's data path
chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', '..', 'data'))

# Initialize the system and visualization
sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()

# Create and initialize the HMMWV vehicle
hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(veh.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.ChCollisionType_BULLET)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.Initialize()

# Create and initialize the terrain
terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Create and initialize the driver inputs
driver = veh.HMMWVDriver()
driver.Initialize(hmmwv)

# Create and initialize IMU and GPS sensors
imu = veh.ChSensorIMU()
gps = veh.ChSensorGPS()
hmmwv.Attach(imu, chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaternionD(1, 0, 0, 0))
hmmwv.Attach(gps, chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaternionD(1, 0, 0, 0))

# Add logo and camera to the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVector3d(0, 5, -10))

# Simulation loop
while vis.Run():
    # Advance the vehicle, terrain, driver, and sensors
    sys.DoStepDynamics(0.01)
    driver.Advance()
    imu.Update()
    gps.Update()

    # Output vehicle mass (for demonstration purposes)
    print(f'Vehicle mass: {hmmwv.GetVehicleMass():.2f} kg')

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Clean up and exit
vis.Cleanup()