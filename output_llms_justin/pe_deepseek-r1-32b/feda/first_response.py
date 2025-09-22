import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Initialize PyChrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataFile(''))

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('FEDA Vehicle Simulation')
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.SetCameraRotation(chrono.ChQuaterniond(1, 0, 0, 0))
vis.EnableCameraTracking(True)

# Create the physical system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the vehicle
vehicle = veh.FEDAVehicle()
vehicle.SetContactMethod(chrono.ChContactMethod.NSC)
vehicle.SetTireModel(veh.TireModelType.PACEJKa)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
vehicle.SetVisualizationType(veh.VisualizationType.MESH)
vehicle.Initialize(sys)

# Create the terrain
terrain = veh.RigidTerrain(sys)
terrain_texture = chrono.GetChronoDataFile('textures/concrete.jpg')
terrain_patch = terrain.AddPatch(terrain_texture, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Set camera to follow the vehicle
vis.GetCamera().SetDistance(10)
vis.GetCamera().SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))

# Set visualization for all vehicle parts to mesh type
for part in vehicle.GetComponentsList():
    if part.GetVisualShape():
        part.GetVisualShape().SetType(chrono.VisualShapeType.MESH)

# Create driver
driver = veh.Driver()
driver.SetDriverType(veh.DriverType.KEYBOARD)
vehicle.SetDriver(driver)

# Simulation loop
time_step = 0.02
frame_rate = 50

while vis.Run():
    # Update driver inputs
    driver.Synchronize(time_step)
    
    # Update vehicle
    vehicle.Update(time_step)
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Control frame rate
    chronoirr.Sleep(int(1000 / frame_rate))