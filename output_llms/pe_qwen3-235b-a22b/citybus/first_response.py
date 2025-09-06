import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))

# Initialize the Chrono system
sys = chrono.ChSystemSMC()

# Create the CityBus vehicle
bus = veh.CityBus(sys)
bus.SetContactMethod(chrono.ChContactMethod_SMC)
bus.SetChassisCollisionType(veh.ChassisCollisionType_MESH)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(0.005)
bus.Initialize()

# Create Rigid Terrain with custom texture
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 200)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()

# Add camera and position it to follow the vehicle
camera_pos = chrono.ChVector3d(0, 3, -6)
vis.AddCamera(camera_pos, bus.GetChassis().GetPos())

# Initialize driver system
driver = veh.VehicleDriver(bus)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()

# Simulation loop parameters
step_timer = chrono.ChRealtimeStepTimer()
time_step = 0.02  # 50 FPS

# Main simulation loop
while vis.Run():
    time = sys.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    # Synchronize vehicle and terrain
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Update camera to follow vehicle
    chassis_pos = bus.GetChassis().GetPos()
    new_camera_pos = chassis_pos + chrono.ChVector3d(0, 3, -6)
    vis.GetSceneManager().getActiveCamera().setPosition(new_camera_pos)
    vis.GetSceneManager().getActiveCamera().setTarget(chassis_pos)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()