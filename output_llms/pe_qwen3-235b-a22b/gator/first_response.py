import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))

# Create the physical system
sys = chrono.ChSystemSMC()

# Configure contact material for terrain
terrain_mat = chrono.ChContactMaterialSMC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)
terrain_mat.SetYoungModulus(2e7)

# Create flat rigid terrain
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(terrain_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        200, 100)  # Length and width in meters
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))
terrain.Initialize()

# Create Gator vehicle
gator = veh.Gator(sys)
gator.SetContactMethod(chrono.ChContactMethod_SMC)
gator.SetTireType(veh.TIRE_MODEL_TMEASY)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QuatFromAngleZ(0)))
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.Initialize()

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, -10))
vis.AddTypicalLights()

# Create interactive driver
driver = veh.InteractiveDriverIRR(vis)
driver.SetVehicle(gator.GetVehicle())
driver.SetSteeringDelta(0.06)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# Real-time step control
realtime_timer = chrono.ChRealtimeStepTimer()
step_size = 1.0 / 50  # 50 FPS simulation rate

# Simulation loop
while vis.Run():
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Synchronize vehicle and driver
    time = sys.GetChTime()
    gator.Synchronize(time, driver_inputs, terrain)
    driver.Synchronize(time)
    
    # Advance simulation
    sys.DoStepDynamics(step_size)
    
    # Update driver
    driver.Advance(step_size)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time execution
    realtime_timer.Spin(step_size)