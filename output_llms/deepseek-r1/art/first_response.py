import os
import math
import time
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================
# Simulation Parameters
# =============================================================================
CONTACT_METHOD = chrono.ChContactMethod_SMC
STEP_SIZE = 0.005
VISUALIZATION_FPS = 50  # Target frames per second for visualization

# =============================================================================
# Initialize Simulation System
# =============================================================================
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# =============================================================================
# Create and Configure ARTcar Vehicle
# =============================================================================
initial_pos = chrono.ChVectorD(0, 0, 0.5)
initial_rot = chrono.Q_ROTATE_Y_TO_Z  # Default orientation for ground vehicles

artcar = veh.ARTVehicle(
    system,
    initial_pos,
    initial_rot,
    contact_method=CONTACT_METHOD,
    visualization_type=veh.VisualizationType_MESH
)
artcar.Initialize()

# =============================================================================
# Rigid Terrain Setup
# =============================================================================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, 1),
    100.0,  # Terrain length
    100.0   # Terrain width
)
patch.SetTexture(veh.GetChronoDataFile("terrain/textures/concrete.jpg"), 100, 100)
terrain.Initialize()

# =============================================================================
# Interactive Driver System
# =============================================================================
driver = veh.InteractiveDriver(artcar.GetVehicle())
artcar.GetVehicle().SetDriver(driver)

# =============================================================================
# Irrlicht Visualization Setup
# =============================================================================
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("ARTcar Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1.5))
vis.AddTypicalLights()

# Link driver to visualization event system
driver.SetInputDataFile(veh.GetDataFile("driver/InputData.txt"))
driver.Initialize()
vis.AddUserEventReceiver(driver.GetInputReceiver())

# =============================================================================
# Simulation Loop with Real-Time Control
# =============================================================================
frame_interval = 1.0 / VISUALIZATION_FPS
last_frame_time = time.time()

while vis.Run():
    # Control visualization frame rate
    current_time = time.time()
    if (current_time - last_frame_time) < frame_interval:
        time.sleep(frame_interval - (current_time - last_frame_time))
    last_frame_time = current_time

    # Advance simulation
    time = system.GetChTime()
    driver.Synchronize(time)
    artcar.Synchronize(time)
    system.DoStepDynamics(STEP_SIZE)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Draw GUI elements
    irr.ChIrrTools.drawAllStats(
        vis,
        artcar.GetVehicle().GetSteeringController().GetControlModeString(),
        True
    )
    irr.ChIrrTools.drawGUI(vis)

# Properly clean up resources
vis.GetDevice().closeDevice()