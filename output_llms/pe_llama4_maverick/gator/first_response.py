import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np

# =============================================================================
# Setup
# =============================================================================

# Output directory
out_dir = chrono.GetChronoOutputPath() + "GATOR"

# Contact method (NSC or SMC)
contact_method = chrono.ChContactMethod_NSC

# Simulation time step
time_step = 1e-3

# Terrain dimensions
terrainLength = 100.0  # length (X direction)
terrainWidth = 100.0  # width (Y direction)

# Vehicle initial location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # rotation about Y axis

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_MESH
steering_vis_type = veh.VisualizationType_MESH
wheel_vis_type = veh.VisualizationType_MESH

# Type of tire model (RIGID, TMEASY, PAC89, or PAC02)
tire_model = veh.TireModelType_TMEASY

# =============================================================================
# Create the vehicle system
# =============================================================================

# Create the Gator vehicle
gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisVisType(chassis_vis_type)
gator.SetSuspensionVisType(suspension_vis_type)
gator.SetSteeringVisType(steering_vis_type)
gator.SetWheelVisType(wheel_vis_type)
gator.SetTireType(tire_model)
gator.Initialize(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireStepSize(time_step)

# Create the terrain
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(gator.GetSystem())
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -10, 2))
vis.AddTypicalLights()

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3  # time to go from 0 to +1
driver.SetSteeringDelta(driver.GetSteeringMax() / steering_time * time_step)
driver.SetThrottleDelta(driver.GetThrottleMax() / throttle_time * time_step)
driver.SetBrakingDelta(driver.GetBrakingMax() / braking_time * time_step)

# Simulation loop
while vis.Run():
    time = gator.GetSystem().GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()
    gator.SetDriverInputs(driver_inputs)

    # Update modules (process inputs, advance state)
    driver.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    # Advance simulation for one timestep for all modules
    driver.Advance(time_step)
    gator.Advance(time_step)
    terrain.Advance(time_step)

    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Cap the frame rate
    vis.WaitForEndScene(20)

# Cleanup
vis.GetDevice().closeDevice()