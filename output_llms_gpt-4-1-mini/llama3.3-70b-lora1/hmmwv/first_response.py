import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.7)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 3e-3
tire_step_size = step_size

# Simulation end time
tend = 15

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = "./HMMWV"


# =============================================================================


print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

# Create the HMMWV vehicle, set parameters, and initialize
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.SetDriveline8WD(True)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetSteeringVisType(steering_vis_type)
hmmwv.SetSuspensionVisType(suspension_vis_type)
hmmwv.SetWheelVisType(wheel_vis_type)
hmmwv.SetTireVisType(tire_vis_type)
hmmwv.SetChassisVisType(chassis_vis_type)
hmmwv.SetChassisFixed(False)
hmmwv.Initialize()

hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = hmmwv.GetSystem().GetChTime()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Collect output data from modules (for inter-module communication)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)