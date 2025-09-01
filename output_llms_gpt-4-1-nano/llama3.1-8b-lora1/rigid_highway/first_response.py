import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.Initialize()

    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             terrainLength, 0, 
                             terrainWidth, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
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

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1 for opposite side)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # This is schedule to occur after the vehicle is initialized, so that the
    # visualization window can be brought to the foreground by the user.
    time = 0.0

    # Simulation loop
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(render_step_size)
        terrain.Advance(render_step_size)
        hmmwv.Advance(render_step_size)
        vis.Advance(render_step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Collision type for chassis (PRIMITIVES, MESH, NONE)
chassis_collision_type = veh.COLL_NONE

# Simulation constants
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75) # point tracked by the camera
render_step_size = 1e-3

# Time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1 for opposite side)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Type of engine model
engine_model = veh.EngineModelType_SIMPLE

# Type of transmission model
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS

# Drive type (none, or revolute joints for front/fixed axle)
drive_type = veh.DrivelineTypeWV_Full
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model
tire_model = veh.TireModelType_TMEASY

# Mesh terrain
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction
terrainPatchMat = True; # use different material for each patch?

# =============================================================================

main()