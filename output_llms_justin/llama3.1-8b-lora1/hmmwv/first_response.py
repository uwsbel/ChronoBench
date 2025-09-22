import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create systems

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
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
                             chrono.ChCoordsysd(chrono.VNULL, chrono.QUNIT), 
                             terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile(terrainTexture), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # This is schedule to occur after all other initialization
    # (terrain, etc.)
    # Set simulation step sizes
    step_size = 1e-3
    timestep = chrono.ChStepSizeType(step_size / render_step_size)

    # Simulation loop
    while vis.Run() :
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = hmmwv.GetSystem().GetChTime()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(timestep)
        terrain.Advance(timestep)
        hmmwv.Advance(timestep)
        vis.Advance(timestep)

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 1, 4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Collision type for chassis
chassis_collision_type = veh.COLL_CHASSIS_NONE

# Type of engine model
engine_model = veh.ENGINE_MODEL_TYPE
if engine_model == veh.ENGINE_MODEL_SIMPLE:
    engine_file = ""
elif engine_model == veh.ENGINE_MODEL_SHAFTS:
    engine_file = veh.GetDataFile("engines/MTU-604Z-8A.json")

# Type of transmission model
transmission_model = veh.TRANMISSION_MODEL_SIMPLE
if transmission_model == veh.TRANMISSION_MODEL_AUTOMATIC_SHAFTS:
    transmission_file = veh.GetDataFile("transmissions/AZ89-5.json")

# Drive type
drive_type = veh.DRIVE_RWD

# Steering type
steering_type = veh.STEERING_PITMAN_ARM

# Type of tire model
tire_model = veh.TIRES_MODEL_TMEASY
tire_file = ""
if tire_model == veh.TIRES_MODEL_TMEASY:
    tire_file = veh.GetDataFile("tires/TMEasyF/TF_604x32_TMEasyF.json")

# Rigid terrain
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction
terrainTexture = "tires/tile1.jpg"; # texture file

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
timestep = chrono.ChStepSizeType(step_size)

# Visualization type
vis_type = veh.CH_VIS_IRRITCHL

# Render step size (or simulation fps)
render_step_size = 1.0 / 50

main()