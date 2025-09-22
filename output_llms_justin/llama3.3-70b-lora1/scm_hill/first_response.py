import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math


#// =============================================================================

def main():
    print("Copyright (c) 2017 projectchrono.org" + "\n\n")

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
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    # Create the terrain

    # create the SCM deformable terrain
    terrain = veh.SCMTerrain(hmmwv.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,
                             round(terrainLength / mesh_length), round(terrainWidth / mesh_length))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

    # set terrain parameters
    terrain.SetGrain(0.02, 0.00, 0.02);
    terrain.SetRod(0.02, 0.00, 0.02)
    # myterrain.SetRod(0.06,0.00,0.06)
    terrain.SetXmin(-200.0)
    terrain.SetXmax(200.0)
    terrain.SetYmin(-100.0)
    terrain.SetYmax(300.0)

    # create the height map
    # int w = 256;
    # int h = 128;
    # float2* H = new float2[w * h];
    # for (int i = 0; i < h; i++)
    # {
    #     for (int j = 0; j < w; j++) {
    #         double x = -200 + j * 400.0 / (w - 1);  // from -200 to + 200
    #         double y = -100 + i * 400.0 / (h - 1);  // from -100 to +300
    #         double zh = 2 * sin(4*x*3.14/400)*sin(2*y*3.14/400);
    #         H[i * w + j] = float2(x, y);
    #     }
    # }
    # height = myterrain.GetHeight();
    # for (int i = 0; i < h; i++)
    # {
    #     for (int j = 0; j < w; j++) {
    #         double x = H[i * w + j].x;
    #         double y = H[i * w + j].y;
    #         double zh =  2 * sin(4*x*3.14/400)*sin(2*y*3.14/400);
    #         height.AddPoint(x, y, zh);
    #     }
    # }
    # delete[] H;

    # or simply
    height = terrain.GetHeight()
    for i in range(-200, 200, 1):
        for j in range(-100, 300, 1):
            # zh = 2 * sin(4*i*3.14/400)*sin(2*j*3.14/400);
            zh = 0
            if (abs(i) < 75):
                if (j > 100):
                    zh = -8 + (j - 100) / 12.0
                if (j < -50):
                    zh = +10 - (j + 50) / 7.0
            height.AddPoint(i, j, zh)

    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV-SCM')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.2), 6.0, 0.5)
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

    # Set the iteration loop
    driver.Initialize()

    # Simulation loop

    # number of simulation steps between miscellaneous events
    render_steps = m.ceil(render_step_size / step_size)
    debug_steps = m.ceil(debug_step_size / step_size)

    # Initialize simulation frame counter and simulation time
    step_number = 0
    render_frame = 0

    if (contact_vis):
        vis.SetSymbolscale(1.0)
    # Run the main simulation loop. In this case we will get driver inputs from the Irrlicht interface.
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        # Render scene and output POV-Ray data
        if (step_number % render_steps == 0) :
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
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        # Increment frame number
        step_number += 1

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of SCMTerrain terrain (car section)
terrainHeight = 0      # terrain height in SCMTerrain section
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 3e-3;
tire_step_size = 1e-3;

# Time interval between two render frames
render_step_size = 1.0 / 50;  # FPS = 50

# Output directories
out_dir = os.path.join(os.path.dirname(__file__), "HMMWV")
out_dir1 = os.path.join(out_dir, "vehicle")
out_dir2 = os.path.join(out_dir, "driver")

# POV-Ray output
povray = True

# =============================================================================


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of SCMTerrain terrain (car section)
terrainHeight = 0      # terrain height in SCMTerrain section
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 3e-3;
tire_step_size = 1e-3;

# Time interval between two render frames
render_step_size = 1.0 / 50;  # FPS = 50

# Output directories
out_dir = os.path.join(os.path.dirname(__file__), "HMMWV")
out_dir1 = os.path.join(out_dir, "vehicle")
out_dir2 = os.path.join(out_dir, "driver")

# POV-Ray output
povray = True

# Engine type: SHAFTS, SIMPLE
engine_model = veh.EngineModelType_SHAFTS
# Drive type: FWD, RWD, or AWD
drive_type = veh.DrivelineTypeWV_AWD
# Braking type: PROPORTIONAL or TESTBRACKING
braking_type = veh.BrakingTypeWV_PROPORTIONAL
# Differential type: NONE or OPEN
diff_type = veh.DifferentialTypeWV_OPEN
# Transmission type: AUTOMATIC, MANUAL, or SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC
steering_type = veh.SteeringTypeWV_PITMAN_ARM
# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Optional: specify the JSON specification file for the vehicle
json_spec = veh.GetDataFile("full_car/vehicle/HMMWV_JSON.json")

# -----------------
main()