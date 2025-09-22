import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetEngineType(engine_type)
    hmmwv.SetTransmissionType(transmission_type)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize(chrono.ChContactMaterialDefault, chrono.ChVector3d(0, 0, -0.0), chrono.CH_TIME_STEP, chrono.CH_TIME_STEP_TIRE);

    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, -1.0), chrono.QUNIT))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockSynchronizer(25))
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetVehicle().GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()

    # Simulation loop
    hmmwv.GetVehicle().EnableRealtime(True)
    while (True):
        time = hmmwv.GetSystem().GetChTime()

        # Advance vehicle and terrain
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver)
        hmmwv.GetVehicle().Synchronize(time)

        # Update ROS manager
        ros_manager.Synchronize(time)
        ros_manager.Update()

        # Advance to next time
        driver.Advance(chrono.ChTime(timestep))
        terrain.Advance(chrono.ChTime(timestep))
        hmmwv.Advance(chrono.ChTime(timestep))
        ros_manager.Advance(chrono.ChTime(timestep))

        # End simulation
        if (time >= t_end):
            break

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically generated based on the location of the example script being executed.
# Typically, you might not need to modify this line.
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle position
init_pos = chrono.ChVector3d(0, -1, 0.5)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_PRIMITIVES

# Contact method
contact_method = chrono.ChContactMethod_SMC
#contact_method = chrono.ChContactMethod_NSC

# Engine type
engine_type = veh.EngineModelType_SIMPLE_MAP
#engine_type = veh.EngineModelType_SHAFTS

# Transmission type
transmission_type = veh.TransmissionModelType_AUTOMATIC_SHAFTS

# Drive type
drive_type = veh.DrivelineTypeWV_AWD

# Steering type
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Tire model (ROLLING, RIGID, PAC02)
tire_model = veh.TireModelType_TMEASY

# Simulation step sizes
tire_step_size = 1e-3

# Simulation timestep
timestep = 1e-3

# Simulation end time
t_end = 1000

main()