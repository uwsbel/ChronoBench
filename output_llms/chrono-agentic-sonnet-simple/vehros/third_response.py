import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import pychrono.sensor as sens

def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

    # simulation parameters
    step_size = 1e-3                                                  # physics time step
    sim_end = 10.0                                                    # simulation end time
    render_fps = 25.0                                                 # target render frame rate (25 FPS as truth)
    terrainLength = 100.0                                             # terrain patch length
    terrainWidth = 100.0                                              # terrain patch width

    # HMMWV setup
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)                                      # MANDATORY — fixed won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                   # shaft-based engine model
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic transmission
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                       # all-wheel drive
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)              # pitman arm steering
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire model
    hmmwv.SetTireStepSize(step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # truth's literal banner

    # Visualization box on terrain
    box = chrono.ChBodyEasyBox(3, 3, 6, 1000)                        # static box for visual reference
    box.SetPos(chrono.ChVector3d(10.0, 0.0, 0))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    hmmwv.GetSystem().Add(box)

    # Rigid terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # Irrlicht visualization (generic window, not vehicle-specific)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-5, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # camera at (-5,2.5,1.5)
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Driver system (truth uses ChDriver, not interactive)
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(2.0, 0, 2),
        chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)),
    )
    # Lidar sensor (90 h_samples x 300 v_samples — truth-exact)
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),                                       # attached to chassis
        5.0,                                                          # update rate (Hz)
        offset_pose,                                                  # offset pose
        90,                                                           # horizontal samples
        300,                                                          # vertical samples
        2 * chrono.CH_PI,                                            # horizontal FOV (full 360)
        chrono.CH_PI / 12,                                            # max vertical angle
        -chrono.CH_PI / 6,                                           # min vertical angle
        100.0,                                                        # max range
        0,                                                            # beam_shape (0 = RECTANGULAR)
    )
    lidar.PushFilter(sens.ChFilterDIAccess())                         # access raw lidar depth+intensity
    lidar.PushFilter(sens.ChFilterPCfromDepth())                      # convert depth to point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                       # access XYZI point cloud (scored)
    lidar.SetName("lidar")
    sens_manager.AddSensor(lidar)

    # ROS manager — handlers are SCORED CORE
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())            # clock handler FIRST
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
    ros_manager.Initialize()                                          # initialize AFTER all handlers

    render_every = max(1, round(1.0 / (render_fps * step_size)))     # untagged cadence constant
    step_number = 0

    while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
        time = hmmwv.GetSystem().GetChTime()

        if step_number % render_every == 0:                          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        sens_manager.Update()                                         # pump sensors — once per step

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)                                      # advances the wrapper-owned system

        if not ros_manager.Update(time, step_size):                   # publish to ROS; break on shutdown
            break


        step_number += 1


if __name__ == "__main__":
    main()
