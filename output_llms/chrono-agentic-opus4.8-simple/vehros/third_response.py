import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())             # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')         # locate vehicle data files

    step_size = 1e-3                                                  # physics step
    init_loc = chrono.ChVector3d(0, 0, 0.5)                          # chassis spawn (HMMWV center origin)
    init_rot = chrono.QuatFromAngleZ(0)                              # facing +X

    hmmwv = veh.HMMWV_Full()                                          # full HMMWV catalog model
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)           # no chassis collision mesh
    hmmwv.SetChassisFixed(False)                                     # MANDATORY — fixed chassis won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # initial pose
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                     # TMEASY tire on rigid road
    hmmwv.SetTireStepSize(step_size)                                # tire substep
    hmmwv.Initialize()                                              # build the vehicle
    system = hmmwv.GetSystem()                                       # take the wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())          # report total vehicle mass

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheel mesh
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tire mesh

    terrain = veh.RigidTerrain(system)                              # flat rigid terrain
    patch_mat = chrono.ChContactMaterialNSC()                       # NSC patch material
    patch_mat.SetFriction(0.9)                                      # tire grip
    patch_mat.SetRestitution(0.01)                                  # nearly inelastic
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 m flat patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                  # patch color
    terrain.Initialize()                                            # build the terrain

    box = chrono.ChBodyEasyBox(1.5, 1.5, 1.5, 1000, True, True, patch_mat)  # visualization box prop
    box.SetPos(chrono.ChVector3d(6, 0, 0.75))                      # ahead of the vehicle, in camera view
    box.SetFixed(True)                                             # static landmark for the lidar
    box.SetName("box")                                            # name for the scene
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))  # red box for visibility
    system.Add(box)                                               # add the box to the world

    manager = sens.ChSensorManager(system)                         # owns all sensors

    offset_pose = chrono.ChFramed(                                # lidar mount on the chassis
        chrono.ChVector3d(1.0, 0, 1.5),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    horizontal_samples = 800                                       # horizontal beams
    vertical_samples = 300                                         # vertical beams
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),                                   # attach to the chassis
        5.0,                                                      # update rate (Hz)
        offset_pose,                                              # offset pose
        horizontal_samples,                                       # h_samples
        vertical_samples,                                         # v_samples
        2 * chrono.CH_PI,                                        # horizontal fov (rad)
        chrono.CH_PI / 12,                                       # max vertical angle
        -chrono.CH_PI / 6,                                       # min vertical angle
        100.0,                                                   # max range (m)
        sens.LidarBeamShape_RECTANGULAR,                         # beam shape
        2,                                                       # sample radius
        0.003,                                                   # vertical divergence
        0.003,                                                   # horizontal divergence
        sens.LidarReturnMode_STRONGEST_RETURN,                   # return mode
    )
    lidar.SetName("Lidar Sensor")                                 # sensor name
    lidar.SetLag(0)                                              # no lag
    lidar.SetCollectionWindow(1.0 / 5.0)                        # collection window = 1/update_rate
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
    lidar.PushFilter(sens.ChFilterDIAccess())                    # host access to depth+intensity
    lidar.PushFilter(sens.ChFilterPCfromDepth())                # depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
    lidar.PushFilter(sens.ChFilterXYZIAccess())                 # host access to XYZI
    manager.AddSensor(lidar)                                     # register the lidar

    driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())         # interactive driver on the vehicle
    render_step_size = 1.0 / 50.0                                # 50 fps render cadence
    driver.SetSteeringDelta(render_step_size / 1.0)             # steering ramp
    driver.SetThrottleDelta(render_step_size / 1.0)            # throttle ramp
    driver.SetBrakingDelta(render_step_size / 0.3)            # braking ramp
    driver.Initialize()                                         # build the driver

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()           # vehicle Irrlicht window
    vis.SetWindowTitle("HMMWV ROS Lidar")                      # window title
    vis.SetWindowSize(1280, 720)                               # window size
    vis.SetChaseCamera(chrono.ChVector3d(-5, 2.5, 1.5), 6.0, 0.5)  # camera viewpoint
    vis.Initialize()                                           # build the device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
    vis.AddSkyBox()                                            # sky box
    vis.AddLightDirectional()                                 # directional light
    vis.AttachVehicle(hmmwv.GetVehicle())                     # bind vehicle visuals

    ros_manager = chros.ChROSPythonManager()                  # ROS2 bridge manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())   # publish /clock FIRST
    ros_manager.RegisterHandler(                              # subscribe driver inputs over ROS
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(                              # publish chassis pose/twist
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(                              # publish lidar scan over ROS
        chros.ChROSLidarHandler(lidar, "~/output/lidar/data/laserscan",
                                chros.ChROSLidarHandlerMessageType_LASER_SCAN))
    ros_manager.Initialize()                                  # initialize ONCE, after all handlers

    render_step_number = math.ceil(render_step_size / step_size)  # steps per rendered frame
    sim_end = 4.0                                            # stop after 4 s of sim time
    realtime_timer = chrono.ChRealtimeStepTimer()            # wall-clock pacing
    step_number = 0                                          # step counter


    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()                          # current sim time

        if step_number % render_step_number == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()                 # read driver inputs

        driver.Synchronize(time)                           # sync driver
        terrain.Synchronize(time)                          # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)   # sync vehicle
        vis.Synchronize(time, driver_inputs)              # sync visualization

        driver.Advance(step_size)                          # advance driver
        terrain.Advance(step_size)                         # advance terrain
        hmmwv.Advance(step_size)                           # advance vehicle (steps the system)
        vis.Advance(step_size)                             # advance visualization

        manager.Update()                                   # pump sensors -> fill buffers


        if not ros_manager.Update(time, step_size):        # publish ROS state LAST
            break

        step_number += 1                                   # advance counter
        realtime_timer.Spin(step_size)                     # spin so wall-clock ~ sim time


if __name__ == "__main__":
    main()
