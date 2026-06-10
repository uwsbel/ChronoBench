import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens                                          # lidar sensor module
import pychrono.ros as chros


def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())               # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')           # locate vehicle data files

    init_loc = chrono.ChVector3d(0, 0, 0.5)                            # chassis spawn (origin above road)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                        # identity orientation
    step_size = 1e-3                                                   # integration step (s)

    hmmwv = veh.HMMWV_Full()                                           # full HMMWV catalog wrapper
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision mesh
    hmmwv.SetChassisFixed(False)                                       # chassis must be free to move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # initial pose
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire model
    hmmwv.SetTireStepSize(step_size)                                  # tire integration step
    hmmwv.Initialize()                                                # build the vehicle subsystems
    system = hmmwv.GetSystem()                                        # the wrapper-owned ChSystem
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())            # report total vehicle mass

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh chassis
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # mesh wheels
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # mesh tires

    terrain = veh.RigidTerrain(system)                                # flat rigid terrain
    patch_mat = chrono.ChContactMaterialNSC()                         # NSC patch material
    patch_mat.SetFriction(0.9)                                        # tire grip
    patch_mat.SetRestitution(0.01)                                    # nearly inelastic ground
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 m flat patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # sandy color
    terrain.Initialize()                                              # finalize terrain

    box_mat = chrono.ChContactMaterialNSC()                           # box contact material
    box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, True, box_mat)  # 1 m visualization cube
    box.SetPos(chrono.ChVector3d(10, 0, 0.5))                        # placed ahead of the vehicle
    box.SetFixed(True)                                                # static obstacle the lidar sees
    system.Add(box)                                                   # add to the shared system

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht window
    vis.SetWindowTitle("HMMWV ROS Lidar")                            # window title
    vis.SetWindowSize(1280, 1024)                                     # window resolution
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)      # chase camera on chassis
    vis.Initialize()                                                 # create device first
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo overlay
    vis.AddSkyBox()                                                   # sky backdrop
    vis.AddLightDirectional()                                        # directional light (vehicle truth)
    vis.AddCamera(chrono.ChVector3d(-5, 2.5, 1.5))                   # extra viewpoint
    vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visuals

    driver = veh.ChInteractiveDriverIRR(vis)                         # interactive driver bound to vis
    driver.SetSteeringDelta(0.02)                                    # steering ramp per step
    driver.SetThrottleDelta(0.02)                                    # throttle ramp per step
    driver.SetBrakingDelta(0.06)                                     # braking ramp per step
    driver.Initialize()                                              # finalize driver

    manager = sens.ChSensorManager(system)                          # owns all sensors

    offset_pose = chrono.ChFramed(                                   # lidar mount on the chassis
        chrono.ChVector3d(0, 0, 1.5),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),                                     # ride on the chassis
        5.0,                                                        # update_rate (Hz)
        offset_pose,                                                # offset pose
        800,                                                        # horizontal samples
        300,                                                        # vertical samples
        2 * chrono.CH_PI,                                           # horizontal FOV (rad)
        chrono.CH_PI / 12,                                          # max vertical angle
        -chrono.CH_PI / 6,                                          # min vertical angle
        100.0,                                                      # max range (m)
        sens.LidarBeamShape_RECTANGULAR,                           # rectangular beam
        2,                                                          # sample radius
        0.003,                                                      # vertical divergence angle
        0.003,                                                      # horizontal divergence angle
        sens.LidarReturnMode_STRONGEST_RETURN,                     # strongest-return mode
    )
    lidar.SetName("Lidar Sensor")                                   # name for ROS topic
    lidar.SetLag(0)                                                 # no lag
    lidar.SetCollectionWindow(1.0 / 5.0)                           # collection window = 1/update_rate
    lidar.PushFilter(sens.ChFilterDIAccess())                      # host access to depth+intensity
    lidar.PushFilter(sens.ChFilterPCfromDepth())                  # depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                   # host access to XYZI cloud
    manager.AddSensor(lidar)                                       # register lidar with the manager

    ros_manager = chros.ChROSPythonManager()                       # ROS2 bridge manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())         # /clock first
    driver_handler = chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")  # subscribe inputs
    ros_manager.RegisterHandler(driver_handler)                    # register driver handler
    body_handler = chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state")  # publish pose
    ros_manager.RegisterHandler(body_handler)                      # register body handler
    lidar_handler = chros.ChROSLidarHandler(lidar, "~/output/lidar/data")  # publish point cloud
    ros_manager.RegisterHandler(lidar_handler)                     # register lidar handler
    ros_manager.Initialize()                                       # initialize once, after all handlers

    render_step_size = 1.0 / 50.0                                  # render cadence (50 fps)
    render_every = max(1, round(render_step_size / step_size))    # physics steps per frame

    sim_end = 12.0                                                # simulation duration (s)
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = system.GetChTime()                            # current sim time
            driver_inputs = driver.GetInputs()                   # current driver command

            driver.Synchronize(time)                             # update driver
            terrain.Synchronize(time)                            # update terrain
            hmmwv.Synchronize(time, driver_inputs, terrain)      # update vehicle
            vis.Synchronize(time, driver_inputs)                 # update vis HUD

            driver.Advance(step_size)                            # advance driver
            terrain.Advance(step_size)                           # advance terrain
            hmmwv.Advance(step_size)                             # advances the wrapper-owned system
            vis.Advance(step_size)                               # advance vis

            manager.Update()                                     # pump sensors -> fills lidar buffer

            if not ros_manager.Update(time, step_size):          # publish state to ROS, break on shutdown
                break
            if system.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
