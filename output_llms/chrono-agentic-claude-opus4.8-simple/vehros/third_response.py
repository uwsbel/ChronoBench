import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core data path
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data path

    hmmwv = veh.HMMWV_Full()                                             # full HMMWV model
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # rigid-terrain default
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
    hmmwv.SetChassisFixed(False)                                        # chassis must be free to move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                      # engine type
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # transmission type
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                          # all-wheel drive
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)                 # steering type
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tire model
    hmmwv.SetTireStepSize(1e-3)                                          # tire integration step
    hmmwv.Initialize()                                                   # build the vehicle

    system = hmmwv.GetSystem()                                          # the wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required with contact
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # diagnostic banner

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)   # chassis vis
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension vis
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)  # steering vis
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)         # wheel vis
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)    # tire vis

    terrain = veh.RigidTerrain(system)                                  # rigid terrain on the shared system
    patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
    patch_mat.SetFriction(0.9)                                         # terrain friction
    patch_mat.SetRestitution(0.01)                                    # terrain restitution
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 100)    # 100x100 flat patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()                                              # finalize terrain

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)           # visualization box (no collision)
    box.SetPos(chrono.ChVector3d(10, 0, 0.5))                        # placed ahead of the vehicle, in lidar view
    box.SetFixed(True)                                              # static obstacle for the lidar to see
    system.Add(box)                                                # add the box to the system

    driver = veh.ChDriver(hmmwv.GetVehicle())                         # base driver, fed over ROS
    driver.Initialize()                                               # initialize driver

    sens_manager = sens.ChSensorManager(system)                       # sensor manager on the shared system
    offset_pose = chrono.ChFramed(                                     # lidar mounted forward/up on the chassis
        chrono.ChVector3d(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),                                       # attached to the chassis
        5.0,                                                         # update_rate (Hz)
        offset_pose,                                                 # offset pose
        800,                                                         # horizontal samples
        300,                                                         # vertical samples
        2 * chrono.CH_PI,                                            # horizontal fov
        chrono.CH_PI / 12,                                           # max vertical angle
        -chrono.CH_PI / 6,                                           # min vertical angle
        100.0,                                                       # max range
        sens.LidarBeamShape_RECTANGULAR,                             # beam shape
        2,                                                           # sample radius
        0.003,                                                       # vertical divergence angle
        0.003,                                                       # horizontal divergence angle
        sens.LidarReturnMode_STRONGEST_RETURN,                       # return mode
    )
    lidar.SetName("Lidar Sensor")                                    # name
    lidar.SetLag(0)                                                  # no lag
    lidar.SetCollectionWindow(1.0 / 5.0)                             # collection window = 1/update_rate
    lidar.PushFilter(sens.ChFilterDIAccess())                        # host access to depth+intensity
    lidar.PushFilter(sens.ChFilterPCfromDepth())                     # depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                      # host access to XYZI point cloud
    sens_manager.AddSensor(lidar)                                    # register the lidar

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht visual system
    vis.SetWindowTitle('HMMWV over ROS')                              # window title
    vis.SetWindowSize(1280, 1024)                                     # window size
    vis.SetChaseCamera(chrono.ChVector3d(-5, 2.5, 1.5), 6.0, 0.5)     # chase camera (prompt perspective)
    vis.Initialize()                                                  # initialize device first
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo (after Initialize)
    vis.AddLightDirectional()                                         # vehicle scene directional light
    vis.AddSkyBox()                                                   # sky box
    vis.AttachVehicle(hmmwv.GetVehicle())                             # bind the vehicle to the visual system

    ros_manager = chros.ChROSPythonManager()                          # python ROS manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())            # /clock first
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))  # SUBSCRIBE throttle/steer/brake
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))  # publish chassis state
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud",
                                                        chros.ChROSLidarHandlerMessageType_LASER_SCAN))  # publish lidar scan
    ros_manager.Initialize()                                          # initialize once, after registration

    step_size = 1e-3                                                  # integration step
    render_step_size = 1.0 / 50.0                                     # 50 fps render cadence
    render_steps = math.ceil(render_step_size / step_size)            # physics steps per rendered frame

    step_number = 0
    while vis.Run():                                                 # real-time render loop
        time = system.GetChTime()                                   # current sim time

        if step_number % render_steps == 0:                         # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()                          # inputs (set by ROS subscriber)

        driver.Synchronize(time)                                    # synchronize driver
        terrain.Synchronize(time)                                   # synchronize terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)             # synchronize vehicle
        vis.Synchronize(time, driver_inputs)                        # synchronize visualization

        driver.Advance(step_size)                                   # advance driver
        terrain.Advance(step_size)                                  # advance terrain
        hmmwv.Advance(step_size)                                    # advance vehicle (steps the system)
        vis.Advance(step_size)                                      # advance visualization

        sens_manager.Update()                                       # pump sensors (fills lidar buffers)

        if not ros_manager.Update(time, step_size):                 # publish to ROS, break on shutdown
            break

        step_number += 1                                            # frame/step counter

if __name__ == "__main__":
    main()
