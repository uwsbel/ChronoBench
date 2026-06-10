import math

import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr


def main():
    veh.SetDataPath(ch.GetChronoDataPath() + "vehicle/")

    
    
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        ch.ChCoordsysd(
            ch.ChVector3d(0, 0, 1.6),
            ch.ChQuaterniond(1, 0, 0, 0)
        )
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    system = hmmwv.GetSystem()

    
    
    
    terrain = veh.RigidTerrain(system)

    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)

    terrain.Initialize()

    
    
    
    
    
    vis_box = ch.ChBodyEasyBox(1.0, 1.0, 1.0, 1000.0, False, True)
    vis_box.SetPos(ch.ChVector3d(8.0, 2.0, 0.5))
    vis_box.SetFixed(True)
    system.Add(vis_box)

    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV - Rigid terrain with lidar")
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()

    
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))

    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        ch.ChVector3d(1.5, -2.5, 5.5),
        ch.ChVector3d(0, 0, 0.5),
        3,
        4,
        10,
        40,
        512,
    )

    
    
    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    
    
    sens_manager = sens.ChSensorManager(system)

    lidar_update_rate = 10.0
    lidar_horizontal_samples = 1024
    lidar_vertical_samples = 32
    lidar_horizontal_fov = 2.0 * math.pi
    lidar_max_vert_angle = math.radians(15.0)
    lidar_min_vert_angle = math.radians(-15.0)
    lidar_max_distance = 100.0

    
    lidar_pose = ch.ChFramed(
        ch.ChVector3d(0.8, 0.0, 1.8),
        ch.ChQuaterniond(1, 0, 0, 0)
    )

    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),
        lidar_update_rate,
        lidar_pose,
        lidar_horizontal_samples,
        lidar_vertical_samples,
        lidar_horizontal_fov,
        lidar_max_vert_angle,
        lidar_min_vert_angle,
        lidar_max_distance,
    )

    lidar.SetName("HMMWV_Lidar")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(1.0 / lidar_update_rate)

    
    
    
    
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(
        sens.ChFilterVisualizePointCloud(
            640,
            480,
            1.0,
            "Lidar Point Cloud"
        )
    )
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    sens_manager.AddSensor(lidar)

    
    
    
    ros_manager = chros.ChROSPythonManager()

    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(
            25,
            driver,
            "~/input/driver_inputs"
        )
    )

    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(
            25,
            hmmwv.GetChassisBody(),
            "~/output/hmmwv/state"
        )
    )

    
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(
            lidar_update_rate,
            lidar,
            "~/output/hmmwv/lidar/pointcloud"
        )
    )

    ros_manager.Initialize()

    
    
    
    time_step = 1e-3
    time_end = 30.0

    render_step_size = 1.0 / 25.0
    render_steps = math.ceil(render_step_size / time_step)
    step_number = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = system.GetChTime()

        if time >= time_end:
            break

        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        
        sens_manager.Update()

        
        ros_time = system.GetChTime()
        if not ros_manager.Update(ros_time, time_step):
            break

        step_number += 1


if __name__ == "__main__":
    main()