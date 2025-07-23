import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import math
import pychrono.sensor as sens

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))
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

    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    box = ch.ChBodyEasyBox(0.5, 0.5, 0.5, 1000)
    box.SetPos(ch.ChVector3d(0, 0, 0.25))
    box.SetFixed(True)
    box.GetVisualShape(0).SetMaterial(0, ch.ChVisualMaterialNSC())
    hmmwv.GetChassis().AddChild(box)

    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),
        5,              # Hz
        450,            # rpm
        360,            # scan lines
        -ch.CH_PI_2,    # vertical offset
        ch.CH_2PI,      # vertical fov
        -ch.CH_PI,      # horizontal offset
        ch.CH_2PI,      # horizontal fov
        100.0,          # max lidar range
        0.01,           # sample radius
        0.1             # lidar divergence angle
    )
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(480, 480, 1))
    sens_manager.AddSensor(lidar)

    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/hmmwv/lidar"))
    ros_manager.Initialize()

    time = 0
    time_step = 1e-3
    time_end = 30
    hmmwv.GetVehicle().EnableRealtime(True)
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
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
        if not ros_manager.Update(time, time_step):
            break

if __name__ == "__main__":
    main()