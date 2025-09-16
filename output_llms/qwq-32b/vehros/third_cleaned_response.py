import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr
import math

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVectorD(0, 0, 1.6), ch.ChQuaternionD(1, 0, 0, 0)))
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
    
    patch = terrain.AddPatch(patch_mat, ch.ChCoordsysd(), 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    
    box = ch.ChBodyEasyBox(2, 2, 0.5, 1000, True, True)
    box.SetPos(ch.ChVectorD(0, 0, 0.5))
    box.SetBodyFixed(True)
    hmmwv.GetSystem().Add(box)

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    
    vis.AddCamera(ch.ChVectorD(-5, 2.5, 1.5), ch.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVectorD(1.5, -2.5, 5.5), ch.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

    driver = veh.ChDriver(hmmwv)
    driver.Initialize()

    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())

    
    lidar = sens.ChLidarSensor()
    lidar.SetName("lidar")
    lidar.SetPosition(ch.ChVectorD(0, 0, 1.5))
    lidar.SetDirection(ch.ChVectorD(0, 1, 0))
    lidar.SetFOV(90, 90)
    lidar.SetResolution(360, 180)
    lidar.SetRange(0.1, 100)
    lidar.SetSamplingPeriod(1e-3)  
    distance_filter = sens.ChFilterDistance(lidar)
    distance_filter.SetMinDistance(0.1)
    distance_filter.SetMaxDistance(100)
    lidar.AddFilter(distance_filter)
    lidar.AttachTo(hmmwv.GetChassisBody())
    sens_manager.AddSensor(lidar)

    
    ros_lidar_handler = chros.ChROSLidarHandler(25, lidar, "~/output/hmmwv/lidar")
    ros_manager.RegisterHandler(ros_lidar_handler)
    ros_manager.Initialize()

    time_step = 1e-3
    time_end = 30
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    step_number = 0
    hmmwv.EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
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

        if not ros_manager.Update(time, time_step):
            break
        step_number += 1

if __name__ == "__main__":
    main()