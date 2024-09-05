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

    
    box_size = ch.ChVectorD(1, 1, 1)
    box_pos = ch.ChVectorD(0, 0, 1.5)
    box_body = ch.ChBodyEasyBox(box_size, 1000, True, True, box_pos)
    hmmwv.GetSystem().Add(box_body)

    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())

    
    lidar_sensor = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),  
        10,  
        0.1,  
        0.0,  
        3.14159,  
        0.0,  
        100.0,  
        0.01,  
        True,  
        True  
    )
    lidar_sensor.SetName("lidar_sensor")
    lidar_sensor.SetLag(0.0)
    lidar_sensor.SetFilter(sens.ChLidarFilterGaussian(0.01))
    lidar_sensor.SetFilter(sens.ChLidarFilterRemission(0.01, 0.99))
    lidar_sensor.SetFilter(sens.ChLidarFilterIntensity(0.01, 0.99))
    sens_manager.AddSensor(lidar_sensor)

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar_sensor, "~/output/lidar"))
    ros_manager.Initialize()

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    time = 0
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        if (step_number % render_steps == 0):
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