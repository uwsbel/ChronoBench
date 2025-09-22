import math
import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens                    
from   pychrono import irrlicht as chronoirr


def main():

    
    
    
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    sens.SetChronoDataPath(ch.GetChronoDataPath() + 'sensor/')

    
    
    
    hmmwv = veh.HMMWV_Full()

    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    hmmwv.SetInitPosition(
        ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6),   
                       ch.ChQuaterniond(1, 0, 0, 0))
    )

    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()

    
    hmmwv.SetChassisVisualizationType    (veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType (veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType   (veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType      (veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType       (veh.VisualizationType_MESH)

    system = hmmwv.GetSystem()

    
    
    
    terrain          = veh.RigidTerrain(system)
    patch_material   = ch.ChContactMaterialNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)

    patch = terrain.AddPatch(patch_material, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    
    
    
    box_body = ch.ChBodyEasyBox(
        1.0, 1.0, 1.0,          
        1000,                   
        True,                   
        True,                   
        patch_material)
    box_body.SetPos(ch.ChVector3d(2, 0, 2))
    system.Add(box_body)

    
    
    
    sens_manager = sens.ChSensorManager(system)

    
    sens_manager.scene.AddPointLight(ch.ChVector3f(0, 0, 10), ch.ChColor(1, 1, 1), 800)

    lidar_offset = ch.ChFramed(ch.ChVector3d( 0.8, 0, 1.5),   
                               ch.ChQuaterniond(1, 0, 0, 0))  

    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),     
        10.0,                       
        lidar_offset,               
        640,                        
        32,                         
        math.radians(30.0),         
        math.radians(360.0),        
        100.0)                      

    lidar.SetName("HMMWV_LiDAR")

    
    lidar.PushFilter(sens.ChFilterLidarNoise(0.02))
    lidar.PushFilter(sens.ChFilterLidarXYZReturn())   
    lidar.PushFilter(sens.ChFilterAccess())           

    sens_manager.AddSensor(lidar)

    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV – rigid terrain + LiDAR')
    vis.Initialize()

    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()

    
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))

    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        ch.ChVector3d(1.5, -2.5, 5.5),   
        ch.ChVector3d(0, 0, 0.5),        
        3, 4, 10, 40, 512)

    
    
    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    
    
    ros_manager = chros.ChROSPythonManager()

    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(10, lidar, "~/output/hmmwv/lidar"))

    ros_manager.Initialize()

    
    
    
    time_step          = 1e-3
    render_fps         = 25
    render_step_size   = 1.0 / render_fps
    render_steps       = math.ceil(render_step_size / time_step)
    step_number        = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():

        time = system.GetChTime()

        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        inputs = driver.GetInputs()

        driver.Synchronize (time)
        terrain.Synchronize(time)
        hmmwv.Synchronize  (time, inputs, terrain)

        
        driver.Advance (time_step)
        terrain.Advance(time_step)
        hmmwv.Advance  (time_step)

        
        sens_manager.Update()                       

        
        if not ros_manager.Update(time, time_step):
            break                                   

        step_number += 1



if __name__ == "__main__":
    main()