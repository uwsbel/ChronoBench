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
    
    
    box_body = ch.ChBodyEasyBox(2, 2, 1,  
                                1000,     
                                True,     
                                True)     
    box_body.SetPos(ch.ChVector3d(5, 0, 0.5))  
    box_body.SetFixed(True)  
    box_body.GetVisualShape(0).SetColor(ch.ChColor(0.8, 0.2, 0.2))  
    hmmwv.GetSystem().Add(box_body)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV Vehicle Simulation with Sensors')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))  
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    
    sens_manager.scene.AddPointLight(ch.ChVector3d(0, 0, 100), ch.ChColor(1, 1, 1), 1000)

    
    lidar_offset = ch.ChFramed(ch.ChVector3d(0.5, 0, 0.5),  
                             ch.ChQuaterniond(1, 0, 0, 0))  
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),  
        30,                      
        lidar_offset,            
        1800,                    
        16,                      
        ch.CH_PI,                
        ch.CH_PI / 6,            
        100.0                    
    )
    lidar.SetName("Vehicle Lidar")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.02)

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())  
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))  
    lidar.PushFilter(sens.ChFilterVisualize(640, 360, "Lidar Data"))  
    lidar.PushFilter(sens.ChFilterXYZIAccess())  
    sens_manager.AddSensor(lidar)

    
    ros_manager.RegisterHandler(sens.ChROSLidarHandler(lidar, "~/output/lidar/pointcloud"))
    ros_manager.Initialize()

    
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25  
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

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