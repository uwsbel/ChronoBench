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
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)
    vis.AddMesh(ch.GetChronoDataFile("vehicle/hmmwv/objects/box.obj"))
    box = ch.ChBodyEasyBox(2, 2, 2, 1000, False, False)
    box.SetPos(ch.ChVector3d(5, 0, 1))
    box.SetFixed(False)
    box.Initialize()
    hmmwv.GetSystem().Add(box)

    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()  

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  
    
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()  

    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())

    
    offset_pose = ch.ChFramed(ch.ChVector3d(-12, 0, 2), ch.QUNIT)
    lidar_sample = sens.ChLidarSample()
    lidar_sample.SetName("Lidar Sample")
    lidar_sample.SetLidarPeriod(0.1)
    lidar_sample.SetOffsetPose(offset_pose)
    lidar_sample.SetLidarResolution(1280, 20)
    lidar_sample.SetMaxAngle(2 * ch.CH_PI)
    lidar_sample.SetLag(0)
    lidar_sample.SetCollectionMethod(sens.ChSampleFilterFilterCollection())
    sens_manager.AddSensor(lidar_sample)
    lidar_data = sens.ChLidarSensorData(1280, 20)
    lidar_sample.SetLidarData(lidar_data)
    lidar_sample.AddFilter(sens.ChFilterLidarNoiseGaussian(0.08, 0.01))
    lidar_sample.AddFilter(sens.ChFilterLidarFoV(0, 2 * ch.CH_PI, -ch.CH_PI / 4, ch.CH_PI / 4))
    lidar_sample.AddFilter(sens.ChFilterLidarDepth(0, 80))
    lidar_sample.AddFilter(sens.ChFilterLidarReturn(sens.ChFilterLidarReturnMode_RETURN_STRONGEST))
    lidar_sample.SetOutputDirectory(ch.GetChronoDataFile("SENSOR_DATA/"))

    
    lidar = sens.ChLidarSensor(hmmwv.GetChassisBody(),            
                               update_rate,                      
                               lidar_data,                       
                               offset_pose,                      
                               return_mode,                      
                               update_rate,                      
                               min_angle,                        
                               max_angle,                        
                               min_vert_angle,                   
                               max_vert_angle,                   
                               1.0,                              
                               max_vert_bins,                    
                               max_range)                        
    sens_manager.AddSensor(lidar)

    
    graph = sens.ChFilterGraph()
    graph.AddFilter(sens.ChFilterPCA(0, 2))
    graph.AddFilter(sens.ChFilterPlaneRANSAC(0.15, 200, 0.95))
    lidar.PushFilterGraph(graph)

    
    lidar_handler = chros.ChROSLidarHandler(25, lidar, "~/output/hmmwv/lidar", 0.0)
    ros_manager.RegisterHandler(lidar_handler)

    
    time = 0
    time_step = 1e-3  
    time_end = 30  
    
    
    time_step = 1e-3
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