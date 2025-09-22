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
    
    patch = terrain.AddPatch(patch_mat, ch.ChCoordsysd(ch.ChVector3d(0,0,0), ch.QUNIT), 200.0, 200.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 
    terrain.Initialize()  

    
    
    box_size = ch.ChVector3d(1.0, 1.0, 1.0)  
    
    box_pos = ch.ChVector3d(5, 2, box_size.z / 2 + 1e-3)
    
    box_body = ch.ChBody() 
    box_body.SetPos(box_pos)
    box_body.SetFixed(True) 

    
    box_coll_shape = ch.ChCollisionShapeBox(patch_mat, box_size.x / 2, box_size.y / 2, box_size.z / 2)
    box_body.AddCollisionShape(box_coll_shape)
    box_body.EnableCollision(True) 

    
    box_visual_shape = ch.ChVisualShapeBox(box_size.x, box_size.y, box_size.z)
    try:
        
        box_visual_shape.SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    except Exception as e:
        print(f"Warning: Could not set texture for visualization box: {e}. Using default color.")
    box_body.AddVisualShape(box_visual_shape)
    
    hmmwv.GetSystem().Add(box_body) 

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z) 
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV with Lidar, ROS, and Visualization Box') 
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    
    vis.AddCamera(ch.ChVector3d(-8, 4, 2.5), ch.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    
    
    try:
        sens_manager.SetRaycastingMethod(sens.RendersharpEngine.IRRSHADER, 4) 
    except Exception as e:
        print(f"Warning: Could not set Rendersharp IRRSHADER. Error: {e}. Falling back to CPU raycasting.")
        sens_manager.SetRaycastingMethod(sens.RaycastingMethod_RAYSHAFT_SW, 4)
    sens_manager.SetVerbose(False) 

    
    lidar_update_rate = 10  
    
    lidar_offset_pose = ch.ChFrameD(ch.ChVector3d(0.0, 0, 0.8), ch.QUNIT) 
    
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),      
        lidar_update_rate,           
        lidar_offset_pose,           
        2000,                        
        32,                          
        2 * math.pi,                 
        math.radians(15),            
        math.radians(-15),           
        100.0,                       
        sens.LidarBeamShape_RECTANGULAR, 
        1,                           
        0.003,                       
        0.1                          
    )
    lidar.SetName("LidarSensor")
    lidar.SetLag(1 / lidar_update_rate) 
    lidar.SetMaxCollectionWindow(0.0)   

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())      
    lidar.PushFilter(sens.ChFilterIntensityFromRange()) 
    lidar.PushFilter(sens.ChFilterXYZIAccess())       
    
    
    

    sens_manager.AddSensor(lidar) 

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler()) 
    
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    
    lidar_ros_frame_id = "lidar_link" 
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(
        lidar_update_rate,  
        lidar,              
        "~/output/hmmwv/lidar/point_cloud",  
        lidar_ros_frame_id  
    ))
    
    try:
        ros_manager.Initialize()  
    except Exception as e:
        print(f"Error: Could not initialize ROS manager: {e}")
        print("Please ensure ROS master (roscore) is running and
print("error happened with only start ```python")