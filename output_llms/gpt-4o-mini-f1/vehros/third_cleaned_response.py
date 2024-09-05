import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens
from pychrono.sensor import *
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
    
    
    box = ch.ChBodyEasyBox(3, 3, 3, 1000, True, True,patch_mat)
    box.SetPos(ch.ChVector3d(0, 0, 1.5))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/blue.png"))
    hmmwv.GetSystem().Add(box)
    
    
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

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  
    
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, "~/output/lidar/data"))
    ros_manager.Initialize()  
    
    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    
    
    lidar = sens.ChLidarSensor(hmmwv.GetChassis().GetBody(),  
                                ch.ChFrame<double>(ch.ChVector3d(2.0, 0, 1.5), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0))),  
                                0.5,              
                                2,                
                                512,              
                                2*CH_PI,          
                                CH_PI/8,          
                                -CH_PI/4,         
                                100.0,            
                                sens.LidarBeamShape_RECTANGULAR,  
                                0.002,            
                                0.002,            
                                0,                
                                0,                
                                sens.LidarReturnMode_STRONGEST_RETURN  
                                )
    
    lidar.SetUpdateFrequency(10)
    
    sens_manager.AddSensor(lidar)
    
    
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
        
        
        sens_manager.Update()
        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        
        if not ros_manager.Update(time, time_step):
            break  
        step_number += 1

if __name__ == "__main__":
    main()