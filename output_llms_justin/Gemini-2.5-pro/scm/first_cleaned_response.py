import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import os
import math

def main():
    
    
    CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR')
    if not CHRONO_DATA_DIR:
        print("Error: CHRONO_DATA_DIR environment variable not set.")
        print("Please set this variable to the location of your Chrono data files.")
        return
    
    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    
    vehicle.SetDataPath(os.path.join(CHRONO_DATA_DIR, 'vehicle', ''))

    
    step_size = 0.005  
    render_fps = 50
    render_step_size = 1.0 / render_fps
    simulation_duration = 120  

    
    initLoc = chrono.ChVectorD(0, 0, 0.6) 
    initRot = chrono.Q_from_AngZ(0)      

    
    scm_initial_length = 30.0  
    scm_initial_width = 10.0   
    scm_resolution = 0.05    

    
    patch_look_ahead = 2.0       
    patch_half_length = 7.0      
    patch_half_width = 3.5       
    patch_interaction_depth = 0.3 

    
    bekker_Kphi = 0.2e6    
    bekker_Kc = 0.0        
    bekker_n = 1.1         
    mohr_cohesion = 0.0    
    mohr_friction = 30.0   
    janosi_shear_k = 0.01  
    elastic_K = 4e7      
    damping_K = 1.5e5    

    
    camera_distance = 8.0      
    camera_height_offset = 2.5 
    camera_chase_point_local = chrono.ChVectorD(0.5, 0.0, 0.5) 

    
    
    system = chrono.ChSystemNSC() 
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81)) 
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetSolverForceTolerance(1e-4) 

    
    
    hmmwv = vehicle.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC) 
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot)) 
    hmmwv.SetPowertrainType(vehicle.PowertrainModelType_SHAFTS) 
    hmmwv.SetDriveType(vehicle.DrivelineTypeWV_AWD) 
    hmmwv.SetTireType(vehicle.TireModelType_RIGID) 

    
    hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
    
    hmmwv.Initialize() 
    
    
    chassis_body = hmmwv.GetChassisBody()

    
    
    terrain = vehicle.SCMDeformableTerrain(system)
    
    
    
    
    
    

    
    terrain.SetSoilParameters(bekker_Kphi, bekker_Kc, bekker_n,
                              mohr_cohesion, mohr_friction, janosi_shear_k,
                              elastic_K, damping_K)

    
    
    
    
    
    terrain.Initialize(scm_initial_length, scm_initial_width, scm_resolution)
    
    
    terrain.EnableMovingPatch(True) 
    terrain.SetMovingPatchSettings(chassis_body,                             
                                   chrono.ChVectorD(patch_look_ahead, 0, 0), 
                                   chrono.ChVectorD(patch_half_length, patch_half_width, patch_interaction_depth)) 

    
    
    terrain.SetPlotType(vehicle.SCMDeformableTerrain.PLOT_SINKAGE, 0.0, 0.2) 
    
    
    


    
    
    app = irr.ChIrrApp(system, "HMMWV on SCM Deformable Terrain", irr.dimension2du(1280, 720))
    app.SetTimestep(step_size) 

    
    app.AddTypicalSky()
    app.AddTypicalLights()
    
    
    
    app.GetCamera().SetPosition(irr.vector3df(initLoc.x + 10, initLoc.y + 5, initLoc.z + 5))
    app.GetCamera().SetTarget(irr.vector3df(initLoc.x, initLoc.y, initLoc.z))
    
    
    
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    
    driver = vehicle.ChInteractiveDriverIRR(app)
    driver.SetSteeringDelta(0.04) 
    driver.SetThrottleDelta(0.08) 
    driver.SetBrakingDelta(0.10)  
    driver.Initialize()
    hmmwv.SetDriver(driver) 

    
    class MyEventReceiver(irr.IEventReceiver):
        def __init__(self, app_ref):
            super().__init__()
            self.app_ref = app_ref 
            self.quit_simulation = False
        def OnEvent(self, event):
            
            if event.EventType == irr.EET_KEY_INPUT_EVENT and \
               not event.KeyInput.PressedDown and \
               event.KeyInput.Key == irr.KEY_ESCAPE: 
                print("Escape key pressed. Exiting simulation.")
                self.quit_simulation = True
                
                self.app_ref.GetDevice().closeDevice() 
                return True 
            return False 

    event_receiver = MyEventReceiver(app)
    app.SetUserEventReceiver(event_receiver) 

    
    realtime_timer = chrono.ChRealtimeStepTimer() 
    current_render_time = 0.0

    print("\nINSTRUCTIONS:")
    print("Press UP/DOWN arrow keys for throttle/brake.")
    print("Press LEFT/RIGHT arrow keys for steering.")
    print("Press ESC to quit the simulation.\n")

    
    while app.GetDevice().run() and not event_receiver.quit_simulation:
        time = system.GetChTime() 

        
        if time >= simulation_duration:
            print(f"Simulation ended after {simulation_duration} seconds.")
            break

        
        if chassis_body:
            ch_pos = chassis_body.GetPos() 
            ch_rot = chassis_body.GetRot() 
            
            
            cam_target_global = ch_pos + ch_rot.Rotate(camera_chase_point_local)
            
            
            
            
            dir_forward_vehicle = ch_rot.GetXaxis() 
            
            dir_up_world = chrono.ChVectorD(0, 0, 1) 
            
            cam_pos_global = cam_target_global - dir_forward_vehicle * camera_distance + dir_up_world * camera_height_offset

            
            app.GetCamera().SetPosition(irr.vector3df(cam_pos_global.x, cam_pos_global.y, cam_pos_global.z))
            app.GetCamera().SetTarget(irr.vector3df(cam_target_global.x, cam_target_global.y, cam_target_global.z))


        
        if time >= current_render_time:
            app.BeginScene(
print("error happened with only start ```python")