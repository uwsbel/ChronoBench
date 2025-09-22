import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.scm as scm
import os
import math




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))





step_size = 5e-4  


time_end = 100


initLoc = chrono.ChVectorD(-70, 1.0, -70) 
initRot = chrono.ChQuaternionD(1, 0, 0, 0) 


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH 
tire_vis_type = veh.VisualizationType_MESH




tire_model = veh.TireModelType_RIGID


trackPoint = chrono.ChVectorD(0.0, 0.0, 0.0) 


patch_dim_x = 160.0  
patch_dim_y = 160.0  


enable_bulldozing = True



plot_type = scm.SCMDeformableTerrain.PLOT_SINKAGE

plot_output_min = 0
plot_output_max = 0.3 





system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetMaxItersSolverSpeed(150)
system.SetMaxItersSolverStab(150)






my_hmmwv = veh.HMMWV_Reduced(system)
my_hmmwv.SetContactMethod(chrono.ChMaterialSurface.SMC) 
my_hmmwv.SetChassisFixed(False)
my_hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE) 
my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD) 
my_hmmwv.SetTireType(tire_model)
my_hmmwv.SetTireStepSize(step_size) 


my_hmmwv.Initialize()


my_hmmwv.SetChassisVisualizationType(chassis_vis_type)
my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
my_hmmwv.SetSteeringVisualizationType(steering_vis_type)
my_hmmwv.SetWheelVisualizationType(wheel_vis_type)
my_hmmwv.SetTireVisualizationType(tire_vis_type)


vehicle = my_hmmwv.GetVehicle()





terrain = scm.SCMDeformableTerrain(system)




terrain.SetSoilParameters(2e6,   
                          0,     
                          1.1,   
                          20e3,  
                          30.0,  
                          0.01,  
                          1.2e8, 
                          3e4)   


terrain.SetBulldozingFlow(enable_bulldozing)
if enable_bulldozing:
    terrain.SetBulldozingParameters(55,   
                                    1,    
                                    0.2,  
                                    1)    


terrain.SetPlotType(plot_type, plot_output_min, plot_output_max)




heightmap_file = chrono.GetChronoDataFile("terrain/height_maps/slope.bmp")



terrain_length = patch_dim_x
terrain_width = patch_dim_y


min_height = 0.0
max_height = 2.0 








up_dir = chrono.ChVectorD(0, 1, 0)
resolution_x = 0.1  
resolution_y = 0.1  

terrain.Initialize(heightmap_file,
                   terrain_length, terrain_width,
                   min_height, max_height,
                   up_dir,
                   resolution_x, resolution_y)


terrain.GetMesh().SetWireframe(False) 





vis_app = irr.ChIrrApp(system, "HMMWV on SCM Deformable Terrain", irr.dimension2du(1280, 720))
vis_app.SetTimestep(step_size) 






vis_app.AddLight(irr.SLight(chrono.ChVectorD(100,100,100), chrono.ChColor(0.8,0.8,0.8),300))
vis_app.AddLight(irr.SLight(chrono.ChVectorD(-100,100,-100), chrono.ChColor(0.8,0.8,0.8),300))






vis_app.AssetBindAll()
vis_app.AssetUpdateAll()





vis_app.SetChaseCamera(trackPoint, my_hmmwv.GetChassis().GetBody(), 6.0)
vis_app.SetChaseCameraState(irr.ChChaseCamera.Track) 
vis_app.SetChaseCameraPosition(my_hmmwv.GetChassis().GetBody().TransformPointLocalToParent(trackPoint) + chrono.ChVectorD(-8, 3, 0))
vis_app.SetChaseCameraAngle(-math.pi/8) 
vis_app.SetChaseCameraMultipliers(1.0, 10.0) 





driver = veh.ChIrrGuiDriver(vis_app)



driver.SetSteeringDelta(0.02)  
driver.SetThrottleDelta(0.04)  
driver.SetBrakingDelta(0.06)
driver.Initialize()







realtime_timer = chrono.ChRealtimeStepTimer()



time = 0.0

print("Starting simulation...")
print("------------------------------------")
print("Controls:")
print("  Throttle:  W / S")
print("  Steering:  A / D")
print("  Braking:   Spacebar (hold)")
print("  Toggle Camera Mode: C")
print("------------------------------------")


while vis_app.GetDevice().run():
    time = system.GetChTime()

    
    if time >= time_end:
        print("Simulation ended.")
        break

    
    steering_input = driver.GetSteering()
    throttle_input = driver.GetThrottle()
    braking_input = driver.GetBraking()

    
    driver.Synchronize(time)
    terrain.Synchronize(time) 
    my_hmmwv.Synchronize(time, steering_input, throttle_input, braking_input, terrain)
    
    vis_app.BeginScene(True, True, chrono.ChColor(0.2, 0.3, 0.5)) 
    vis_app.Render() 

    
    driver.DrawAll()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_hmmwv.Advance(step_size)
    system.DoStepDynamics(step_size) 
    

    vis_app.EndScene()

    
    realtime_timer.Spin(step_size)


print("Simulation complete.")