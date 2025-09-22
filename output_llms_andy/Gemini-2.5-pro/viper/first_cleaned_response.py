import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))





step_size = 0.005  


t_end = 20.0      





steering_change_duration = 10.0 
max_steering_input = 0.5      
target_speed = 1.0            


init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0) 


camera_chase_dist = 6.0
camera_height = 1.5
camera_angle = 0.2 




print("Creating Chrono system...")
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
sys.GetSolver().AsIterative().SetMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)




print("Creating rigid terrain...")
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain_patch = terrain.AddPatch(patch_mat,
                                 chrono.CSYSNORM, 
                                 200.0, 200.0)    

terrain_patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5)) 
terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()
print("Terrain initialized.")




print("Creating Viper rover...")
viper = veh.Viper(sys)

viper.SetChassisFixed(False) 
viper.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))








viper.SetTireType(veh.TireModelType_TMEASY) 




viper.SetTireCollisionType(veh.Chassis.CollisionType_PRIMITIVES) 


viper.Initialize()
print("Viper rover initialized.")


viper.SetChassisVisualizationType(veh.VisualizationType_MESH)
viper.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetWheelVisualizationType(veh.VisualizationType_MESH)
viper.SetTireVisualizationType(veh.VisualizationType_MESH) 




print("Creating Viper driver...")

driver = veh.ViperSpeedDriver(viper.GetVehicle(), target_speed)
driver.Initialize()
print("Driver initialized.")




print("Creating Irrlicht visualization...")
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowTitle('Viper Rover on Rigid Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(viper.GetChassisBody().GetFrameCOMToAbs().GetPos(), camera_chase_dist, camera_height)
vis.SetChaseCameraAngle(camera_angle) 
vis.SetChaseCameraState(chronoirr.ChChaseCamera.State_ChaseRigid) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddTypicalLights()


vis.BindAll()
print("Visualization initialized.")




print(f"Starting simulation loop for {t_end} seconds...")
time = 0.0
frame_number = 0

while vis.Run() and time < t_end:
    current_time = sys.GetChTime()

    
    
    if current_time < steering_change_duration:
        steering_input = (current_time / steering_change_duration) * max_steering_input
    else:
        steering_input = max_steering_input
    

    driver.SetSteering(steering_input)
    driver.SetThrottle(1.0) 
    driver.SetBraking(0.0)

    
    driver.Synchronize(current_time)
    terrain.Synchronize(current_time)
    viper.Synchronize(current_time, driver.GetSteering(), driver.GetDrivingMode(), driver.GetBraking(), terrain)
    
    

    
    sys.DoStepDynamics(step_size)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if frame_number % 100 == 0: 
        print(f"Time: {current_time:.2f} s, Steering: {steering_input:.3f} rad")
        
        

    time += step_size
    frame_number += 1

print("Simulation finished.")