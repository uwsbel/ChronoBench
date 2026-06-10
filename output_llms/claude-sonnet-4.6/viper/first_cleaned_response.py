import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


system.SetSolverMaxIterations(150)
system.SetSolverForceTolerance(1e-10)






ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)


ground = chrono.ChBodyEasyBox(
    20, 20, 0.5,   
    1000,          
    True,          
    True,          
    ground_mat     
)
ground.SetPos(chrono.ChVectorD(0, 0, -0.25))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)






import pychrono.robot as robot


viper = robot.Viper(system)


driver = robot.ViperDCMotorControl()
viper.SetDriver(driver)


init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)

viper.Initialize(chrono.ChFrameD(init_pos, init_rot))





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVectorD(3, 3, 2),   
    chrono.ChVectorD(0, 0, 0.5)  
)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVectorD(5, 5, 5),   
    chrono.ChVectorD(0, 0, 0),   
    10,                           
    1,                            
    20,                           
    50                            
)





timestep = 1e-3          
total_time = 20.0        
steering_change_time = 5.0  


max_steering = 0.4       
current_time = 0.0


realtime_timer = chrono.ChRealtimeStepTimer()





print("Starting Viper Rover Simulation...")
print(f"Total simulation time: {total_time} s")
print(f"Timestep: {timestep} s")

step_number = 0

while vis.Run() and current_time < total_time:

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    
    
    if current_time < steering_change_time:
        
        steering = max_steering * (current_time / steering_change_time)
    elif current_time < 2 * steering_change_time:
        
        steering = max_steering * (1.0 - (current_time - steering_change_time) / steering_change_time)
    else:
        
        steering = 0.0

    
    driver.SetSteering(steering)

    
    
    
    viper.Update()

    
    
    
    system.DoStepDynamics(timestep)

    
    current_time += timestep
    step_number += 1

    
    realtime_timer.Spin(timestep)

    
    if step_number % 1000 == 0:
        rover_pos = viper.GetChassisPos()
        print(f"Time: {current_time:.2f}s | "
              f"Rover Pos: ({rover_pos.x:.3f}, {rover_pos.y:.3f}, {rover_pos.z:.3f}) | "
              f"Steering: {steering:.3f}")

print("\nSimulation completed!")