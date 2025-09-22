import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math






try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
    veh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))
except KeyError:
    print("Error: CHRONO_DATA_DIR environment variable not set.")
    print("Please set CHRONO_DATA_DIR to the 'data' directory of your Chrono installation.")
    exit(1)





timestep = 0.01  
simulation_duration = 20.0  


initial_pos = chrono.ChVector3d(0, 0.2, 0)  


initial_rot = chrono.ChQuaterniond(1, 0, 0, 0) 


time_to_turn_left_start = 3.0
time_to_turn_left_end = 6.0
time_to_turn_right_start = 9.0
time_to_turn_right_end = 12.0








straight_speed_rad_s = 5.0  
turn_inner_speed_rad_s = 2.0
turn_outer_speed_rad_s = 5.0




print("Initializing Chrono system...")

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type.BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(100)
system.SetMaxPenetrationRecoverySpeed(1.0) 






print("Creating ground...")
ground_material = chrono.ChContactMaterialNSC() 
ground_material.SetFriction(0.9)
ground_material.SetRestitution(0.01)

ground = chrono.ChBodyEasyBox(40, 2, 40, 1000, True, True, ground_material) 
ground.SetPos(chrono.ChVector3d(0, -1, 0)) 
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
system.Add(ground)



print("Initializing Turtlebot...")




robot = veh.TurtleBot(system)



initial_frame = chrono.ChFramed(initial_pos, initial_rot)
robot.Initialize(initial_frame)




driver = veh.ChDriver(robot.GetVehicle()) 
robot.SetDriver(driver) 





print("Initializing Irrlicht visualization...")
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Turtlebot Simulation')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddTypicalLights()



vis.AddCamera(chrono.ChVector3d(3, 2.5, 4), chrono.ChVector3d(0, 0.5, 0)) 


vis.SetSymbolscale(0.1) 
vis.EnableContactDrawing(irr.ContactsDrawMode_CONTACT_FORCES) 




print(f"Starting simulation loop for {simulation_duration} seconds...")
current_time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    irr.draw_chrono_logo(vis, chrono.ChVector2d(10,10), chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')

    
    
    left_wheel_speed = 0.0
    right_wheel_speed = 0.0

    if time_to_turn_left_start <= current_time < time_to_turn_left_end:
        
        left_wheel_speed = turn_inner_speed_rad_s
        right_wheel_speed = turn_outer_speed_rad_s
        if current_time < time_to_turn_left_start + timestep: 
             print(f"Time: {current_time:.2f}s - Turning Left")
    elif time_to_turn_right_start <= current_time < time_to_turn_right_end:
        
        left_wheel_speed = turn_outer_speed_rad_s
        right_wheel_speed = turn_inner_speed_rad_s
        if current_time < time_to_turn_right_start + timestep: 
             print(f"Time: {current_time:.2f}s - Turning Right")
    else:
        
        left_wheel_speed = straight_speed_rad_s
        right_wheel_speed = straight_speed_rad_s
        if current_time == 0.0 or \
           (current_time > time_to_turn_left_end and current_time < time_to_turn_left_end + timestep) or \
           (current_time > time_to_turn_right_end and current_time < time_to_turn_right_end + timestep):
            if straight_speed_rad_s != 0:
                print(f"Time: {current_time:.2f}s - Moving Straight")
            else:
                print(f"Time: {current_time:.2f}s - Stopped")


    robot.SetMotorSpeed(left_wheel_speed, right_wheel_speed)

    
    
    
    robot.Update(current_time, driver) 

    
    system.DoStepDynamics(timestep)

    
    current_time += timestep

    vis.EndScene()

    
    if current_time >= simulation_duration:
        print(f"Simulation ended at time: {current_time:.2f}s")
        break

print("Simulation complete.")


del vis
del system