import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as chronorobo
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))




ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
ground.EnableCollision(True)


ground_box = chrono.ChVisualShapeBox(20, 20, 0.2)
ground_box.SetColor(chrono.ChColor(0.2, 0.3, 0.1))
ground.AddVisualShape(ground_box)

ground_coll = chrono.ChCollisionShapeBox(20, 20, 0.2)
ground.AddCollisionShape(ground_coll)

system.Add(ground)





init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.QuatFromAngleZ(20 * chrono.CH_DEG_TO_RAD)
turtlebot = chronorobo.TurtleBot()
turtlebot.Initialize(chrono.ChCoordsysd(init_pos, init_rot), system)


turtlebot.SetRobotVisualizationType(chronorobo.RobotVisualizationType_MESH)
turtlebot.SetWheelVisualizationType(chronorobo.RobotVisualizationType_PRIMITIVES)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 2, 1.5))
vis.AddTypicalLights()




time_step = 0.005
sim_time = 0
max_time = 30.0


motor_speed = 3.0  
turn_duration = 2.0  
current_motor_speed = 0




while vis.Run() and sim_time < max_time:
    
    
    
    if sim_time < 5.0:
        
        left_speed = motor_speed
        right_speed = motor_speed
    elif sim_time < 5.0 + turn_duration:
        
        left_speed = motor_speed
        right_speed = -motor_speed
    elif sim_time < 10.0 + turn_duration:
        
        left_speed = motor_speed
        right_speed = motor_speed
    elif sim_time < 10.0 + 2*turn_duration:
        
        left_speed = -motor_speed
        right_speed = motor_speed
    else:
        
        left_speed = motor_speed
        right_speed = motor_speed

    
    turtlebot.SetMotorSpeeds(left_speed, right_speed)
    
    
    
    
    vis.BeginScene()
    vis.Render()
    
    
    trail_color = chrono.ChColor(1, 0, 0)
    vis.DrawLine(turtlebot.GetChassis().GetPos(), 
                 turtlebot.GetChassis().GetPos() + chrono.ChVector3d(0.1,0,0),
                 trail_color)
    
    
    vis.GetGUI().SetStaticText(f"Time: {sim_time:.2f}s\n" 
                               f"Left Motor: {left_speed:.2f} rad/s\n"
                               f"Right Motor: {right_speed:.2f} rad/s", 
                               400, 10)
    
    vis.EndScene()
    system.DoStepDynamics(time_step)
    sim_time += time_step

print("Simulation completed successfully")