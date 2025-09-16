import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
body_ground.SetShapeBoxes(0.5, 0.1, 1.0)  
body_ground.SetCollide(True)
body_ground.SetMaterialSurfaceNSC(chrono.ChMaterialSurfaceNSC())
system.Add(body_ground)




body_rover = chrono.ChBodyEasy()
body_rover.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_rover.SetShapeBoxes(0.3, 0.3, 0.3)
body_rover.SetCollide(True)
body_rover.SetMaterialSurfaceNSC(chrono.ChMaterialSurfaceNSC())
system.Add(body_rover)


joint_steering = chrono.ChLinkRevolute()
joint_steering.Initialize(body_rover,
                          chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)),  
                          chrono.ChFrameD(chrono.ChVectorD(0.15, 0, 0)), 
                          chrono.ChVectorD(0, 1, 0)) 
system.AddLink(joint_steering)



motor_steering = chrono.ChMotorLinearSpring()
motor_steering.Set_spring_rest_length(0)
motor_steering.Set_spring_k(1000)
motor_steering.Set_damping_c(10)
joint_steering.AddMotor(motor_steering)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -3))
vis.AddTypicalLights()
vis.AddLogo()
vis.SetShadowsEnabled(True)


time_step = 0.01
time_end = 10

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    angle = np.sin(chrono.ChTime() / 2) * 0.5  
    motor_steering.Set_spring_rest_length(angle)

    if system.GetChTime() > time_end:
        vis.GetIrrlichtApplication().Close()