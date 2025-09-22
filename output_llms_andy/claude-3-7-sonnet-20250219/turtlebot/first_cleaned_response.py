import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import math


chrono.SetChronoDataPath('C:/path/to/chrono/data/')  
system = chrono.ChSystemNSC()  
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.8)
ground_material.SetRestitution(0.1)

ground = chrono.ChBodyEasyBox(20, 0.2, 20, 1000, True, True, ground_material)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)


ground_texture = chrono.ChVisualMaterial()
ground_texture.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.6))
ground.GetVisualShape(0).SetMaterial(0, ground_texture)


robot_radius = 0.15
robot_height = 0.1
wheel_radius = 0.035
wheel_width = 0.01
wheel_distance = 0.16


robot_material = chrono.ChMaterialSurfaceNSC()
robot_material.SetFriction(0.7)
robot_material.SetRestitution(0.1)

robot_body = chrono.ChBodyEasyCylinder(robot_radius, robot_height, 1000, True, True, robot_material)
robot_body.SetPos(chrono.ChVectorD(0, wheel_radius + robot_height/2, 0))  
robot_body.SetRot(chrono.Q_from_AngZ(0))  
system.Add(robot_body)


robot_texture = chrono.ChVisualMaterial()
robot_texture.SetDiffuseColor(chrono.ChColor(0.1, 0.1, 0.8))
robot_body.GetVisualShape(0).SetMaterial(0, robot_texture)


wheel_material = chrono.ChMaterialSurfaceNSC()
wheel_material.SetFriction(0.9)
wheel_material.SetRestitution(0.1)


wheel_L = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 1000, True, True, wheel_material)
wheel_L.SetPos(chrono.ChVectorD(0, wheel_radius, wheel_distance/2))
wheel_L.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI/2))  
system.Add(wheel_L)


wheel_R = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 1000, True, True, wheel_material)
wheel_R.SetPos(chrono.ChVectorD(0, wheel_radius, -wheel_distance/2))
wheel_R.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI/2))  
system.Add(wheel_R)


wheel_texture = chrono.ChVisualMaterial()
wheel_texture.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.2))
wheel_L.GetVisualShape(0).SetMaterial(0, wheel_texture)
wheel_R.GetVisualShape(0).SetMaterial(0, wheel_texture)


joint_L = chrono.ChLinkLockRevolute()
joint_L.Initialize(wheel_L, robot_body, 
                  chrono.ChCoordsysD(chrono.ChVectorD(0, wheel_radius, wheel_distance/2), 
                                     chrono.Q_from_AngX(chrono.CH_C_PI/2)))
system.Add(joint_L)

joint_R = chrono.ChLinkLockRevolute()
joint_R.Initialize(wheel_R, robot_body, 
                  chrono.ChCoordsysD(chrono.ChVectorD(0, wheel_radius, -wheel_distance/2), 
                                     chrono.Q_from_AngX(chrono.CH_C_PI/2)))
system.Add(joint_R)


motor_L = chrono.ChLinkMotorRotationSpeed()
motor_L.Initialize(wheel_L, robot_body, 
                  chrono.ChFrameD(chrono.ChVectorD(0, wheel_radius, wheel_distance/2), 
                                 chrono.Q_from_AngX(chrono.CH_C_PI/2)))
motor_L.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
motor_L.SetMotorFunction(chrono.ChFunction_Const(0))  
system.Add(motor_L)

motor_R = chrono.ChLinkMotorRotationSpeed()
motor_R.Initialize(wheel_R, robot_body, 
                  chrono.ChFrameD(chrono.ChVectorD(0, wheel_radius, -wheel_distance/2), 
                                 chrono.Q_from_AngX(chrono.CH_C_PI/2)))
motor_R.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
motor_R.SetMotorFunction(chrono.ChFunction_Const(0))  
system.Add(motor_R)


caster_radius = 0.02
caster_front = chrono.ChBodyEasySphere(caster_radius, 1000, True, True, robot_material)
caster_front.SetPos(chrono.ChVectorD(robot_radius*0.8, caster_radius, 0))
system.Add(caster_front)


caster_joint = chrono.ChLinkLockPointPlane()
caster_joint.Initialize(caster_front, robot_body, chrono.ChCoordsysD(chrono.ChVectorD(robot_radius*0.8, caster_radius, 0)))
system.Add(caster_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(3, 5, -2), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512)


time_step = 0.001
total_time = 10.0  
current_time = 0


while vis.Run() and current_time < total_time:
    
    if current_time < 2.0:
        
        motor_L.SetMotorFunction(chrono.ChFunction_Const(10.0))
        motor_R.SetMotorFunction(chrono.ChFunction_Const(10.0))
    elif current_time < 4.0:
        
        motor_L.SetMotorFunction(chrono.ChFunction_Const(10.0))
        motor_R.SetMotorFunction(chrono.ChFunction_Const(3.0))
    elif current_time < 6.0:
        
        motor_L.SetMotorFunction(chrono.ChFunction_Const(3.0))
        motor_R.SetMotorFunction(chrono.ChFunction_Const(10.0))
    elif current_time < 8.0:
        
        motor_L.SetMotorFunction(chrono.ChFunction_Const(10.0))
        motor_R.SetMotorFunction(chrono.ChFunction_Const(10.0))
    else:
        
        motor_L.SetMotorFunction(chrono.ChFunction_Const(0.0))
        motor_R.SetMotorFunction(chrono.ChFunction_Const(0.0))
    
    
    system.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    current_time += time_step
    
    
    if int(current_time*100) % 100 == 0:  
        pos = robot_body.GetPos()
        print(f"Time: {current_time:.1f}s, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")

print("Simulation completed")