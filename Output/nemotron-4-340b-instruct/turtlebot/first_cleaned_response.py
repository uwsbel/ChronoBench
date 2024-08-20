import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.ChSystemSMC()
system = chrono.ChSystemSMC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground_body = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True, chrono.ChVectorD(0, -0.5, 0))
system.Add(ground_body)



turtlebot_position = chrono.ChVectorD(0, 0.2, 0)
turtlebot_orientation = chrono.ChQuaternionD(1, 0, 0, 0)
turtlebot = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1, True, True, turtlebot_position, turtlebot_orientation)
system.Add(turtlebot)


wheel_radius = 0.05
wheel_width = 0.02
wheel_mass = 0.1
left_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, wheel_mass, True, True, chrono.ChVectorD(-0.1, 0.1, 0), chrono.ChQuaternionD(1, 0, 0, 0))
right_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, wheel_mass, True, True, chrono.ChVectorD(0.1, 0.1, 0), chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(left_wheel)
system.Add(right_wheel)


left_motor = chrono.ChLinkMotorRotationSpeed()
left_motor.Initialize(turtlebot, left_wheel, chrono.ChFrameD(chrono.ChVectorD(-0.1, 0.1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(left_motor)

right_motor = chrono.ChLinkMotorRotationSpeed()
right_motor.Initialize(turtlebot, right_wheel, chrono.ChFrameD(chrono.ChVectorD(0.1, 0.1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(right_motor)



contact_method = chrono.ChMaterialSurface.SMC
contact_material = chrono.ChMaterialSurfaceSMC()
contact_material.SetFriction(0.9)
contact_material.SetRestitution(0.0)
left_wheel.GetCollisionModel().SetDefaultMaterial(contact_material)
right_wheel.GetCollisionModel().SetDefaultMaterial(contact_material)
ground_body.GetCollisionModel().SetDefaultMaterial(contact_material)


left_motor_speed = 0
right_motor_speed = 0


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation")
vis.AddCamera(chronoirr.ChCamera(chronoirr.ChVectorD(0, 1, -2), chronoirr.ChVectorD(0, 0.2, 0)))
vis.AddLight(chronoirr.ChLight(chronoirr.ChVectorD(10, 10, 10), chronoirr.ChColor(1, 1, 1), 100))
vis.Initialize()


timestep = 0.01
simulation_time = 0
turn_time = 2  

while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    if simulation_time > turn_time:
        left_motor_speed = -1
        right_motor_speed = 1
    elif simulation_time > turn_time * 2:
        left_motor_speed = 1
        right_motor_speed = -1
    else:
        left_motor_speed = 0
        right_motor_speed = 0

    left_motor.SetSpeedFunction(chrono.ChFunction_Const(left_motor_speed))
    right_motor.SetSpeedFunction(chrono.ChFunction_Const(right_motor_speed))

    system.DoStepDynamics(timestep)
    simulation_time += timestep

    vis.EndScene()