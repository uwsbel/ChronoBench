import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)


ground = chrono.ChBodyEasyBox(100, 2, 100, 2000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/mars_surface.jpg"))


ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.2)
ground_mat.SetRollingFriction(0.01)
ground_mat.SetSpinningFriction(0.01)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(ground_mat, 50, 1, 50, chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().BuildModel()

system.Add(ground)



rover_body = chrono.ChBodyEasyBox(1.5, 0.5, 2.0, 300, True, True)
rover_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
rover_body.SetMass(300)
rover_body.SetInertiaXX(chrono.ChVectorD(50, 50, 50))
rover_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/nasa_logo.png"))
system.Add(rover_body)


rover_mat = chrono.ChMaterialSurfaceNSC()
rover_mat.SetFriction(0.9)
rover_mat.SetRestitution(0.1)
rover_mat.SetRollingFriction(0.05)
rover_mat.SetSpinningFriction(0.05)


wheels = []
wheel_radius = 0.3
wheel_width = 0.2
wheel_mass = 30


wheel_positions = [
    chrono.ChVectorD(0.7, -0.1, 0.8),  
    chrono.ChVectorD(0.7, -0.1, -0.8),  
    chrono.ChVectorD(0.0, -0.1, 0.8),   
    chrono.ChVectorD(0.0, -0.1, -0.8),  
    chrono.ChVectorD(-0.7, -0.1, 0.8),  
    chrono.ChVectorD(-0.7, -0.1, -0.8), 
]

motors = []

for i, pos in enumerate(wheel_positions):
    
    wheel = chrono.ChBodyEasyCylinder(
        wheel_radius, wheel_width, 1000, 
        True, True, rover_mat
    )
    
    
    wheel.SetPos(rover_body.GetPos() + pos)
    
    
    if i % 2 == 0:  
        wheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1)))
    else:  
        wheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1)))
    
    wheel.SetMass(wheel_mass)
    wheel.SetInertiaXX(chrono.ChVectorD(2, 2, 2))
    wheel.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.2))
    
    
    system.Add(wheel)
    wheels.append(wheel)
    
    
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(wheel, rover_body, 
                    chrono.ChCoordsysD(rover_body.GetPos() + pos, 
                                      chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1))))
    system.Add(joint)
    
    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(wheel, rover_body, 
                    chrono.ChFrameD(rover_body.GetPos() + pos, 
                                   chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1))))
    motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor.SetMotorFunction(chrono.ChFunction_Const(0.0))  
    system.Add(motor)
    motors.append(motor)


camera_mount = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 5, True, False)
camera_mount.SetPos(rover_body.GetPos() + chrono.ChVectorD(0.8, 0.5, 0))
camera_mount.SetCollide(False)
system.Add(camera_mount)


camera_joint = chrono.ChLinkLockLockLock()
camera_joint.Initialize(camera_mount, rover_body, chrono.ChCoordsysD(camera_mount.GetPos()))
system.Add(camera_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, -6), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(10, 20, 10), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 5)


vis.AddShadowAll()


step_size = 0.005
realtime_timer = chrono.ChRealtimeStepTimer()


motor_speed = 0  
steering_angle = 0  

print("Simulation started. Use keys: WASD to control rover.")
print("  W/S - accelerate/decelerate")
print("  A/D - steer left/right")


while vis.Run():
    
    if vis.GetKeyManager().IsKeyPressed(chronoirr.KEY_KEY_W):
        motor_speed += 0.5  
    if vis.GetKeyManager().IsKeyPressed(chronoirr.KEY_KEY_S):
        motor_speed -= 0.5  
    if vis.GetKeyManager().IsKeyPressed(chronoirr.KEY_KEY_A):
        steering_angle += 0.02  
    if vis.GetKeyManager().IsKeyPressed(chronoirr.KEY_KEY_D):
        steering_angle -= 0.02  
    
    
    if abs(steering_angle) > 0.01:
        
        for i in [1, 3, 5]:
            if steering_angle > 0:  
                motors[i].SetMotorFunction(chrono.ChFunction_Const(motor_speed * (1 - 0.5*abs(steering_angle))))
            else:  
                motors[i].SetMotorFunction(chrono.ChFunction_Const(motor_speed * (1 + 0.5*abs(steering_angle))))
                
        
        for i in [0, 2, 4]:
            if steering_angle > 0:  
                motors[i].SetMotorFunction(chrono.ChFunction_Const(motor_speed * (1 + 0.5*abs(steering_angle))))
            else:  
                motors[i].SetMotorFunction(chrono.ChFunction_Const(motor_speed * (1 - 0.5*abs(steering_angle))))
    else:
        
        for motor in motors:
            motor.SetMotorFunction(chrono.ChFunction_Const(motor_speed))
    
    
    rover_pos = rover_body.GetPos()
    vis.GetActiveCamera().setPosition(chronoirr.vector3df(
        rover_pos.x - 5, rover_pos.y + 3, rover_pos.z
    ))
    vis.GetActiveCamera().setTarget(chronoirr.vector3df(
        rover_pos.x, rover_pos.y, rover_pos.z
    ))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    realtime_timer.Spin(step_size)

print("Simulation ended.")