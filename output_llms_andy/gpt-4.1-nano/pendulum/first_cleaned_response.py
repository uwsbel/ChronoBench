import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, 'Simple Pendulum Simulation', irr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 1, -3))
application.AddTypicalLights()


ground = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)


mass = 1.0
radius = 0.1
inertia = chrono.ChVectorD(0.005, 0.005, 0.005)  

pendulum_body = chrono.ChBodyEasySphere(radius, 1000, True, True)
pendulum_body.SetMass(mass)
pendulum_body.SetInertiaXX(inertia)

pivot_point = chrono.ChVectorD(0, 1, 0)  
initial_angle_deg = 30  
initial_angle_rad = chrono.CH_C_PI / 6  
initial_x = 0.5 * chrono.Sin(initial_angle_rad)
initial_y = 1 - 0.5 * chrono.Cos(initial_angle_rad)
pendulum_body.SetPos(chrono.ChVectorD(initial_x, initial_y, 0))
system.Add(pendulum_body)


pivot = chrono.ChVectorD(0, 1, 0)
revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum_body, chrono.ChCoordsysD(pivot))
system.Add(revolute_joint)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.005
total_time = 10  
current_time = 0

print("Time\tAngle(deg)\tAngularVelocity(deg/s)")

while application.GetDevice().run() and current_time < total_time:
    
    if int(current_time / time_step) % 20 == 0:  
        
        rel_rot = pendulum_body.GetRelRot()
        angle_rad = rel_rot.Q_to_Rot().GetAngle()
        
        
        
        joint_angle = revolute_joint.GetRevoluteAngle()
        joint_vel = revolute_joint.GetRevoluteAngleVel()
        print(f"{current_time:.2f}\t{chrono.RadToDeg(joint_angle):.2f}\t{chrono.RadToDeg(joint_vel):.2f}")

    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    system.DoStepDynamics(time_step)
    application.AssetUpdateAll()

    current_time += time_step