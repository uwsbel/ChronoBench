import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


L = 1.0           
mass = 1.0        
width = 0.1       
height = 0.1      


pendulum = chrono.ChBody()


Ixx = (1/12) * mass * (height**2 + width**2)
Iyy = (1/12) * mass * (L**2 + height**2)
Izz = (1/12) * mass * (L**2 + width**2)
pendulum.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))
pendulum.SetMass(mass)


pendulum.SetPos(chrono.ChVectorD(0, -L/2, 0))
pendulum.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  


pendulum_shape = chrono.ChBoxShape()
pendulum_shape.GetBoxGeometry().Size = chrono.ChVectorD(width/2, L/2, height/2)
pendulum.GetAssets().push_back(pendulum_shape)


system.Add(pendulum)


joint = chrono.ChRevoluteJoint()
system.Add(joint)
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


application = chronoirr.ChIrrApp(system, 'Pendulum Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 3), chronoirr.vector3df(0, 0, 0))
application.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(0, 0, 0), 3, 1, 10, 40, 512, chronoirr.SColorf(1, 1, 1))
application.AssetBindAll()
application.AssetUpdateAll()


next_log_time = 0.0
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    application.DoStep()
    
    
    time = system.GetChTime()
    if time >= next_log_time:
        theta = joint.GetAngle()  
        omega = joint.GetSpeed()  
        print(f"Time: {time:.3f}s, Theta: {theta:.4f} rad, Omega: {omega:.4f} rad/s")
        next_log_time += 0.1  
    
    application.EndScene()