import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetCollide(False)
system.Add(ground)


pendulum_length = 1.0  
pendulum_mass = 1.0    
pendulum_inertia = (1/3) * pendulum_mass * pendulum_length**2  

pendulum = chrono.ChBody()
pendulum.SetMass(pendulum_mass)
pendulum.SetInertiaXX(chrono.ChVectorD(0, pendulum_inertia, pendulum_inertia))
pendulum.SetPos(chrono.ChVectorD(pendulum_length, 0, 0))
pendulum.SetCollide(False)
system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)


pendulum.SetPos(chrono.ChVectorD(pendulum_length * np.sin(np.pi / 4), -pendulum_length * np.cos(np.pi / 4), 0))  
pendulum.SetPos_dt(chrono.ChVectorD(0, 0, 0))  


application = chronoirr.ChIrrApp(system, "Simple Pendulum Simulation", chronoirr.dimension2d(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 2, 5), chronoirr.vector3df(0, 0, 0))


time_step = 0.01
application.SetTimestep(time_step)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    system.DoStepDynamics(time_step)

    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f"Time: {system.GetChTime():.2f} s, Position: {pendulum_pos}, Velocity: {pendulum_vel}")


application.GetDevice().drop()