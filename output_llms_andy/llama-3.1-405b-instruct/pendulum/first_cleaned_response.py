import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(system,  
                              10, 10, 1,  
                              1000,  
                              True,  
                              True)  
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBodyEasyCylinder(system,  
                                     0.5,  
                                     2.0,  
                                     1000,  
                                     True,  
                                     True)  
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(pendulum)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 1, 0)))
system.Add(joint)


application = chronoirr.ChIrrApp(system, 'PyChrono Simple Pendulum', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(2, 2, 2), chronoirr.vector3df(0, 0, 0))


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    
    if system.GetChTime() % 0.1 < 0.01:
        print(f"Time: {system.GetChTime():.2f} s")
        print(f"Pendulum Position: {pendulum.GetPos().x:.2f}, {pendulum.GetPos().y:.2f}, {pendulum.GetPos().z:.2f}")
        print(f"Pendulum Velocity: {pendulum.GetPos_dt().x:.2f}, {pendulum.GetPos_dt().y:.2f}, {pendulum.GetPos_dt().z:.2f}")