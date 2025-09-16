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
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVectorD(0, -2, 0))
system.Add(pendulum)


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground,  
                     pendulum,  
                     chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))  
system.Add(rev_joint)


application = chronoirr.ChIrrApp(system,  
                                 'PyChrono Simple Pendulum',  
                                 chronoirr.dimension2du(800, 600))  


application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(2, -4, 0))  
application.AddSkyBox()


while application.GetDevice().run():
    
    application.DoStepDynamics(0.01)

    
    print('Pendulum position:', pendulum.GetPos())
    print('Pendulum velocity:', pendulum.GetPos_dt())