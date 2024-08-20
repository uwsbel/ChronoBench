import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')  


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              10, 10, 1,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChVectorD(0, 0, -1))  
ground.SetBodyFixed(True)  


pendulum = chrono.ChBodyEasyCylinder(sys,  
                                     0.1,  
                                     1.0,  
                                     1000,  
                                     True,  
                                     True,  
                                     chrono.ChVectorD(0, 0, 1))  
pendulum.SetMass(10)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground,  
                     pendulum,  
                     chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))  
sys.AddLink(rev_joint)  


application = chronoirr.ChIrrApp(sys,  
                                 'PyChrono Simple Pendulum',  
                                 chronoirr.dimension2du(800, 600))  


application.AddTypicalSky()  
application.AddTypicalLights()  
application.AddCamera(chrono.ChVectorD(0, 0, 2))  


while application.GetDevice().run():
    
    sys.DoStepDynamics(0.01)  

    
    print('Pendulum position:', pendulum.GetPos().x(), pendulum.GetPos().y(), pendulum.GetPos().z())
    print('Pendulum velocity:', pendulum.GetPos_dt().x(), pendulum.GetPos_dt().y(), pendulum.GetPos_dt().z())

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()