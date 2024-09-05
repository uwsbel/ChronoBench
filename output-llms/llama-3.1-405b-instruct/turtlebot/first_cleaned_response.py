import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(system,  
                              10, 10, 1,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChVectorD(0, -1, 0),  
                              chrono.ChQuaternionD(1, 0, 0, 0))  


turtlebot = chrono.ChBodyEasyBox(system,  
                                 0.2, 0.2, 0.1,  
                                 100,  
                                 True,  
                                 True,  
                                 chrono.ChVectorD(0, 0.1, 0),  
                                 chrono.ChQuaternionD(1, 0, 0, 0))  


motor_l = chrono.ChLinkMotorRotationSpeed(turtlebot,  
                                          chrono.ChFrameD(chrono.ChVectorD(-0.05, 0, 0),  
                                                          chrono.ChQuaternionD(1, 0, 0, 0)),  
                                          0.1)  
motor_r = chrono.ChLinkMotorRotationSpeed(turtlebot,  
                                          chrono.ChFrameD(chrono.ChVectorD(0.05, 0, 0),  
                                                          chrono.ChQuaternionD(1, 0, 0, 0)),  
                                          0.1)  


system.Add(motor_l)
system.Add(motor_r)


application = chronoirr.ChIrrApp(system,  
                                 'Turtlebot Simulation',  
                                 chronoirr.dimension2du(800, 600))  


application.AddCamera(chronoirr.vector3df(0, 0, 1),  
                      chronoirr.vector3df(0, 0, 0))  


application.AddLight(chronoirr.vector3df(2, 2, 2),  
                     chronoirr.vector3df(0, 0, 0),  
                     3,  
                     0.5,  
                     0.5,  
                     0.5)  


application.AssetBindAll()
application.AssetUpdateAll()

time = 0
while application.GetDevice().run():
    
    if time < 5:
        motor_l.SetMotorFunction(chrono.ChFunction_Ramp(0, 1))
        motor_r.SetMotorFunction(chrono.ChFunction_Ramp(0, 1))
    elif time < 10:
        motor_l.SetMotorFunction(chrono.ChFunction_Ramp(-1, 0))
        motor_r.SetMotorFunction(chrono.ChFunction_Ramp(1, 0))
    else:
        motor_l.SetMotorFunction(chrono.ChFunction_Ramp(0, -1))
        motor_r.SetMotorFunction(chrono.ChFunction_Ramp(0, -1))

    
    application.DoStep()
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    time += application.GetStepSize()


application.GetDevice().closeDevice()