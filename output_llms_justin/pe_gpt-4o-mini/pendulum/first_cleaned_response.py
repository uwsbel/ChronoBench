import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')  
sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetMass(0)  
ground.SetCollide(False)  
ground.SetFixed(True)  
sys.Add(ground)


pendulum_length = 2.0  
pendulum_mass = 1.0  
pendulum = chrono.ChBodyEasyBox(0.1, 0.1, pendulum_length, pendulum_mass, True, True)
pendulum.SetPos(chrono.ChVector3d(0, -pendulum_length / 2, 0))  
pendulum.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.1))  
sys.Add(pendulum)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(rev_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 2, 5))
vis.AddTypicalLights()
vis.AddSkyBox()


time_step = 0.01  
while vis.Run():
    
    sys.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f"Pendulum Position: {pendulum_pos}, Velocity: {pendulum_vel}")


vis.Drop();