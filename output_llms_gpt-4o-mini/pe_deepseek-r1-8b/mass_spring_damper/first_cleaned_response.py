import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, irrlight, vehicle


sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, -1, 0))  
ground.SetMass(0)  
ground.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  
ground.SetFixed(True)  
sys.Add(ground)


mass = chrono.ChBodyEasySphere(0.2, 1000, True, True, chrono.ChContactMaterialSMC())
mass.SetPos(chrono.ChVector3d(0, 0, 5))
sys.Add(mass)


spring_link = chrono.ChLinkTSDA()
spring_link.Initialize(mass, ground, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))


spring_link.SetSpring(100.0)  
spring_link.SetDamping(0.5)  


vis = irrlight.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mass-Spring-Damper Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  


vis.AddTypicalLights()


mass_shape = chrono.ChVisualShapeSphere(0.3, chrono.ChColor(1, 0, 0))  
spring_shape = chrono.ChVisualShapeCylinder(0.05, 0.2, chrono.ChColor(0, 1, 0))  
vis.AddVisualShape(mass_shape)
vis.AddVisualShape(spring_shape)


vis.AttachSystem(sys)


done = False
while not done:
    
    sys.DoStepDynamics(0.01)
    
    
    if not vis.Run():
        done = True
        
    
    vis.BeginScene()
    
    
    vis.Render()
    
    
vis.EndScene()


vis.Destroy()