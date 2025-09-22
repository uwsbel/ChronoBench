import os
import math
import numpy as np
import matplotlib.pyplot as plt

import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()








msize = chrono.ChVector3d(1, 1, 1)

mcub = chrono.ChVisualShapeCube(msize.x, msize.y, msize.z)

mcub.SetColor(chrono.ChColor(0.6, 0, 0))

mbody = chrono.ChBody()
mbody.AddVisualShape(mcub)

mbody.SetPos(chrono.ChVector3d(0, 0, 0))

sys.Add(mbody)











spring = chrono.ChLinkTSDA()

spring.Initialize(mbody,                         
                  sys.GetGroundBody(),           
                  True,                          
                  chrono.ChVector3d(0, 0, 0),     
                  chrono.ChVector3d(3, 0, 0),     
                  chrono.CoordinateSystem())

spring.SetRestLength(3.0)
spring.SetSpringCoefficient(50)

spring.SetDampingCoefficient(3)

springVis = chrono.ChVisualShapeSpring(0.2, 150, 10, 0.1)
spring.AddVisualShapeSpring(springVis)

sys.Add(spring)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 6, -6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(1e-3)