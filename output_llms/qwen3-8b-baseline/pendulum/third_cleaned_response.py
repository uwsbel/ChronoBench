import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
print("error happened with only start ```python")