import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


joint_vis = chrono.ChVisualShapeSphere(2)  
ground.AddVisualShape(joint_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))



pend_1.SetPos(chrono.ChVector3d(1, 0, 1))


pend_1.SetWvel_par(chrono.ChVector3d(0, 0, 0.5))  



sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(sph_1)