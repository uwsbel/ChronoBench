import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl  
import pychrono.irrlicht as chronoirr



beam_L = 10.0  


beam_ro = 0.060  
beam_ri = 0.055  



sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)



sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  




mesh.SetAutomaticGravity(True, 2)



minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)

minertia.SetArea(m.pi * (pow(beam_ro, 2) - pow(beam_ri
print("error happened with only start ```python")