import math as m  
import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as pardiso  
import pychrono.irrlicht as chronoirr  
import os  



class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0



out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"


sys = chrono.ChSystemSMC()


L = 1.2  
H = 0.3  
K = 0.07  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.015, 0
print("error happened with only start ```python")