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
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.015, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  
myfun = ChFunctionMyFun()  
motor.SetAngleFunction(myfun)  
sys.Add(motor)  


mesh = fea.ChMesh()


beam_wy = 0.12  
beam_wz = 0.012  


minertia = fea.ChInertiaCosseratSimple()  
melasticity = fea.ChElasticityCosseratSimple()  
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  
msection1.SetDensity(2700)  
msection1.SetYoungModulus(73e9)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetShearModulusFromPoisson(0.3)  

1. SetShearModulusFromPoisson(0.3)  

The given PyChrono script:

The given PyChrono script:












The given PyChrono::

The given PyChrono::

The given PyChrono script:

The given PyChrono script:

The given PyChrono as ChFunctionMyFun requires modification and extension of PyChrono::

The given PyChrono::

The given PyChrono::
























The given PyChrono::
The given PyChrono::

The given PyChrono.py

The given PyChrono::

The given PyChrono.py

The given PyChrono library for creating a simple motor.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py
The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py


The given PyChrono.py

The given PyChrono.py
The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

The given PyChrono.py

Here is the given PyChrono.py

The given PyChrono.py
Here is the script.py
functionally.py
function description = 


























parts:




































to modify and add the given script:

































.py










































































































































































function(x).py



return 




return 

Secondly





return 










































function


















function(sys).py























.


































































function.py










.
























.






function



function











function.py

.
function.py














function.py






function.py
print("error happened with only start ```python")