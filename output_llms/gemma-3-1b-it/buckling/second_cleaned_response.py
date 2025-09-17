```python
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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


boxtruss = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxtruss, chrono.QUNIT)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


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


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)  


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):