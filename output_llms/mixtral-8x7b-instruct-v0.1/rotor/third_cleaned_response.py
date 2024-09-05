import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        super().__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif self.T1 <= x < self.T2:
            return self.A2 * m.sin(self.w * x)
        else:
            return self.A1 * m.sin(self.w * x + m.pi)

sys = chrono.ChSystemSMC()




rotmotor1 = chrono.ChLinkMotorRotationSpeed()

rotmotor1_func = ChFunctionMyFun(0.5, 1.0, 0.5, 1.5, 2.0, 20.0)
rotmotor1.SetMotorFunction(rotmotor1_func)


rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  
                     truss,  
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
                     
                     )
sys.Add(rotmotor1)