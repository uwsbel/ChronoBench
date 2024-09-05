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

# ... (rest of the code remains the same)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

rotmotor1_func = ChFunctionMyFun(0.5, 1.0, 0.5, 1.5, 2.0, 20.0)
rotmotor1.SetMotorFunction(rotmotor1_func)

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
sys.Add(rotmotor1)

# ... (rest of the code remains the same)

# Uncomment the following line to use a more precise HHT timestepper
# sys.SetTimestepperType(chrono.ChTimestepperHHT_CrankNicolson_Type)

# ... (rest of the code remains the same)