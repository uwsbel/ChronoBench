import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Define the custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def Get_y(self, x):
        if x <= self.T1:
            return self.A1 * x
        elif self.T1 < x <= self.T2:
            return self.A1 * self.T1
        elif self.T2 < x <= self.T3:
            return self.A1 * self.T1 - self.A2 * (x - self.T2)
        else:
            return self.w

# Rest of the script remains the same

# ...

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Initialize the custom motor function
f_custom = ChFunctionMyFun(A1=2, A2=0.1, T1=1, T2=2, T3=3, w=10)
rotmotor1.SetMotorFunction(f_custom)

# ...

# Set up the HHT timestepper
#sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# ...

# The rest of the script remains the same