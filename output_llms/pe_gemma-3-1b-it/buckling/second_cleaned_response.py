import math as m
import pychrono as chrono
import pychrono.robot as fea
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as carro
import pychrono.fea as fea
import pychrono.ard as ard
import pychrono.physics as phy
import pychrono.control as c


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
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos(chrono.ChVector3d(vB + vG) * 0.5)  
sys.AddBody(body_crank)  


universal_joint = chrono.ChLinkUniversal()
universal_joint.Initialize(body_truss, body_crank, chrono.VECT_Y)
sys.Add(universal_joint)


revolution_angles = [chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                   chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)]


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, revolution_angles)
sys.Add(motor)


arm = chrono.ChBody()
arm.SetPos(chrono.ChVector3d(vB + vG, 0))
arm.SetMass(100)
arm.SetInertia(chrono.ChVector3d(1, 0, 0))
arm.SetFixed(True)
sys.Add(arm)


vehicle = chrono.ChBody()
vehicle.SetPos(chrono.ChVector3d(vA, 0))
vehicle.SetMass(100)
vehicle.SetInertia(chrono.ChVector3d(0, 1, 0))
vehicle.SetFixed(True)
sys.Add(vehicle)


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)  










body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos(chrono.ChVector3d(vB + vG) * 0.5)
sys.AddBody(body_crank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, revolution_angles)
sys.Add(motor)


arm = chrono.ChBody()
arm.SetPos(chrono.ChVector3d(vB + vG, 0))
arm.SetMass(100)
arm.SetInertia(chrono.ChVector3d(0, 1, 0))
arm.SetFixed(True)
sys.Add(arm)


vehicle = chrono.ChBody()
vehicle.SetPos(chrono.ChVector3d(vA, 0))
vehicle.SetMass(100)
vehicle.SetInertia(chrono.ChVector3d(0, 1, 0))
vehicle.SetFixed(True)
sys.Add(vehicle)


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)