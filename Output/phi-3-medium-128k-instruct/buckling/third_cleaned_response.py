import math as m  
import pychrono as chrno  
import pychrono.fea as fea  
import pychrono.pardisomkl as mklsolver  
import pychrono.irrlicht as chronicls  
import os  


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0


out_dir = chrno.GetChronoOutputPath() + "BEAM_FAILED"


sys = chrno.ChSystemSMC()


L = 1.2
H = 0.4
K = 0.07
vA = chrno.ChVector3d(0, 0, 0)
vC = chrno.ChVector3d(L, 0, 0)
vB = chrno.ChVector3d(L, -H, 0)
vG = chrno.ChVector3d(L - K, -H, 0)
vd = chrno.ChVector3d(0, 0, 0.0001)


body_trss = chrno.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)


boxtruss = chrno.ChVisualShapeBox(0.03, 0.25, 0.15)
body_trss.AddVisualShape(boxtruss, chrno.ChFramed(chrno.ChVector3d(-0.01, 0, 0), chrno.QUNIT))


body_crank = chrno.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)


boxcrank = chrno.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrno.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrno.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetTorqueFunction(myfun)
sys.Add(motor)


mesh = fea.ChMesh()


beam_wy = 0.12
beam_wz = 0.15


minertia = fea.ChIneritaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrno.VECT_X, 3)


builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[65]
node_mid = builder_iga.GetLastBeamNodes()[32]
print("error happened with only start ```python")