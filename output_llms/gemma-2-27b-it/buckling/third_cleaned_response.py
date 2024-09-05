import math as m  
import pychrono as chrno  
import pychrono.fea as fea  
import pychrono.pardisomkl as mklsolver  
import pychrono.irrlicht as chronicls  
import os  


class ChFunctionMyFun(chrno.ChFunction):
    def __init__(self):
        chrno.ChFunction.__init__(self)
    def GetVal(self, x):
        if x > 0.5:
            return chrno.CH_PI
        else:
            return -chrno.CH_PI * (1.0 - m.cos(chrno.CH_PI * x / 0.3)) / 2.0


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
body_trss.AddVisualShape(boxtruss, chrno.ChFrame(chrno.ChVector3d(-0.01, 0, 0), chrno.QUNIT))


body_crank = chrno.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)


boxcrank = chrno.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrno.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrno.ChFrame(vG))
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


section2 = fea.ChBeamSectionAdvancedEuler()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrno.ChVector3d(1, 0, 0))


node_top = builderA.GetLastBeamNodes()[1]
node_down = builderA.GetLastBeamNodes()[-1]


constr_bb = chrno.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)


sphereconstr2 = chrno.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)


builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrno.ChVector3d(0, 1, 0))


node_crnkG = builderB.GetLastBeamNodes()[0]
node_
print("error happened with only start ```python")