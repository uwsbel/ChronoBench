import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
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

vA = chrono.ChVectorD(0, 0, 0)
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)


body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.Add(body_truss)


boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.015, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.Add(body_crank)


boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG, chrono.QUNIT))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)


mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)


beam_wy = 0.12
beam_wz = 0.012

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.ChVectorD(0,1,0), 3)


first_node = builder_iga.GetLastBeamNodes()[0]
first_node.SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]


section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVectorD(1,0,0))

node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVectorD(0,1,0))

node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]


constr_bb = chrono.ChLinkNode()
constr_bb.Initialize(node_top, node_tip)
sys.Add(constr_bb)

constr_cbd = chrono.ChLinkNode()
constr_cbd.Initialize(node_crankG, body_crank)
sys.Add(constr_cbd)

constr_bc = chrono.ChLinkNode()
constr_bc.Initialize(node_down, node_crankB)
sys.Add(constr_bc)


sphereconstr2 = chrono.ChVisualShapeSphere(0.012)
constr_bb.AddVisualShape(sphereconstr2)

sphereconstr3 = chrono.ChVisualShapeSphere(0.014)
constr_bc.AddVisualShape(sphereconstr3)


mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetSymbolsScale(0.015)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCamera(chrono.ChVectorD(0.0, 0.7, -1.2))

sys.Add(mesh)


pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0.0, 0.7, -1.2))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20, chrono.ChFrameD())
    vis.EndScene()
    sys.DoStepDynamics(0.001)