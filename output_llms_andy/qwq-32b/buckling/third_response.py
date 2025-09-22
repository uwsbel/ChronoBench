import math as m
import pychrono as chrno
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as chronicls
import os

class ChFunctionMyFun(chrno.ChFunction):
    def __init__(self):
        super(ChFunctionMyFun, self).__init__()
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
vA = chrno.ChVectorD(0, 0, 0)
vC = chrno.ChVectorD(L, 0, 0)
vB = chrno.ChVectorD(L, -H, 0)
vG = chrno.ChVectorD(L - K, -H, 0)
vd = chrno.ChVectorD(0, 0, 0.0001)

body_trss = chrno.ChBody()
body_trss.SetFixed(True)
sys.Add(body_trss)

boxtruss = chrno.ChVisualShapeBox(0.03, 0.25, 0.15)
body_trss.AddVisualShape(boxtruss, chrno.ChFrameD(chrno.ChVectorD(-0.01, 0, 0), chrno.QUNIT))

body_crank = chrno.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)

boxcrank = chrno.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

motor = chrno.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrno.ChFrameD(vG, chrno.QUNIT))
myfun = ChFunctionMyFun()
motor.SetMotionFunction(myfun)
sys.Add(motor)

mesh = fea.ChMesh()

beam_wy = 0.12
beam_wz = 0.15

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrno.ChVectorD(1,0,0), 3)
builder_iga.GetLastBeamNodes()[0].SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[65]
node_mid = builder_iga.GetLastBeamNodes()[32]

section2 = fea.ChBeamSectionAdvancedEuler()
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(0.05)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrno.ChVectorD(1,0,0))
node_top = builderA.GetLastBeamNodes()[1]
node_down = builderA.GetLastBeamNodes()[-1]

constr_bb = chrno.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.GetFrame(), node_tip.GetFrame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)

sphereconstr2 = chrno.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)

section3 = fea.ChBeamSectionEulerAdvanced()
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(0.06)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrno.ChVectorD(0,1,0))
node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

constr_cbd = chrno.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.GetFrame(), node_crnkG.GetFrame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

constr_bc = chrno.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_down.GetFrame(), node_crankB.GetFrame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)

sphereconstr3 = chrno.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

mesh.SetAutomaticGravity(True)
sys.Add(mesh)

mvisualizebeamA = chrno.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrno.ChVisualShapeFEA.DataType_ELEM_BEAM_MYY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrno.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrno.ChVisualShapeFEA.GlyphType_NODE_VECTORS)
mvisualizebeamC.SetFEMdataType(chrno.ChVisualShapeFEA.DataType_FULL)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)

vis = chronicls.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrno.GetChronoDataFile('logo_pychrono_alpha.jpg'))
vis.AddSkyBox()
vis.AddCamera(chrno.ChVectorD(0.0, 1.0, -1.5))
vis.AddTypicalLights()

custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)
ts = chrno.ChTimestepperHHT(sys)
ts.SetAlpha(-0.2)  # HHT alpha parameter
ts.SetMaxiters(10)
ts.SetTolerance(1e-6)
sys.SetTimestepper(ts)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronicls.drawGrid(vis, 0.1, 0.01, 10, 10, chrno.ChCoordsysD(chrno.ChVectorD(0,0,0), chrno.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.002)