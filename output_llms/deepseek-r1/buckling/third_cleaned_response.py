import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as chronoirr


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0


sys = chrono.ChSystemSMC()  


L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)


body_truss = chrono.ChBody()  
body_truss.SetFixed(True)
sys.Add(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)


boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)  
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
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(
    mesh, 
    msection1, 
    30, 
    vA, 
    vC, 
    chrono.ChVector3d(1, 0, 0),  
    3
)


builder_iga.GetLastBeamNodes()[0].SetFixed(True)  
node_tip = builder_iga.GetLastBeamNodes()[-1]  


section2 = fea.ChBeamSectionEulerAdvanced()  
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))


node_top = builderA.GetLastBeamNodes()[0]  
node_down = builderA.GetLastBeamNodes()[-1]  


constr_bb = chrono.ChLinkMateGeneric()  
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_tip.Frame())
constr_bb.SetConstrainedCoords(True, True, True, True, True, True)  
sys.Add(constr_bb)


sphereconstr2 = chrono.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)


builderB = fea.ChBuilderBeamEuler()  
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))


node_crnkG = builderB.GetLastBeamNodes()[0]  
node_crankB = builderB.GetLastBeamNodes()[-1]  


constr_cbd = chrono.ChLinkMateGeneric()  
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), body_crank.GetFrame())
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)  
sys.Add(constr_cbd)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_down.Frame(), node_crankB.Frame())
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)  
sys.Add(constr_bc)


sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)


mesh.SetAutomaticGravity(True)
sys.Add(mesh)


mvisualizebeamA = fea.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_VECTORS)
mvisualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Mechanism Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()


solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(solver)
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)  
hht_ts = sys.GetTimestepper().AsChTimestepperHHT()
hht_ts.SetAlpha(-0.2)
hht_ts.SetMaxIters(100)
hht_ts.SetAbsTolerances(1e-6)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)