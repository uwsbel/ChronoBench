import math as m
import pychrono as chrono  
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as irrlicht  
import os


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()  
    def Get_y(self, x):  
        if x > 0.5:
            return chrono.CH_C_PI
        else:
            return -chrono.CH_C_PI * (1.0 - m.cos(chrono.CH_C_PI * x / 0.3)) / 2.0


out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"


sys = chrono.ChSystemSMC()  


L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)  
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)


body_truss = chrono.ChBody()  
body_truss.SetBodyFixed(True)  
sys.Add(body_truss)


boxtruss = chrono.ChBoxShape()  
boxtruss.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.03, 0.25, 0.15))
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)


boxcrank = chrono.ChBoxShape()  
boxcrank.GetBoxGeometry().SetLengths(chrono.ChVectorD(K, 0.05, 0.03))
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)  
sys.Add(motor)


mesh = fea.ChMesh()


beam_wy = 0.12
beam_wz = 0.15


minertia = fea.ChInertiaCosseratSimple()  
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetGshearModulus(melasticity.GetYoungModulus() / (2 * (1 + 0.35)))  
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)


builder_iga.GetLastBeamNodes()[0].SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]  
node_mid = builder_iga.GetLastBeamNodes()[15]  


section2 = fea.ChBeamSectionEulerAdvanced()  
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetGwithPoissonRatio(0.25)  
section2.SetBeamRaleyghDamping(0.000)  
section2.SetAsCircularSection(hbeam_d)


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))


node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]


constr_bb = chrono.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)


sphereconstr2 = chrono.ChSphereShape()  
sphereconstr2.GetSphereGeometry().rad = 0.02
constr_bb.AddVisualShape(sphereconstr2)


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetGwithPoissonRatio(0.25)  
section3.SetBeamRaleyghDamping(0.000)  
section3.SetAsCircularSection(crankbeam_d)


builderB = fea.ChBuilderBeamEuler()  
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))


node_crankG = builderB.GetLastBeamNodes()[0]  
node_crankB = builderB.GetLastBeamNodes()[-1]


constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)


sphereconstr3 = chrono.ChSphereShape()  
sphereconstr3.GetSphereGeometry().rad = 0.01
constr_bc.AddVisualShape(sphereconstr3)


mesh.SetAutomaticGravity(True)
sys.Add(mesh)


mvisualizebeamA = fea.ChVisualizationFEAmesh(mesh)  
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddAsset(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualizationFEAmesh(mesh)  
mvisualizebeamC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_VECT_SPEED)
mvisualizebeamC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddAsset(mvisualizebeamC)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))
vis.AddTypicalLights()


custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)


ts = chrono.ChTimestepperHHT(sys)  
ts.SetStepControl(True)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    irrlicht.drawGrid(vis, 0.1, 0.01, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.002)