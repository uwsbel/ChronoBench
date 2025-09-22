import math as m  
import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as mklsolver  
import pychrono.irrlicht as chronicls  
import os  


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0


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


body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)


boxtruss_shape = chrono.ChBoxShape(chrono.ChVectorD(0.03, 0.25, 0.15))
body_trss.AddVisualShape(boxtruss_shape, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)


boxcrank_shape = chrono.ChBoxShape(chrono.ChVectorD(K * 0.5, 0.05 * 0.5, 0.03 * 0.5))
body_crank.AddVisualShape(boxcrank_shape)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)
sys.AddLink(motor)


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
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

last_beam_nodes = builder_iga.GetLastBeamNodes()

last_beam_nodes[0].SetFixed(True)


node_tip = last_beam_nodes[-1]


node_mid = last_beam_nodes[len(last_beam_nodes)//2]


section2 = fea.ChBeamSectionAdvancedEuler()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.0)
section2.SetAsCircularSection(hbeam_d)


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))

last_vertical_nodes = builderA.GetLastBeamNodes()
if len(last_vertical_nodes) < 2:
    raise RuntimeError("Not enough vertical beam nodes built")

node_top = last_vertical_nodes[0]
node_down = last_vertical_nodes[-1]


constr_bb = chrono.ChLinkMateParallel()

constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_tip.Frame())
sys.AddLink(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)



sphere_node_top = chrono.ChBodyEasySphere(0.02, 100, True, True)
sphere_node_top.SetPos(node_top.GetPos())
sphere_node_top.SetBodyFixed(True)
sys.Add(sphere_node_top)


sphere_node_tip = chrono.ChBodyEasySphere(0.02, 100, True, True)
sphere_node_tip.SetPos(node_tip.GetPos())
sphere_node_tip.SetBodyFixed(True)
sys.Add(sphere_node_tip)


section3 = fea.ChBeamSectionAdvancedEuler()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.0)
section3.SetAsCircularSection(crankbeam_d)


builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))

last_crank_nodes = builderB.GetLastBeamNodes()

node_crnkG = last_crank_nodes[0]
node_crankB = last_crank_nodes[-1]


constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.AddLink(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_down.Frame(), node_crankB.Frame())
sys.AddLink(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)


sphere_node_down = chrono.ChBodyEasySphere(0.01, 100, True, True)
sphere_node_down.SetPos(node_down.GetPos())
sphere_node_down.SetBodyFixed(True)
sys.Add(sphere_node_down)

sphere_node_crankB = chrono.ChBodyEasySphere(0.01, 100, True, True)
sphere_node_crankB.SetPos(node_crankB.GetPos())
sphere_node_crankB.SetBodyFixed(True)
sys.Add(sphere_node_crankB)


mesh.SetAutomaticGravity(True)
sys.Add(mesh)


mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_VECTORS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_FULL)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)


vis = chronicls.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.jpg'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))
vis.AddTypicalLights()


custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)


ts = chrono.ChTimestepperHHT()
ts.SetStepControl(True)
sys.SetTimestepper(ts)

time_step = 0.002


while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronicls.drawGrid(vis, 0.1, 0.01, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(time_step)