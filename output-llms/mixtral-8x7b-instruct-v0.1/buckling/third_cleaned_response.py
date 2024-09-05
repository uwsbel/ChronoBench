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
os.makedirs(out_dir, exist_ok=True)


sys = chrno.ChSystemSMC()


L = 1.2
H = 0.4
K = 0.07
vA = chrno.ChVector3d(0, 0, 0)
vC = chrno.ChVector3d(L, 0, 0)
vB = chrno.ChVector3d(L, -H, 0)
vG = chrno.ChVector3d(L - K, -H, 0)
vd = chrno.ChVector3d(0, 0, 0.0001)


body_truss = chrno.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)


boxtruss = fea.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(boxtruss, chrno.ChFrame(chrno.ChVectorD(-0.01, 0, 0), chrno.QUNIT))


body_crank = chrno.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)


boxcrank = fea.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrno.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrno.ChFrame(vG))
myfun = ChFunctionMyFun()
motor.SetTorqueFunction(myfun)
sys.Add(motor)


mesh = fea.ChMesh()


beam_wy = 0.12
beam_wz = 0.15


minertia = fea.ChInertiaCosserat()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosserat()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrno.Vect_X, 3)


builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[65]
node_mid = builder_iga.GetLastBeamNodes()[32]


section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrno.ChVectorD(1, 0, 0))


node_top = builderA.GetLastBeamNodes()[1]
node_down = builderA.GetLastBeamNodes()[-1]


constr_bb = chrno.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.GetFrame(), node_top.GetFrame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)


sphereconstr2 = fea.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)


builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrno.ChVectorD(0, 1, 0))


node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]


constr_cbd = chrno.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.GetFrame(), node_crnkG.GetFrame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)


constr_bc = chrno.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.GetFrame(), node_crankB.GetFrame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)


sphereconstr3 = fea.ChVisualShapeSphere(0.01)
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
mvisualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_FULL)
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
vis.AddCamera(chrno.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()


custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)


ts = chrno.ChTimestepperEulerExplicit(sys)
ts.SetStepControl(True)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronicls.drawGrid(vis, 0.1, 0.01, 10, 10, chrno.ChCoordsysd(chrno.ChVector3d(0, 0, 0), chrno.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.002)