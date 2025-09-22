import math as m  
import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as pardiso  
import pychrono.irrlicht as chronoirr  
import os  



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
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


boxcrank = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  
myfun = ChFunctionMyFun()  
motor.SetAngleFunction(myfun)  
sys.Add(motor)  


mesh = fea.ChMesh()


beam_wy = 0.10  
beam_wz = 0.01  


minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  
melasticity.SetShearModulus(26.0e9)  

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  
msection1.SetDrawThickness(beam_wy, beam_wz)  


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_Y, 3)  


builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]  
node_mid = builder_iga.GetLastBeamNodes()[15]  


beam_d = 0.025  
center = node_tip.GetPos() + vd  


minertia2 = fea.ChInertiaCosseratSimple()
minertia2.SetAsCircularSection(beam_d, 2700)  

melasticity2 = fea.ChElasticityCosseratSimple()
melasticity2.SetYoungModulus(73.0e9)  
melasticity2.SetShearModulus(26.0e9)  

msection2 = fea.ChBeamSectionCosserat(minertia2, melasticity2)  
msection2.SetDrawThickness(beam_d, beam_d)  


builder_iga2 = fea.ChBuilderBeamIGA()
builder_iga2.BuildBeam(mesh, msection2, 3, center, center + chrono.ChVector3d(0, 0.2, 0), chrono.VECT_Y, 3)  


node_top = builder_iga2.GetLastBeamNodes()[0]
node_down = builder_iga2.GetLastBeamNodes()[-1]


constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)  


sphereconstr2 = chrono.ChVisualShapeSphere(0.01)
constr_bb.AddVisualShape(sphereconstr2)


msection3 = fea.ChBeamSectionCosserat(minertia2, melasticity2)  
msection3.SetDrawThickness(beam_d, beam_d)  

builder_iga3 = fea.ChBuilderBeamIGA()
builder_iga3.BuildBeam(mesh, msection3, 3, vG, vB, chrono.ChVector3d(0, 1, 0))  

node_crankG = builder_iga3.GetLastBeamNodes()[0]  


constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)  


sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_cbd.AddVisualShape(sphereconstr3)


constr_bd = chrono.ChLinkLockRevolute()
constr_bd.Initialize(body_truss, builder_iga3.GetLastBeamNodes()[1], chrono.ChFramed(vG, chrono.ChVector3d(0, 0, 1)))
sys.Add(constr_bd)


sphereconstr4 = chrono.ChVisualShapeSphere(0.01)
constr_bd.AddVisualShape(sphereconstr4)


mesh.SetAutomaticGravity(False)  


sys.Add(mesh)


mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)  
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Beam buckling demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0))  
vis.AddTypicalLights()  


pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)  


ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)  


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)