import math as m  
import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as mklsolver  
import pychrono.irrlicht as chronoirr  
import os  


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__() 
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            
            
            
            
            
            
            if x < 0.3: 
                 return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0
            elif x <=0.5: 
                 return -chrono.CH_PI 
            else: 
                 return chrono.CH_PI



out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)


sys = chrono.ChSystemSMC() 


L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001) 


body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)


boxtruss_vis = chrono.ChVisualShapeBox(0.03, 0.25, 0.15) 
body_trss.AddVisualShape(boxtruss_vis, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5) 
sys.AddBody(body_crank)


boxcrank_vis = chrono.ChVisualShapeBox(K, 0.05, 0.03) 
body_crank.AddVisualShape(boxcrank_vis)


motor = chrono.ChLinkMotorRotationAngle() 
motor.Initialize(body_trss, body_crank, chrono.ChFramed(vG)) 
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun) 
sys.AddLink(motor) 


mesh = fea.ChMesh()


beam_wy = 0.12 
beam_wz = 0.15 


minertia = fea.ChInertiaCosseratSimple() 
minertia.SetAsRectangularSection(beam_wy, beam_wz)
minertia.SetDensity(2700) 
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity) 
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz * 0.5) 


builder_iga = fea.ChBuilderBeamIGA()

builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.ChVector3d(0, 0, 1), 3)


builder_iga.GetLastBeamNodes()[0].SetFixed(True) 

node_tip = builder_iga.GetLastBeamNodes()[-1]

num_iga_nodes = len(builder_iga.GetLastBeamNodes())
node_mid = builder_iga.GetLastBeamNodes()[num_iga_nodes // 2]



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


constr_bb.Initialize(node_tip, node_top, False, node_tip.Frame(), node_top.Frame())
sys.AddLink(constr_bb) 

constr_bb.SetConstrainedCoords(True, False, True, False, False, False) 


sphereconstr2_vis = chrono.ChVisualShapeSphere(0.02) 
constr_bb.AddVisualShape(sphereconstr2_vis)


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


constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.AddLink(constr_cbd) 



constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.AddLink(constr_bc) 
constr_bc.SetConstrainedCoords(True, True, False, True, True, False) 


sphereconstr3_vis = chrono.ChVisualShapeSphere(0.01) 
constr_bc.AddVisualShape(sphereconstr3_vis)


mesh.SetAutomaticGravity(True) 
sys.Add(mesh)


mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY) 
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(True) 
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShape(mvisualizebeamA) 

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)

mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS) 
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) 
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01) 
mvisualizebeamC.SetZbufferHide(False) 
mesh.AddVisualShape(mvisualizebeamC) 


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768) 
vis.SetWindowTitle('Corrected Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.8)) 
vis.AddTypicalLights()










sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL) 



ts = chrono.ChTimestepperHHT(sys) 

ts.SetAlpha(-0.2)  
ts.SetMaxiters(10) 
ts.SetAbsoler(1e-4) 
ts.SetMode(chrono.ChTimestepperHHT.ACCELERATION) 
ts.SetVerbose(False) 
sys.SetTimestepper(ts)


time_step = 0.002
sim_time = 2.0 


while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.2, 0.2, 10, 10, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChColor(0.3,0.3,0.3)) 
    vis.EndScene()
    sys.DoStepDynamics(time_step)

    if sys.GetChTime() > sim_time:
        vis.GetDevice().closeDevice() 

del sys