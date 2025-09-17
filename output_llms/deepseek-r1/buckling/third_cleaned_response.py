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
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0


out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"


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
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVector3d(-0.01, 0, 0)))  


body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)


boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
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
melasticity.SetShearModulus(72.0e9 / (2 * (1 + 0.35)))  
msection1 = fea.ChBeamSectionCosserat(melasticity, minertia)  
msection1.SetDrawThickness(beam_wy, beam_wz)  


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
nodes_iga = builder_iga.GetLastBeamNodes()
node_tip = nodes_iga[-1]  
node_mid = nodes_iga[len(nodes_iga)//2]  


section2 = fea.ChBeamSectionEulerAdvanced()  
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulus(75.0e9 / (2 * (1 + 0.25)))  
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
nodes_euler = builderA.GetLastBeamNodes()
node_top = nodes_euler[0]   
node_down = nodes_euler[-1]  


constr_bb = fea.ChLinkNodeNode()  
constr_bb.Initialize(node_top, node_tip)
sys.Add(constr_bb)


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulus(75.0e9 / (2 * (1 + 0.25)))
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)


builderB = fea.ChBuilderBeamEuler()  
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
nodes_crank = builderB.GetLastBeamNodes()
node_crnkG = nodes_crank[0]    
node_crankB = nodes_crank[-1]  


constr_cbd = fea.ChLinkNodeBody()  
constr_cbd.Initialize(node_crnkG, body_crank)
sys.Add(constr_cbd)


constr_bc = fea.ChLinkNodeNode()  
constr_bc.Initialize(node_down, node_crankB)
sys.Add(constr_bc)


sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))  
sys.Add(mesh)


mvisualizebeamA = fea.ChVisualShapeFEA(mesh)  
mvisualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(True)
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
vis.SetWindowTitle('Fixed Beam Simulation')
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
hht_ts.SetStepControl(True)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.DrawGrid(0.1, 0.1, 10, 10,  
                 chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                 True, True)
    vis.EndScene()
    sys.DoStepDynamics(0.002)