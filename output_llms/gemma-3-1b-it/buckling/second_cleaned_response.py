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
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


boxtruss = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxtruss, chrono.QUNIT)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)


mesh = fea.ChMesh()


beam_wy = 0.10  
beam_wz = 0.01  


minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  
melasticity.SetShearModulusFromPoisson(0.3)  
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  

msection1 = fea.ChBeamSectionEulerAdvanced()
msection1.SetDensity(2700)  
msection1.SetYoungModulus(73.0e9)  
msection1.SetShearModulusFromPoisson(0.3)  
msection1.SetRayleighDamping(0.000)  
msection1.SetAsCircularSection(hbeam_d)  


builder_iga = fea.ChBuilderBeamEuler()
builder_iga.BuildBeam(mesh, msection1, 3, vA + vd, vC + vd, chrono.ChVector3d(0, 1, 0), 3)  


node_crankG = builderA.GetLastBeamNodes()[0]
node_crankB = builderA.GetLastBeamNodes()[-1]


constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)


sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)


mesh.SetAutomaticGravity(False)  


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Beams and constraints')  
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