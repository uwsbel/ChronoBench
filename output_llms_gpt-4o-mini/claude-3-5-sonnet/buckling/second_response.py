import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def Get_y(self, x):
        if x > 0.4:
            return chrono.CH_C_PI
        else:
            return -chrono.CH_C_PI * (1.0 - m.cos(chrono.CH_C_PI * x / 0.4)) / 2.0

out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

sys = chrono.ChSystemSMC()

# Updated geometry parameters
L = 1.2
H = 0.3
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)

body_truss = chrono.ChBody()
body_truss.SetBodyFixed(True)
sys.Add(body_truss)

# Updated truss visualization
boxtruss = chrono.ChBoxShape()
boxtruss.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.03, 0.25, 0.12))
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))

body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.Add(body_crank)

# Updated crank visualization
boxcrank = chrono.ChBoxShape()
boxcrank.GetBoxGeometry().SetLengths(chrono.ChVectorD(K, 0.03, 0.03))
body_crank.AddVisualShape(boxcrank)

motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)

mesh = fea.ChMesh()

# Updated horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.012

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetShearModulus(melasticity.GetYoungModulus() / (2 * (1 + 0.3)))
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)

builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Updated vertical beam parameters
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetGwithPoissonRatio(0.3)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))

node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)

# Updated constraint visualization
sphereconstr2 = chrono.ChSphereShape()
sphereconstr2.GetSphereGeometry().rad = 0.012
constr_bb.AddVisualShape(sphereconstr2)

# Updated crank beam parameters
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetGwithPoissonRatio(0.3)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))

node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

# Updated constraint visualization
sphereconstr3 = chrono.ChSphereShape()
sphereconstr3.GetSphereGeometry().rad = 0.014
constr_bc.AddVisualShape(sphereconstr3)

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

mvisualizebeamA = fea.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.015)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 0.7, -1.2))
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
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.001)