import math as m
import pychrono as chrono  # Corrected alias from 'chrno' to 'chrono' for consistency
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as chronoirr
import os

# Custom function class for motor angle
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0

out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)

sys = chrono.ChSystemSMC()  # Corrected typo in ChSystemSMC

# Geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# Truss body
body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.Add(body_trss)

boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_trss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Crank body
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)

boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Motor setup (corrected to rotation angle motor)
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_trss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)  # Corrected to SetAngleFunction
sys.Add(motor)

# FEM mesh
mesh = fea.ChMesh()

# Horizontal IGA beam parameters
beam_wy = 0.12
beam_wz = 0.15

minertia = fea.ChInertiaCosseratSimple()  # Corrected typo in ChInertiaCosseratSimple
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build IGA beam (corrected Y-axis direction)
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.ChVector3d(0, 1, 0), 3)

# Corrected node indices (30 elements -> 31 nodes)
node_tip = builder_iga.GetLastBeamNodes()[-1]  # Last node
node_mid = builder_iga.GetLastBeamNodes()[15]  # Mid node
node_tip.SetFixed(True)

# Vertical Euler beam
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))

# Corrected node indices for vertical beam
node_top = builderA.GetLastBeamNodes()[0]    # First node
node_down = builderA.GetLastBeamNodes()[-1]  # Last node

# Constraints corrected to ChLinkNodeBody
constr_bb = fea.ChLinkNodeBody()
constr_bb.Initialize(node_top, body_trss)
sys.Add(constr_bb)

sphereconstr2 = chrono.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)

# Crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()  # Corrected module reference
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))

node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Corrected constraint types
constr_cbd = fea.ChLinkNodeBody()
constr_cbd.Initialize(node_crnkG, body_crank)
sys.Add(constr_cbd)

constr_bc = fea.ChLinkNodeNode()  # Connect two FEA nodes
constr_bc.Initialize(node_down, node_crankB)
sys.Add(constr_bc)

sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

sys.Add(mesh)

# Visualization
mvisualizebeamA = fea.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_VECTORS)
mvisualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_FULL)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Corrected Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.jpg'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Solver and timestepper (corrected to HHT)
solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetAlpha(-0.2)
ts.SetMaxiters(20)
ts.SetAbsTolerances(1e-5)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.ChIrrTools.drawGrid(vis, 0.1, 0.01, 10, 10, 
                                 chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.002)