import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Updated geometry parameters
L = 1.2  # Changed from 1.0
H = 0.3  # Changed from 0.25
K = 0.07  # Changed from 0.05
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# Create a truss body
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

# Updated truss visualization dimensions
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)  # Changed from (0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))  # Fixed ChFramed->ChFrameD

# Create a crank body
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)

# Updated crank visualization dimensions
boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)  # Changed from (K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))  # Fixed ChFramed->ChFrameD
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container
mesh = fea.ChMesh()

# Updated horizontal beam parameters
beam_wy = 0.12  # Changed from 0.10
beam_wz = 0.012  # Changed from 0.01

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.ChVector3d(0,1,0), 3)  # Fixed VECT_Y->ChVector3d(0,1,0)

# Fix the first node
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Updated vertical beam parameters
hbeam_d = 0.03  # Changed from 0.024
section2 = fea.ChBeamSectionEulerAdvanced()
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build vertical beam with updated element count
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))  # Changed from 3 elements

node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create constraint between beams
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)

# Updated constraint visualization size
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)  # Changed from 0.01
constr_bb.AddVisualShape(sphereconstr2)

# Updated crank beam parameters
crankbeam_d = 0.054  # Changed from 0.048
section3 = fea.ChBeamSectionEulerAdvanced()
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build crank beam with updated element count
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))  # Changed from 3 elements

node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create constraint between crank beam and body
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create constraint between vertical and crank beams
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

# Updated constraint visualization size
sphereconstr3 = chrono.ChVisualShapeSphere(0.014)  # Changed from 0.01
constr_bc.AddVisualShape(sphereconstr3)

# Final setup
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Visualization for FEM mesh
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.015)  # Changed from 0.01
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))  # Changed position
vis.AddTypicalLights()

# Solver and timestepper
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.001)