import math as m
import pychrono as chrono  # Fixed alias consistency
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as chronoirr  # Fixed alias
import os

# Custom function class for motor angle
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0

# Define output directory
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create physical system (fixed typo)
sys = chrono.ChSystemSMC()  # Corrected class name

# Define geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# Create fixed truss body
body_truss = chrono.ChBody()  # Fixed variable name consistency
body_truss.SetFixed(True)
sys.Add(body_truss)  # Corrected method name

# Attach visualization to truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVector3d(-0.01, 0, 0)))  # Fixed frame class

# Create crank body
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)  # Corrected method name

# Attach visualization to crank
boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))  # Fixed frame class
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)  # Corrected function type
sys.Add(motor)

# Create FEM mesh
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section for IGA beam
minertia = fea.ChInertiaCosseratSimple()  # Corrected class name
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # Corrected class
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

# Fix first node and get reference nodes
nodes_iga = builder_iga.GetLastBeamNodes()
nodes_iga[0].SetFixed(True)  # Use list indexing
node_tip = nodes_iga[-1]  # Last node
node_mid = nodes_iga[len(nodes_iga)//2]  # Middle node

# Define vertical beam parameters (Euler)
section2 = fea.ChBeamSectionEulerAdvanced()  # Corrected class
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build vertical beam
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))

# Get nodes from vertical beam
nodes_euler = builderA.GetLastBeamNodes()
node_top = nodes_euler[0]  # First node
node_down = nodes_euler[-1]  # Last node

# Create constraint between beams
constr_bb = chrono.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, chrono.False, 
                    node_top.GetFrameRefToAbs(), node_tip.GetFrameRefToAbs())  # Corrected initialization
sys.Add(constr_bb)

# Add constraint visualization
sphereconstr2 = chrono.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)

# Create crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build crank beam
builderB = fea.ChBuilderBeamEuler()  # Fixed module reference
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))

# Get crank beam nodes
nodes_crank = builderB.GetLastBeamNodes()
node_crnkG = nodes_crank[0]
node_crankB = nodes_crank[-1]

# Create constraint: crank beam to body
constr_cbd = chrono.ChLinkMateGeneric(True, True, True, True, True, True)  # Corrected initialization
constr_cbd.Initialize(node_crnkG, body_crank, 
                     chrono.False, node_crnkG.GetFrameRefToAbs(), node_crnkG.GetFrameRefToAbs())
sys.Add(constr_cbd)

# Create constraint: vertical beam to crank beam
constr_bc = chrono.ChLinkMateGeneric(True, True, False, True, True, False)  # Corrected initialization
constr_bc.Initialize(node_down, node_crankB, 
                    chrono.False, node_crankB.GetFrameRefToAbs(), node_crankB.GetFrameRefToAbs())
sys.Add(constr_bc)

# Add constraint visualization
sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

# Final setup
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Create mesh visualization
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

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Fixed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Fixed extension
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Setup solver
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)

# Setup HHT timestepper
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht_stepper = sys.GetTimestepper().AsTimestepperHHT()
hht_stepper.SetAlpha(-0.2)
hht_stepper.SetStepControl(True)
hht_stepper.SetTolerance(1e-6)
hht_stepper.SetMaxIters(100)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.RenderGrid(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 0.1, 10)  # Fixed grid rendering
    vis.EndScene()
    sys.DoStepDynamics(0.002)