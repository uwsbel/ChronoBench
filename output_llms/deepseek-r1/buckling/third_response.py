import math as m
import pychrono as chrono
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

# Define output directory
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create physical system
sys = chrono.ChSystemSMC()  # Fixed typo in class name

# Geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# Create fixed truss body
body_truss = chrono.ChBody()  # Fixed variable name
body_truss.SetFixed(True)
sys.Add(body_truss)  # Use Add() instead of AddBody()

# Attach visualization to truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVector3d(-0.01, 0, 0)))  # Fixed frame type

# Create crank body
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)

# Attach visualization to crank
boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))  # Fixed frame type and variable name
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)  # Corrected method name
sys.Add(motor)

# Create FEM mesh
mesh = fea.ChMesh()

# Horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# IGA beam section properties
minertia = fea.ChInertiaCosseratSimple()  # Fixed class name
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulus(72.0e9 / (2 * (1 + 0.35)))  # Corrected Poisson calculation
msection1 = fea.ChBeamSectionCosserat(melasticity, minertia)  # Corrected class and order
msection1.SetDrawThickness(beam_wy, beam_wz)  # Corrected dimensions

# Build IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(
    mesh,                  # mesh to populate
    msection1,             # section material
    30,                    # number of elements
    vA,                    # start point
    vC,                    # end point
    chrono.ChVector3d(1, 0, 0),  # direction
    3                      # order (cubic)
)
nodes_iga = builder_iga.GetLastBeamNodes()
node_tip = nodes_iga[-1]  # Last node (tip)
node_mid = nodes_iga[len(nodes_iga)//2]  # Middle node

# Vertical beam parameters (Euler)
section2 = fea.ChBeamSectionEulerAdvanced()  # Corrected class name
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulus(75.0e9 / (2 * (1 + 0.25)))  # Corrected Poisson calculation
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build vertical Euler beam
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
nodes_euler = builderA.GetLastBeamNodes()
node_top = nodes_euler[0]   # First node (top)
node_down = nodes_euler[-1]  # Last node (bottom)

# Create constraint between beams
constr_bb = fea.ChLinkNodeNode()  # Use FEA constraint for nodes
constr_bb.Initialize(node_top, node_tip)
sys.Add(constr_bb)

# Create crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulus(75.0e9 / (2 * (1 + 0.25)))
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build crank beam
builderB = fea.ChBuilderBeamEuler()  # Fixed module name
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
nodes_crank = builderB.GetLastBeamNodes()
node_crnkG = nodes_crank[0]    # First node (at vG)
node_crankB = nodes_crank[-1]  # Last node (at vB)

# Constraint between crank beam and body
constr_cbd = fea.ChLinkNodeBody()  # Use FEA constraint
constr_cbd.Initialize(node_crnkG, body_crank)
sys.Add(constr_cbd)

# Constraint between vertical beam and crank beam
constr_bc = fea.ChLinkNodeNode()  # Use FEA constraint
constr_bc.Initialize(node_down, node_crankB)
sys.Add(constr_bc)

# Set gravity and add mesh to system
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))  # Explicit gravity setting
sys.Add(mesh)

# Visualization settings
mvisualizebeamA = fea.ChVisualShapeFEA(mesh)  # Fixed module
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

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Fixed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Solver settings
solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(solver)
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)  # Use HHT integrator
hht_ts = sys.GetTimestepper().AsChTimestepperHHT()
hht_ts.SetAlpha(-0.2)
hht_ts.SetStepControl(True)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.DrawGrid(0.1, 0.1, 10, 10,  # Fixed grid drawing
                 chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                 True, True)
    vis.EndScene()
    sys.DoStepDynamics(0.002)