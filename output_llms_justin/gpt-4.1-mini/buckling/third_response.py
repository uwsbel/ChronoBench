import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronicls  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)

# Attach a visualization shape to the truss
boxtruss_shape = chrono.ChBoxShape(chrono.ChVectorD(0.03, 0.25, 0.15))
body_trss.AddVisualShape(boxtruss_shape, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)

# Attach a visualization shape to the crank
boxcrank_shape = chrono.ChBoxShape(chrono.ChVectorD(K * 0.5, 0.05 * 0.5, 0.03 * 0.5))
body_crank.AddVisualShape(boxcrank_shape)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)
sys.AddLink(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam (30 elements -> 31 nodes)
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

last_beam_nodes = builder_iga.GetLastBeamNodes()
# Fix the first node of the horizontal beam
last_beam_nodes[0].SetFixed(True)

# Fix node_tip index to safe range (e.g., last node)
node_tip = last_beam_nodes[-1]

# node_mid (middle node approx)
node_mid = last_beam_nodes[len(last_beam_nodes)//2]

# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionAdvancedEuler()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.0)
section2.SetAsCircularSection(hbeam_d)

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))

last_vertical_nodes = builderA.GetLastBeamNodes()
if len(last_vertical_nodes) < 2:
    raise RuntimeError("Not enough vertical beam nodes built")

node_top = last_vertical_nodes[0]
node_down = last_vertical_nodes[-1]

# Create a constraint between the horizontal and vertical beams
constr_bb = chrono.ChLinkMateParallel()
# Initialize parallel mate between frames of two nodes
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_tip.Frame())
sys.AddLink(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)

# Visualization of constraint points:
# For node_top visualization:
sphere_node_top = chrono.ChBodyEasySphere(0.02, 100, True, True)
sphere_node_top.SetPos(node_top.GetPos())
sphere_node_top.SetBodyFixed(True)
sys.Add(sphere_node_top)

# For node_tip visualization:
sphere_node_tip = chrono.ChBodyEasySphere(0.02, 100, True, True)
sphere_node_tip.SetPos(node_tip.GetPos())
sphere_node_tip.SetBodyFixed(True)
sys.Add(sphere_node_tip)

# Create a crank beam
section3 = fea.ChBeamSectionAdvancedEuler()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.0)
section3.SetAsCircularSection(crankbeam_d)

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))

last_crank_nodes = builderB.GetLastBeamNodes()

node_crnkG = last_crank_nodes[0]
node_crankB = last_crank_nodes[-1]

# Create a constraint between the crank beam and the body crank
constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.AddLink(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_down.Frame(), node_crankB.Frame())
sys.AddLink(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)

# Visualization spheres for these constraint nodes:
sphere_node_down = chrono.ChBodyEasySphere(0.01, 100, True, True)
sphere_node_down.SetPos(node_down.GetPos())
sphere_node_down.SetBodyFixed(True)
sys.Add(sphere_node_down)

sphere_node_crankB = chrono.ChBodyEasySphere(0.01, 100, True, True)
sphere_node_crankB.SetPos(node_crankB.GetPos())
sphere_node_crankB.SetBodyFixed(True)
sys.Add(sphere_node_crankB)

# Final touches:
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_VECTORS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_FULL)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization system
vis = chronicls.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.jpg'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)

# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT()
ts.SetStepControl(True)
sys.SetTimestepper(ts)

time_step = 0.002

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronicls.drawGrid(vis, 0.1, 0.01, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(time_step)