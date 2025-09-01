import math as m
import pychrono as chrono  # Corrected import name
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as irrlicht  # Corrected import name
import os

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()  # Corrected super() call
    def Get_y(self, x):  # Corrected method name
        if x > 0.5:
            return chrono.CH_C_PI
        else:
            return -chrono.CH_C_PI * (1.0 - m.cos(chrono.CH_C_PI * x / 0.3)) / 2.0

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()  # Corrected class name

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)  # Corrected vector class
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()  # Corrected variable name
body_truss.SetBodyFixed(True)  # Corrected method name
sys.Add(body_truss)

# Attach a visualization shape to the truss
boxtruss = chrono.ChBoxShape()  # Corrected class name
boxtruss.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.03, 0.25, 0.15))
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)

# Attach a visualization shape to the crank
boxcrank = chrono.ChBoxShape()  # Corrected class name
boxcrank.GetBoxGeometry().SetLengths(chrono.ChVectorD(K, 0.05, 0.03))
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)  # Corrected method name
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()  # Corrected class name
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetGshearModulus(melasticity.GetYoungModulus() / (2 * (1 + 0.35)))  # Corrected Poisson ratio calculation
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # Corrected class name
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes()[0].SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]  # Corrected index
node_mid = builder_iga.GetLastBeamNodes()[15]  # Corrected index

# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionEulerAdvanced()  # Corrected class name
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetGwithPoissonRatio(0.25)  # Corrected method name
section2.SetBeamRaleyghDamping(0.000)  # Corrected method name
section2.SetAsCircularSection(hbeam_d)

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the horizontal and vertical beams
constr_bb = chrono.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)

# Attach a visualization shape for the constraint
sphereconstr2 = chrono.ChSphereShape()  # Corrected class name
sphereconstr2.GetSphereGeometry().rad = 0.02
constr_bb.AddVisualShape(sphereconstr2)

# Create a crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetGwithPoissonRatio(0.25)  # Corrected method name
section3.SetBeamRaleyghDamping(0.000)  # Corrected method name
section3.SetAsCircularSection(crankbeam_d)

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler()  # Corrected class name
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))

# Define nodes at the ends of the crank beam
node_crankG = builderB.GetLastBeamNodes()[0]  # Corrected variable name
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the body crank
constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)

# Attach a visualization shape for the constraint
sphereconstr3 = chrono.ChSphereShape()  # Corrected class name
sphereconstr3.GetSphereGeometry().rad = 0.01
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = fea.ChVisualizationFEAmesh(mesh)  # Corrected class name
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddAsset(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualizationFEAmesh(mesh)  # Corrected class name
mvisualizebeamC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_VECT_SPEED)
mvisualizebeamC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddAsset(mvisualizebeamC)

# Create the Irrlicht visualization system
vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)

# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT(sys)  # Corrected timestepper
ts.SetStepControl(True)
sys.SetTimestepper(ts)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    irrlicht.drawGrid(vis, 0.1, 0.01, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.002)