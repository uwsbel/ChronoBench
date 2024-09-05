import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrno  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronicls  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrno.ChFunction):
    def __init__(self):
        chrno.ChFunction.__init__(self)
    def GetVal(self, x):
        if x > 0.5:
            return chrno.CH_PI
        else:
            return -chrno.CH_PI * (1.0 - m.cos(chrno.CH_PI * x / 0.3)) / 2.0

# Define the output directory path
out_dir = chrno.GetChronoOutputPath() + "BEAM_FAILED"
os.makedirs(out_dir, exist_ok=True)

# Create a Chrono::Engine physical system
sys = chrno.ChSystemSMC()

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrno.ChVector3d(0, 0, 0)
vC = chrno.ChVector3d(L, 0, 0)
vB = chrno.ChVector3d(L, -H, 0)
vG = chrno.ChVector3d(L - K, -H, 0)
vd = chrno.ChVector3d(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_truss = chrno.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

# Attach a visualization shape to the truss
boxtruss = fea.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(boxtruss, chrno.ChFrame(chrno.ChVectorD(-0.01, 0, 0), chrno.QUNIT))

# Create a crank body:
body_crank = chrno.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)

# Attach a visualization shape to the crank
boxcrank = fea.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrno.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrno.ChFrame(vG))
myfun = ChFunctionMyFun()
motor.SetTorqueFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosserat()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosserat()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrno.Vect_X, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[65]
node_mid = builder_iga.GetLastBeamNodes()[32]

# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrno.ChVectorD(1, 0, 0))

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[1]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the horizontal and vertical beams
constr_bb = chrno.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.GetFrame(), node_top.GetFrame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)

# Attach a visualization shape for the constraint
sphereconstr2 = fea.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)

# Create a crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrno.ChVectorD(0, 1, 0))

# Define nodes at the ends of the crank beam
node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the body crank
constr_cbd = chrno.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.GetFrame(), node_crnkG.GetFrame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrno.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.GetFrame(), node_crankB.GetFrame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)

# Attach a visualization shape for the constraint
sphereconstr3 = fea.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Create visualization for the FEM mesh:
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

# Create the Irrlicht visualization system
vis = chronicls.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrno.GetChronoDataFile('logo_pychrono_alpha.jpg'))
vis.AddSkyBox()
vis.AddCamera(chrno.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)

# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrno.ChTimestepperEulerExplicit(sys)
ts.SetStepControl(True)
sys.SetTimestepper(ts)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronicls.drawGrid(vis, 0.1, 0.01, 10, 10, chrno.ChCoordsysd(chrno.ChVector3d(0, 0, 0), chrno.QUNIT))
    vis.EndScene()
    sys.DoStepDynamics(0.002)