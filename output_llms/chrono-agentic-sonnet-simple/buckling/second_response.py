import math as m  # Import the math library
import os  # Import OS for environment + frame dirs
import pychrono as chrono  # Main PyChrono library
import pychrono.fea as fea  # FEA module
import pychrono.pardisomkl as pardiso  # Pardiso solver
import pychrono.irrlicht as chronoirr  # Irrlicht visualization

# Custom function class for motor angle
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0

# Create a Chrono physical system
sys = chrono.ChSystemSMC()

# Key geometrical parameters
L = 1.2   # length (modified from 1)
H = 0.3   # height (modified from 0.25)
K = 0.07  # crank length (modified from 0.05)

vA = chrono.ChVector3d(0, 0, 0)          # Point A
vC = chrono.ChVector3d(L, 0, 0)          # Point C
vB = chrono.ChVector3d(L, -H, 0)         # Point B
vG = chrono.ChVector3d(L - K, -H, 0)     # Point G
vd = chrono.ChVector3d(0, 0, 0.0001)     # Small offset vector

# Create a truss body, fixed in space
body_truss = chrono.ChBody()
body_truss.SetFixed(True)  # Make the truss immobile
sys.AddBody(body_truss)

# Attach visualization shape to the truss (modified dimensions)
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)  # modified from (0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  # Position of the crank body
sys.AddBody(body_crank)

# Attach visualization shape to the crank (modified dimensions)
boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)  # modified from (K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  # motor between truss and crank
myfun = ChFunctionMyFun()  # custom angle function
motor.SetAngleFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container
mesh = fea.ChMesh()

# Horizontal beam parameters (modified widths)
beam_wy = 0.12   # width in Y direction (modified from 0.10)
beam_wz = 0.012  # width in Z direction (modified from 0.01)

# Section properties for the IGA (Cosserat) horizontal beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  # rectangular section with density
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  # Young's modulus
melasticity.SetShearModulusFromPoisson(0.3)  # shear modulus from Poisson
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  # section dimensions
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # beam section
msection1.SetDrawThickness(beam_wy, beam_wz)  # drawing thickness

# Build the IGA horizontal beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)  # 32 spans, cubic order

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]  # tip node
node_mid = builder_iga.GetLastBeamNodes()[17]   # mid node

# Vertical beam parameters - Euler beam, modified diameter and element count
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03  # diameter (modified from 0.024)
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build vertical beam with 6 Euler elements (modified from 3)
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))

# Keep reference to node container (SWIG GC pitfall)
beam_nodes_A = builderA.GetLastBeamNodes()
node_top = beam_nodes_A[0]    # top of vertical beam
node_down = beam_nodes_A[-1]  # bottom of vertical beam

# Constraint between horizontal beam tip and vertical beam top (3 DOF translational)
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)  # constrain x, y, z only

# Visualization sphere for this constraint (modified size)
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)  # modified from 0.01
constr_bb.AddVisualShape(sphereconstr2)

# Crank beam parameters - Euler beam, modified diameter and element count
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054  # diameter (modified from 0.048)
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build crank beam with 5 Euler elements (modified from 3)
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))

# Keep reference to node container (SWIG GC pitfall)
beam_nodes_B = builderB.GetLastBeamNodes()
node_crankG = beam_nodes_B[0]   # crank beam at G end
node_crankB = beam_nodes_B[-1]  # crank beam at B end

# Constraint between crank beam and body_crank (all 6 DOF)
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Constraint between vertical beam bottom and crank beam B end (5 DOF)
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

# Visualization sphere for this constraint (modified size)
sphereconstr3 = chrono.ChVisualShapeSphere(0.014)  # modified from 0.01
constr_bc.AddVisualShape(sphereconstr3)

# Disable automatic gravity on FEA elements
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# FEA visualization - surface/scalar field (beam moment Mx)
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)  # beam moment Mx
mvisualizebeamA.SetColorscaleMinMax(-500, 500)  # color scale
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

# FEA visualization - node coordinate system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node coordinate systems
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.015)  # modified from 0.01
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()  # Initialize FIRST (Irrlicht call order)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))  # modified camera position
vis.AddTypicalLights()

# Pardiso MKL solver for stiff beam stiffness matrices
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

# HHT timestepper - canonical-minimal form
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

time_step = 0.001  # timestep for stiff beams
render_fps = 60    # render framerate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # render cadence

# Main simulation loop - pure-Irrlicht FEA scene, no stdout output
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # draw grid
    vis.EndScene()
    sys.DoStepDynamics(time_step)
