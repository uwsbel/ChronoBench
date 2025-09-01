import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as pardiso  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# -------------------------
# ERROR FIXES AND INSTRUCTIONS APPLIED BELOW
# -------------------------

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

# -------------------------
# 1. Geometry Parameters Update
# -------------------------
L = 1.2      # Length (was 1)
H = 0.3      # Height (was 0.25)
K = 0.07     # Crank length (was 0.05)
vA = chrono.ChVector3d(0, 0, 0)  # Point A
vC = chrono.ChVector3d(L, 0, 0)  # Point C
vB = chrono.ChVector3d(L, -H, 0)  # Point B
vG = chrono.ChVector3d(L - K, -H, 0)  # Point G
vd = chrono.ChVector3d(0, 0, 0.0001)  # Small offset vector

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

# -------------------------
# 2. Truss Body Visualization
# -------------------------
# Corrected: ChVisualShapeBox expects a ChVector3d for half-dimensions, not full dimensions.
boxtruss = chrono.ChVisualShapeBox(chrono.ChVector3d(0.03/2, 0.25/2, 0.12/2))  # (was 0.02,0.2,0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFrame(chrono.ChVector3d(-0.015, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)

# -------------------------
# 3. Crank Body Visualization
# -------------------------
boxcrank = chrono.ChVisualShapeBox(chrono.ChVector3d(K/2, 0.03/2, 0.03/2))  # (was K,0.02,0.02)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFrame(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# -------------------------
# 4. Beam Parameters Update
# -------------------------
# Horizontal beam:
beam_wy = 0.12   # Width in Y (was 0.10)
beam_wz = 0.012  # Width in Z (was 0.01)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Vertical beam:
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03  # Diameter (was 0.024)
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))  # 6 elements (was 3)

node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Constraint between horizontal and vertical beams
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)

# -------------------------
# 5. Visualization Shape Updates (constraint spheres)
# -------------------------
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)  # (was 0.01)
constr_bb.AddVisualShape(sphereconstr2)

# Crank beam:
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054  # Diameter (was 0.048)
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))  # 5 elements (was 3)

node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Constraint between crank beam and body crank
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Constraint between vertical beam and crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

sphereconstr3 = chrono.ChVisualShapeSphere(0.014)  # (was 0.01)
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(False)

# Add the mesh to the physical system
sys.Add(mesh)

# -------------------------
# 6. Glyph Customization
# -------------------------
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
mvisualizebeamC.SetSymbolsScale(0.015)  # (was 0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# -------------------------
# 7. Camera Position
# -------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))  # (was 0.0, 0.6, -1.0)
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

# Use the HHT timestepper for less numerical damping and higher accuracy
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