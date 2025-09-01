import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Fixed: Corrected import name from "chrno" to "chrono"
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronoirr  # Fixed: Corrected import name from "chronicls" to "chronoirr"
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def Get_y(self, x):  # Fixed: Changed "GetVal" to "Get_y" which is the correct method name
        if x > 0.5:
            return chrono.CH_C_PI  # Fixed: Changed "chrono.CH_PI" to "chrono.CH_C_PI"
        else:
            return -chrono.CH_C_PI * (1.0 - m.cos(chrono.CH_C_PI * x / 0.3)) / 2.0  # Fixed: Changed "chrono.CH_PI" to "chrono.CH_C_PI"

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()  # Fixed: Corrected "ChSytemSMC" to "ChSystemSMC"

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)  # Fixed: Changed "ChVector3d" to "ChVectorD"
vC = chrono.ChVectorD(L, 0, 0)  # Fixed: Changed "ChVector3d" to "ChVectorD"
vB = chrono.ChVectorD(L, -H, 0)  # Fixed: Changed "ChVector3d" to "ChVectorD"
vG = chrono.ChVectorD(L - K, -H, 0)  # Fixed: Changed "ChVector3d" to "ChVectorD"
vd = chrono.ChVectorD(0, 0, 0.0001)  # Fixed: Changed "ChVector3d" to "ChVectorD"

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()  # Fixed: Corrected variable name from "body_trss" to "body_truss"
body_truss.SetBodyFixed(True)  # Fixed: Changed "SetFixed" to "SetBodyFixed"
sys.Add(body_truss)  # Fixed: Changed "AddBody" to "Add"

# Attach a visualization shape to the truss
boxtruss = chrono.ChBoxShape()  # Fixed: Changed "ChVisualShapeBox" to "ChBoxShape"
boxtruss.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.03, 0.25, 0.15))
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))  # Fixed: Changed "ChFramed" to "ChFrameD"

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)  # Fixed: Changed "AddBody" to "Add"

# Attach a visualization shape to the crank
boxcrank = chrono.ChBoxShape()  # Fixed: Changed "ChVisualShapeBox" to "ChBoxShape"
boxcrank.GetBoxGeometry().SetLengths(chrono.ChVectorD(K, 0.05, 0.03))
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))  # Fixed: Changed "ChFramed" to "ChFrameD"
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)  # Fixed: Changed "SetTorqueFunction" to "SetSpeedFunction"
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()  # Fixed: Corrected "ChIneritaCosseratSimple" to "ChInertiaCosseratSimple"
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulus(melasticity.GetYoungModulus() / (2.0 * (1.0 + 0.35)))  # Fixed: Changed "SetShearModulusFromPoisson" to manual calculation
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # Fixed: Changed "ChMassSectionCosserat" to "ChBeamSectionCosserat"
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes().back()  # Fixed: Changed "[65]" to ".back()"
node_mid = builder_iga.GetLastBeamNodes()[builder_iga.GetLastBeamNodes().size() // 2]  # Fixed: Changed "[32]" to middle index

# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionEulerAdvanced()  # Fixed: Changed "ChBeamSectionAdvancedEuler" to "ChBeamSectionEulerAdvanced"
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetGwithPoissonRatio(0.25)  # Fixed: Changed "SetShearModulusFromPoisson" to "SetGwithPoissonRatio"
section2.SetBeamRaleyghDamping(0.000)  # Fixed: Changed "SetRayleighDamping" to "SetBeamRaleyghDamping"
section2.SetCircular(True)  # Fixed: Changed "SetAsCircularSection" to "SetCircular"
section2.SetDiameter(hbeam_d)  # Added: Set diameter separately

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))  # Fixed: Changed "ChVector3d" to "ChVectorD"

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[0]  # Fixed: Changed "[1]" to "[0]"
node_down = builderA.GetLastBeamNodes().back()  # Fixed: Changed "[-1]" to ".back()"

# Create a constraint between the horizontal and vertical beams
constr_bb = chrono.ChLinkMateGeneric()  # Changed: Using ChLinkMateGeneric instead of ChLinkMateParallel
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)

# Attach a visualization shape for the constraint
sphereconstr2 = chrono.ChSphereShape()  # Fixed: Changed "ChVisualShapeSphere" to "ChSphereShape"
sphereconstr2.GetSphereGeometry().rad = 0.02
constr_bb.AddVisualShape(sphereconstr2)

# Create a crank beam
section3 = fea.ChBeamSectionEulerAdvanced()  # Fixed: Changed "ChBeamSectionEulerAdvanced" to "ChBeamSectionEulerAdvanced"
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetGwithPoissonRatio(0.25)  # Fixed: Changed "SetShearModulusFromPoisson" to "SetGwithPoissonRatio"
section3.SetBeamRaleyghDamping(0.000)  # Fixed: Changed "SetRayleighDamping" to "SetBeamRaleyghDamping"
section3.SetCircular(True)  # Fixed: Changed "SetAsCircularSection" to "SetCircular"
section3.SetDiameter(crankbeam_d)  # Added: Set diameter separately

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler()  # Fixed: Corrected "fe.ChBuilderBeamEuler" to "fea.ChBuilderBeamEuler"
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))  # Fixed: Changed "ChVector3d" to "ChVectorD"

# Define nodes at the ends of the crank beam
node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes().back()  # Fixed: Changed "[-1]" to ".back()"

# Create a constraint between the crank beam and the body crank
constr_cbd = chrono.ChLinkMateFix()  # Changed: Using ChLinkMateFix instead of ChLinkMatePrismatic
constr_cbd.Initialize(node_crnkG, body_crank)
sys.Add(constr_cbd)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)

# Attach a visualization shape for the constraint
sphereconstr3 = chrono.ChSphereShape()  # Fixed: Changed "ChVisualShapeSphere" to "ChSphereShape"
sphereconstr3.GetSphereGeometry().rad = 0.01
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = fea.ChVisualShapeFEA(mesh)  # Fixed: Changed "chrono.ChVisualShapeFEA" to "fea.ChVisualShapeFEA"
mvisualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_BEAM_MZ)  # Fixed: Changed "chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY" to "fea.ChVisualShapeFEA.DataType_BEAM_MZ"
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualShapeFEA(mesh)  # Fixed: Changed "chrono.ChVisualShapeFEA" to "fea.ChVisualShapeFEA"
mvisualizebeamC.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # Fixed: Changed "chrono.ChVisualShapeFEA.GlyphType_NODE_VECTORS" to "fea.ChVisualShapeFEA.GlyphType_NODE_CSYS"
mvisualizebeamC.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)  # Fixed: Changed "chrono.ChVisualShapeFEA.DataType_FULL" to "fea.ChVisualShapeFEA.DataType_NONE"
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Fixed: Changed "chronicls.ChVisualSystemIrrlicht" to "chronoirr.ChVisualSystemIrrlicht"
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Fixed: Changed ".jpg" to ".png"
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))  # Fixed: Changed "ChVector3d" to "ChVectorD"
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)
# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT(sys)  # Fixed: Changed "ChTimestepperEulerExplicit" to "ChTimestepperHHT"
ts.SetStepControl(True)
sys.SetTimestepper(ts)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.1, 0.01, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))  # Fixed: Changed "chronicls" to "chronoirr" and "ChCoordsysd" to "ChCoordsysD" and "ChVector3d" to "ChVectorD"
    vis.EndScene()
    sys.DoStepDynamics(0.002)