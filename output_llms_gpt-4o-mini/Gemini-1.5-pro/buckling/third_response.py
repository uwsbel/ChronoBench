import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronicls  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def Get_y(self, x):  # Use Get_y instead of GetVal
        if x > 0.5:
            return chrono.CH_C_PI  # Use chrono.CH_C_PI instead of chrono.CH_PI
        else:
            return -chrono.CH_C_PI * (1.0 - m.cos(chrono.CH_C_PI * x / 0.3)) / 2.0  # Use chrono.CH_C_PI instead of chrono.CH_PI

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()  # Use chrono.ChSystemSMC instead of chrono.ChSytemSMC

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)  # Use chrono.ChVectorD instead of chrono.ChVector3d
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_trss = chrono.ChBodyEasyBox(0.03, 0.25, 0.15, 1000, True, True) # Use chrono.ChBodyEasyBox instead the combination of chrono.ChBody and chrono.ChVisualShapeBox
body_trss.SetPos(chrono.ChVectorD(-0.01, 0, 0))
body_trss.SetBodyFixed(True)  # Use SetBodyFixed instead of SetFixed
sys.Add(body_trss)  # Add body to the system


# Create a crank body:
body_crank = chrono.ChBodyEasyBox(K, 0.05, 0.03, 1000, True, True)  # Use chrono.ChBodyEasyBox instead the combination of chrono.ChBody and chrono.ChVisualShapeBox
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)


# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrono.ChFrameD(vG))  # Use chrono.ChFrameD instead of chrono.ChFramed
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)  # Use SetAngleFunction instead of SetTorqueFunction
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()  # Use fea.ChInertiaCosseratSimple instead of fea.ChIneritaCosseratSimple
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMaterialBeamCosserat(minertia, melasticity)  # Use fea.ChMaterialBeamCosserat instead of fea.ChMassSectionCosserat
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[15]  # Index corrected to match the desired node
node_mid = builder_iga.GetLastBeamNodes()[7]  # Index corrected to match the desired node

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
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVectorD(1, 0, 0))  # Use chrono.ChVectorD instead of chrono.ChVector3d

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the horizontal and vertical beams
constr_bb = chrono.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)

# Attach a visualization shape for the constraint
sphereconstr2 = chrono.ChVisualShapeSphere(0.02)
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
builderB = fea.ChBuilderBeamEuler()  # Use fea.ChBuilderBeamEuler instead of fe.ChBuilderBeamEuler
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVectorD(0, 1, 0))  # Use chrono.ChVectorD instead of chrono.ChVector3d

# Define nodes at the ends of the crank beam
node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the body crank
constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)

# Attach a visualization shape for the constraint
sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = fea.ChVisualizationFEAmesh(mesh)  # Use fea.ChVisualizationFEAmesh instead of chrono.ChVisualShapeFEA
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.EDataType.ELEM_BEAM_MY)  # Corrected path and method name
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualizationFEAmesh(mesh)  # Use fea.ChVisualizationFEAmesh instead of chrono.ChVisualShapeFEA
mvisualizebeamC.SetFEMglyphType(fea.ChVisualizationFEAmesh.EGlyphType.NODE_VECTORS)  # Corrected path and method name
mvisualizebeamC.SetFEMdataType(fea.ChVisualizationFEAmesh.EDataType.FULL)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization system
vis = chronicls.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)  # Use SetWindowSize instead of SetWindowSize
vis.SetWindowTitle('Failed Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Use .png instead of .jpg
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))  # Use chrono.ChVectorD instead of chrono.ChVector3d
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)
# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT(sys)  # Use chrono.ChTimestepperHHT instead of chrono.ChTimestepperEulerExplicit
ts.SetStepControl(True)
sys.SetTimestepper(ts)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronicls.drawGrid(vis, 0.1, 0.01, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))  # Use chrono.ChCoordsysD and chrono.ChVectorD
    vis.EndScene()
    sys.DoStepDynamics(0.002)