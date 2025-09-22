import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as pardiso  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        # Call the base class constructor
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        # Define a function that returns different angle values based on input x
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Define key geometrical parameters
L = 1.2  # Length
H = 0.3  # Height
K = 0.07  # Crank length
vA = chrono.ChVector3d(0, 0, 0)  # Point A
vC = chrono.ChVector3d(L, 0, 0)  # Point C
vB = chrono.ChVector3d(L, -H, 0)  # Point B
vG = chrono.ChVector3d(L - K, -H, 0)  # Point G
vd = chrono.ChVector3d(0, 0, 0.0001)  # Small offset vector

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()
body_truss.SetFixed(True)  # Make the truss immobile
sys.AddBody(body_truss)  # Add the truss to the physical system

# Attach a visualization shape to the truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.015, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  # Set the position of the crank body
sys.AddBody(body_crank)  # Add the crank to the physical system

# Attach a visualization shape to the crank
boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  # Initialize motor between truss and crank
myfun = ChFunctionMyFun()  # Create an instance of the custom function
motor.SetAngleFunction(myfun)  # Set the angle function for the motor
sys.Add(motor)  # Add the motor to the system

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12  # Width in Y direction
beam_wz = 0.012  # Width in Z direction

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  # Define the rectangular section with density

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  # Set Young's modulus for elasticity
melasticity.SetShearModulusFromPoisson(0.3)  # Set shear modulus using Poisson's ratio
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  # Define the section dimensions

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # Create beam section
msection1.SetDrawThickness(beam_wy, beam_wz)  # Set the drawing thickness

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)  # Add IGA beam to mesh

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]  # Get the node at the tip
node_mid = builder_iga.GetLastBeamNodes()[17]  # Get a node in the middle

# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03  # Diameter of circular section
section2.SetDensity(2700)  # Set density
section2.SetYoungModulus(73.0e9)  # Set Young's modulus
section2.SetShearModulusFromPoisson(0.3)  # Set shear modulus
section2.SetRayleighDamping(0.000)  # Set Rayleigh damping
section2.SetAsCircularSection(hbeam_d)  # Define the circular section

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))  # Add Euler beam to mesh

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the horizontal and vertical beams
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)  # Constrain x, y, z

# Attach a visualization shape for the constraint
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)
constr_bb.AddVisualShape(sphereconstr2)

# Create a crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054  # Diameter of circular section
section3.SetDensity(2700)  # Set density
section3.SetYoungModulus(73.0e9)  # Set Young’s modulus
section3.SetShearModulusFromPoisson(0.3)  # Set shear modulus
section3.SetRayleighDamping(0.000)  # Set Rayleigh damping
section3.SetAsCircularSection(crankbeam_d)  # Define the circular section

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))  # Add Euler beam to mesh

# Define nodes at the ends of the crank beam
node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the body crank
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

# Attach a visualization shape for the constraint
sphereconstr3 = chrono.ChVisualShapeSphere(0.014)
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(False)  # Disable automatic gravity on the FEA elements

# Add the mesh to the physical system
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(
    chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)  # Set data type for visualizing beam moments
mvisualizebeamA.SetColorscaleMinMax(-500, 500)  # Set color scale limits
mvisualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for visualization
mvisualizebeamA.SetWireframe(False)  # Disable wireframe
mesh.AddVisualShapeFEA(mvisualizebeamA)  # Add visualization shape to mesh

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(
    chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # Set glyph type for node coordinate systems
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # Disable additional data visualization
mvisualizebeamC.SetSymbolsThickness(0.006)  # Set thickness for glyphs
mvisualizebeamC.SetSymbolsScale(0.015)  # Set scale for glyphs
mvisualizebeamC.SetZbufferHide(False)  # Disable Z-buffer hiding for glyphs
mesh.AddVisualShapeFEA(mvisualizebeamC)  # Add visualization shape to mesh

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the visual system to the physical system
vis.SetWindowSize(1024, 768)  # Set the window size
vis.SetWindowTitle('Beams and constraints')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better visualization
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))  # Add a camera to the scene
vis.AddTypicalLights()  # Add typical lighting to the scene

# Use a solver that can handle stiffness matrices
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)  # Set the solver to the system

# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)  # Set the timestepper to the system

# Main simulation loop
while vis.Run():
    vis.BeginScene()  # Begin rendering the scene
    vis.Render()  # Render the scene
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # Draw a grid
    vis.EndScene()  # End rendering the scene
    sys.DoStepDynamics(0.001)  # Perform a simulation step with a timestep of 0.001 seconds