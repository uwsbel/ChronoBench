import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis (FEA) module
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module
import math  # Import the math library for mathematical functions

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Add a physical system to manage visualization assets (like lights, cameras)
sys.AddAsset(chrono.ChVisualSystemAsset())

# Set the position for the interactive camera (commented out in this case)
# interactive_system.SetCameraPosition(chrono.ChVector3d(5, 6, 15))

# Create and configure the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('FEA beam buckling')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(30, 30, 30), chrono.ChVector3d(0, 0, 2))  # Add and position the camera
vis.AddTypicalLights()  # Add typical lights for better illumination

# Create a truss body as a reference point in the simulation
truss = chrono.ChBody()
truss.SetFixed(True)  # Make the truss body fixed (immovable)
sys.Add(truss)  # Add the truss body to the simulation system

# Create and configure the FEA mesh (collection of elements)
mesh = fea.ChMesh()

# Define the beam section properties for the FEA elements
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(0.1, 0.2)  # Set the section as a rectangular area (width=0.1, height=0.2)
beam_section.SetYoungModulus(5e8)  # Set Young's modulus for the beam (elasticity)
beam_section.SetShearModulus(2e8)  # Set shear modulus for the beam
beam_section.SetRayleighDamping(0.000)  # Set Rayleigh damping (none in this case)
beam_section.SetSectionCentroid(0, 0.1)  # Define the centroid of the section
beam_section.SetShearCenter(0, 0.1)  # Define the shear center of the section
beam_section.SetWarpingConstant(0.0)  # Set warping constant (0 for no warping)
beam_section.SetTorsionalRigidity(110e6)  # Set torsional rigidity for the beam

# Create nodes to define the beam's geometry in 3D space
beam_thickness = 0.01  # Define the thickness of the beam
mnode1 = fea.ChNodeFEAxyzD(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))  # Create the first node
mnode2 = fea.ChNodeFEAxyzD(chrono.ChVector3d(0, beam_thickness, 0), chrono.ChVector3d(0, 0, 0))  # Create the second node
mnode3 = fea.ChNodeFEAxyzD(chrono.ChVector3d(beam_thickness, beam_thickness, 0), chrono.ChVector3d(0, 0, 0))  # Create the third node
mnode4 = fea.ChNodeFEAxyzD(chrono.ChVector3d(beam_thickness, 0, 0), chrono.ChVector3d(0, 0, 0))  # Create the fourth node

# Add the nodes to the mesh
mesh.AddNode(mnode1)
mesh.AddNode(mnode2)
mesh.AddNode(mnode3)
mesh.AddNode(mnode4)

# Create a beam element and set its properties
melementbeam = fea.ChElementBeamEuler()
melementbeam.SetNodes(mnode1, mnode3)  # Connect the element between the first and third nodes
melementbeam.SetBeamSection(beam_section)  # Assign the beam section properties to the element
mesh.AddElement(melementbeam)  # Add the beam element to the mesh

# Create gasket filler tetrahedral elements
mnodef1 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))  # Create the first node for the filler
mnodef2 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, -beam_thickness))  # Create the second node for the filler
mnodef3 = fea.ChNodeFEAxyz(chrono.ChVector3d(beam_thickness, 0, -beam_thickness))  # Create the third node for the filler
mnodef4 = fea.ChNodeFEAxyz(chrono.ChVector3d(beam_thickness, 0, 0))  # Create the fourth node for the filler

# Add the filler nodes to the mesh
mesh.AddNode(mnodef1)
mesh.AddNode(mnodef2)
mesh.AddNode(mnodef3)
mesh.AddNode(mnodef4)

# Create and configure tetrahedral elements for the filler
melementf1 = fea.ChElementTetra_4()
melementf1.SetNodes(mnodef1, mnodef2, mnodef3, mnodef4)  # Connect the nodes to form a tetrahedron
melementf1.SetMaterial(chrono.ChContinuumElastic())  # Set the material for the element
melementf1.GetMaterial().SetYoungModulus(5e8)  # Set Young's modulus for the material
melementf1.GetMaterial().SetPoissonRatio(0.3)  # Set Poisson's ratio for the material
mesh.AddElement(melementf1)  # Add the tetrahedral element to the mesh

# Create another tetrahedral element for the filler
melementf2 = fea.ChElementTetra_4()
melementf2.SetNodes(mnodef1, mnodef4, mnode3, mnode1)  # Connect the nodes to form a tetrahedron
melementf2.SetMaterial(chrono.ChContinuumElastic())  # Set the material for the element
melementf2.GetMaterial().SetYoungModulus(5e8)  # Set Young's modulus for the material
melementf2.GetMaterial().SetPoissonRatio(0.3)  # Set Poisson's ratio for the material
mesh.AddElement(melementf2)  # Add the tetrahedral element to the mesh

# Add visualization for the mesh (will display FEM elements)
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_MX)
visualizebeamA.SetColorscaleMinMax(-50, 50)  # Set color scale for visualizing forces/moments
visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visualization shape to the mesh

# Add another visualization for beam stiffness
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BEAM_MZ)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_BEAM_MZ)
visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness for symbols in visualization
visualizebeamB.SetStripThickness(0.006)  # Set thickness for strips in visualization
visualizebeamB.SetStripDistribution(True)  # Enable strip distribution
visualizebeamB.SetSweepAxis(chrono.ChVisualShapeFEA.E_SweepAxis_BMZ)  # Set the axis for sweeping
visualizebeamB.SetSmoothFaces(True)  # Enable smooth faces
mesh.AddVisualShapeFEA(visualizebeamB)  # Add the visualization shape to the mesh

# Create a truss body for the upper end of the beam
mtrussbeam_top = chrono.ChBody()
mtrussbeam_top.SetFixed(False)  # Make the truss body movable
sys.Add(mtrussbeam_top)  # Add the truss body to the simulation system

# Create a beam element to connect the beam to the truss body
melementbeam3 = fea.ChElementBeamEuler()
melementbeam3.SetNodes(mnode3, mtrussbeam_top)  # Connect the third node to the truss body
melementbeam3.SetBeamSection(beam_section)  # Assign the beam section properties
mesh.AddElement(melementbeam3)  # Add the beam element to the mesh

# Add visualization for this beam element
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BEAM_MZ)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_BEAM_MZ)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetStripThickness(0.006)
visualizebeamC.SetStripDistribution(True)
visualizebeamC.SetSweepAxis(chrono.ChVisualShapeFEA.E_SweepAxis_BMZ)
visualizebeamC.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create a truss body for the motor-horizontal constraint
mtrussbeam_h = chrono.ChBody()
mtrussbeam_h.SetFixed(True)  # Make the truss body fixed
sys.Add(mtrussbeam_h)

# Create a horizontal beam element to connect the upper truss beam to the horizontal truss beam
melementbeam2 = fea.ChElementBeamEuler()
melementbeam2.SetNodes(mtrussbeam_beam_top, mtrussbeam_h)  # Connect the nodes
melementbeam2.SetBeamSection(beam_section)  # Assign the beam section properties
mesh.AddElement(melementbeam2)

# Add visualization for this beam element
visualizebeamD = chrono.ChVisualShapeFEA(mesh)
visualizebeamD.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BEAM_MZ)
visualizebeamD.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_BEAM_MZ)
visualizebeamD.SetSymbolsThickness(0.006)
visualizebeamD.SetStripThickness(0.006)
visualizebeamD.SetStripDistribution(True)
visualizebeamD.SetSweepAxis(chrono.ChVisualShapeFEA.E_SweepAxis_BMZ)
visualizebeamD.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamD)

# Create and add a gravity force to the system
mforcegravity = chrono.ChForceGravity()
mforcegravity.SetGravity(chrono.ChVector3d(0, -10, 0))  # Set gravity direction and magnitude
sys.AddForce(mforcegravity)  # Add the gravity force to the system

# Implement constraints for the beam/motor horizontal
constraintherebeam = chrono.ChLinkMateGeneric()
constraintherebeam.Initialize(melementbeam3, True, mtrussbeam_beam_top, True, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(constraintherebeam)  # Add the constraint to the system
constraintherebeam.SetMotionAlongLock(chrono.ChAxis_X, True)  # Lock motion along X-axis

# Implement constraints for the motor-horizontal
constraintheremotor = chrono.ChLinkMateGeneric()
constraintheremotor.Initialize(mtrussbeam_h, True, mtrussbeam_beam_top, True, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(constraintheremotor)
constraintheremotor.SetMotionAlongLock(chrono.ChAxis_X, True)
constraintheremotor.SetMotionAlongLock(chrono.ChAxis_Y, True)
constraintheremotor.SetMotionAlongLock(chrono.ChAxis_Z, True)
constraintheremotor.SetMotionRotAlongLock(chrono.ChAxis_X, True)

# Create and configure the motor for the simulation
mymotor = chrono.ChLinkMotorRotationTranslation()
mymotor.Initialize(mtrussbeam_h, mtrussbeam_beam_top, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChFramed(chrono.ChVector3d(0, 0, 0.1), chrono.QUNIT))
sys.Add(mymotor)  # Add the motor to the system

# Define a custom function for the motor's behavior
class MyCustomChFunction(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        omega = 2
        return (180 / math.pi) * (1 - chrono.exp(-omega * x))  # Define the motor's angular velocity function

# Assign the custom function to the motor
mymotor.SetMotorFunction(MyCustomChFunction())

# Create a load container and add it to the system (for managing loads and forces)
load_container = chrono.ChLoadContainer()
sys.Add(load_container)

# Create and add a Rayleigh damping load to the system
myloadD = chrono.ChLoaderRayleigh(0.0, 0, 0)  # Define Rayleigh damping parameters
load_container.Add(myloadD)  # Add the damping load to the container

# Add the FEA mesh to the simulation system
sys.Add(mesh)

# Set solver parameters for the simulation
sys.SetSolverType(chrono.ChSolver.Type_ANITESCU)
msolver = sys.GetSolver()
if msolver.GetType() == chrono.ChSolver.Type_ANITESCU:
    msolver.SetDiagonalPreconditioning(True)  # Enable diagonal preconditioning
    msolver.SetSparseStep(ChronoSolverSparse.DYNAMIC_SPARSE_STEP_CHOLESKY)
    msolver.SetMaxIterations(80)  # Set maximum iterations for the solver
    msolver.SetTolerance(1e-10)  # Set tolerance for the solver
    msolver.EnableWarmStart(True)  # Enable warm start for the solver

# Set the timestepper for the simulation (use HHT method)
timestepper = chrono.ChTimestepperHHT()
sys.SetTimestepper(timestepper)

# Simulation loop to run the visualization and dynamics simulation
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(0.001)  # Advance the simulation by a time step of 0.001 seconds