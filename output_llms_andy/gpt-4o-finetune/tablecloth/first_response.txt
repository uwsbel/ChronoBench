import pychrono.core as chrono  # Import the core Chrono library
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization library for Chrono
import pychrono.fea as fea  # Import the Finite Element Analysis (FEA) components of Chrono
import pychrono.pardisomkl as pardisomkl  # Import the PardisoMKL solver for Chrono
import math  # Import the math library for mathematical functions

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define material properties for the tablecloth
my_viscoelasticity = 1e7  # Set viscoelasticity to a high value (not used directly here)
my_E = 3.5e9  # Set Young's modulus (Elasticity) to 3.5 GPa
my_v = 0.3  # Set Poisson's ratio to 0.3

# Define the dimensions of the tablecloth
hlength = 1.1  # Half-length of the tablecloth
hhwidth = 0.5  # Half-width of the tablecloth
thickness = 0.005  # Thickness of the tablecloth in meters

# Create a mesh to hold the FEM elements
mesh = fea.ChMesh()

# Define the type of beam section for the elements (circular section with diameter 1mm)
 msection = fea.ChBeamSectionEulerAdvanced()
 msection.SetDiameter(0.001)
 msection.SetShearCenterTowardsNeutralAxis()
 msection.SetYoungModulus(my_E)
 msection.SetRayleighDamping(0.))  # No Rayleigh damping
 msection.SetUseMitcalCSectionShearFormulation(True)

# Define a truss load case for applying loads to the structure
mtruss_load_case = fea.ChLoadTrussAbsolute()
mtruss_load_case.Add(chrono.VECT_X, 1, -1)
mtruss_load_case.Add(chrono.VECT_Y, 1, -1)
mtruss_load_case.Add(chrono.VECT_Z, 1, -1)

# Define a function to create a straight beam segment with truss load
def make_straight_beam(p1, p2):
    # Create a segment between two points
    segment = fea.ChBeamSegmentEuler()
    segment.BuildBeam(p1, p2, msection)

    # Load the segment with the truss load case
    segment.Load(mtruss_load_case)

    return segment

# Create nodes for the FEM mesh
last_left = mesh.AddNode(
    fea.ChNodeFEA0(
        chrono.ChVector3d(-hlength, -hhwidth, 0),  # Position: left-bottom
        chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  # Rotation: not defined for this node type
    )
)
last_right = mesh.AddNode(
    fea.ChNodeFEA0(
        chrono.ChVector3d(-hlength, +hhwidth, 0),  # Position: left-top
        chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  # Rotation: not defined for this node type
    )
)

# Create 'numdiv' divisions along the length of the tablecloth
numdiv = 30  # Number of divisions
for i in range(numdiv):
    next_left = mesh.AddNode(
        fea.ChNodeFEA0(
            chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, -hhwidth, 0),  # Position: next left-bottom point
            chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  # Rotation: not defined for this node type
        )
    )
    next_right = mesh.AddNode(
        fea.ChNodeFEA0(
            chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, +hhwidth, 0),  # Position: next left-top point
            chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  # Rotation: not defined for this node type
        )
    )
    
    # Create lines of elements (beams) between nodes
    line1 = fea.ChMesh()
    line1.AddNode(last_left)
    line1.AddNode(next_left)
    beam1 = fea.ChElementBeamEuler()
    beam1.SetSection(msection)
    beam1.Resize(2)
    beam1.SetNodes(line1.GetNode(0), line1.GetNode(1))
    line1.AddElement(beam1)
    mtruss_load_case.AddElement(beam1)
    mload = fea.ChLoadBeamElement()
    mload.AddNext(chrono.VECT_Y, -0.25)
    beam1.AddLoad(mload)
    mesh.AddBeam(line1)

    line2 = fea.ChMesh()
    line2.AddNode(last_right)
    line2.AddNode(next_right)
    beam2 = fea.ChElementBeamEuler()
    beam2.SetSection(msection)
    beam2.Resize(2)
    beam2.SetNodes(line2.GetNode(0), line2.GetNode(1))
    line2.AddElement(beam2)
    mtruss_load_case.AddElement(beam2)
    mesh.AddBeam(line2)

    line3 = fea.ChMesh()
    line3.AddNode(last_left)
    line3.AddNode(last_right)
    beam3 = fea.ChElementBeamEuler()
    beam3.SetSection(msection)
    beam3.Resize(2)
    beam3.SetNodes(line3.GetNode(0), line3.GetNode(1))
    line3.AddElement(beam3)
    mtruss_load_case.AddElement(beam3)
    mesh.AddBeam(line3)

    line4 = fea.ChMesh()
    line4.AddNode(next_left)
    line4.AddNode(next_right)
    beam4=fea.ChElementBeamEuler()
    beam4.SetSection(msection)
    beam4.Resize(2)
    beam4.SetNodes(line4.GetNode(0), line4.GetNode(1))
    line4.AddElement(beam4)
    mtruss_load_case.AddElement(beam4)
    mesh.AddBeam(line4)

    segment = make_straight_beam(chrono.ChVector3d(-hlength + i * 2 * hlength / numdiv, 0, 0),
                                chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, 0, 0))  
    mesh.AddElement(segment)

    segment = make_straight_beam(chrono.ChVector3d(-hlength + i * 2 * hlength / numdiv, -hhwidth / 3, 0),
                                chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, -hhwidth / 3, 0))  
    mesh.AddElement(segment)

    segment = make_straight_beam(chrono.ChVector3d(-hlength + i * 2 * hlength / numdiv, hhwidth / 3, 0),
                                chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, hhwidth / 3, 0))  
    mesh.AddElement(segment)

    last_left = next_left
    last_right = next_right

# Fix the nodes at one end of the tablecloth to simulate attachment to the table
for i in range(10):
    nodeB = mesh.GetNode(numdiv - 1 + i * (numdiv + 1))
    mfix = chrono.ChLinkMateFix()
    mfix.Initialize(nodeB, chrono.ChFramed())
    sys.Add(mfix)

# Add the mesh to the physical system
sys.Add(mesh)

# Create the visualization system for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Kirchhoff-Love shell elements FEA')  # Set the title of the window
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add the Chrono logo
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(0, 0.7, -1.5), chrono.ChVector3d(0, 0, 0))  # Add and position the camera
vis.AddTypicalLights()  # Add typical lights for better illumination

# Use the PardisoMKL solver for better performance in solving linear systems
solver = pardisomkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# Set solver parameters to improve convergence and stability
solver.SetAutomaticScaleFactor(True)  # Automatically scale the problem for the solver
sys.SetTimestepperType(chrono.ChTimestepper.Type.HHT)  # Use the HHT timestepper for integration
solver.GetMaxIterationsGeneral().Set(50)  # Set the maximum number of iterations for the solver

# Set parameters for the HHT timestepper (used for implicit integration)
hht = chrono.ChTimestepperHHT()
sys.SetTimestepper(hht)
hht.SetAlpha(-0.1)  # Set the alpha parameter for numerical damping (-0.1 is a common value)
hht.SetMaxIterations(100)  # Set the maximum iterations for convergence
hht.SetAbsTolerances(1e-5, 1e-2)  # Set absolute tolerances for the solver
hht.SetMode(chrono.ChTimestepper.Mode.NORMAL)  # Set the mode of the timestepper to normal

# Start the interactive simulation loop with a timestep of 0.001 seconds
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(0.001)  # Advance the simulation by 0.001 seconds