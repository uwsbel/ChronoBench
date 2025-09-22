import pychrono as chrono  # Import the main PyChrono module.
import pychrono.fea as fea  # Import the finite element analysis (FEA) module.
import pychrono.pardisomkl as mkl  # Import the Pardiso MKL linear solver module.
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module.

print("Example: PyChrono using Euler-Bernoulli beam finite elements")  # Updated comment

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for beam properties. This will define the characteristics of all beams that use this section.
msection = fea.ChBeamSectionEulerAdvanced()

# Set the width and height of the rectangular section of the beam.
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)  # Set the cross-sectional shape and size of the beam.

# Set the material properties of the beam.
msection.SetYoungModulus(0.01e9)  # Young's modulus, a measure of the stiffness of the material.
msection.SetShearModulus(0.01e9 * 0.3)  # Shear modulus, also related to the rigidity of the material.
msection.SetRayleighDamping(0.000)  # Damping coefficient for Rayleigh damping, affecting the dynamic response.
msection.SetCentroid(0, 0.02)  # Set the position of the centroid.
msection.SetShearCenter(0, 0.1)  # Set the position of the shear center.
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # CORRECTED: Use radians instead of degrees

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Create Euler-Bernoulli beam using builder helper
# ================================================
# Using ChBuilderBeamEuler to create a beam with 5 elements
# from (0,0,-0.1) to (0.2,0,-0.1) with Y as up direction
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,                   # mesh to store elements
    msection,               # beam section properties
    5,                      # number of elements
    chrono.ChVector3d(0, 0, -0.1),  # start point (A)
    chrono.ChVector3d(0.2, 0, -0.1),  # end point (B)
    chrono.ChVector3d(0, 1, 0)  # "up" direction for section
)

# Get reference to nodes
first_node = builder.GetLastBeamNodes().front()
last_node = builder.GetLastBeamNodes().back()

# Apply force to first node
first_node.SetForce(chrono.ChVector3d(0, -1, 0))

# Fix last node using constraint instead of SetFixed(True)
# Create a fixed body to attach constraint to
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# Create and initialize constraint for last node
constr_end = chrono.ChLinkMateGeneric()
constr_end.Initialize(last_node, ground, False, last_node.Frame(), last_node.Frame())
sys.Add(constr_end)
constr_end.SetConstrainedCoords(True, True, True, True, True, True)  # Fix all DOFs

# Add the mesh to the physical system.
sys.Add(mesh)

# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Visualize the bending moments.
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale limits.
visualizebeamA.SetSmoothFaces(True)  # Smooth the faces for visualization.
visualizebeamA.SetWireframe(False)  # Disable wireframe mode.
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # Visualize coordinate systems at nodes.
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No data type visualization.
visualizebeamC.SetSymbolsThickness(0.006)  # Set symbol thickness.
visualizebeamC.SetSymbolsScale(0.01)  # Set symbol scale.
visualizebeamC.SetZbufferHide(False)  # Do not hide symbols behind objects.
mesh.AddVisualShapeFEA(visualizebeamC)

# Create an Irrlicht visualization window.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the simulation system to the visual system.
vis.SetWindowSize(1024, 768)  # Set the window size.
vis.SetWindowTitle('FEA beams')  # Set the window title.
vis.Initialize()  # Initialize the visual system.
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add the Chrono logo.
vis.AddSkyBox()  # Add a skybox.
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))  # Add a camera.
vis.AddTypicalLights()  # Add typical lights for the scene.

# Change the default solver to the MKL Pardiso solver, which is more precise for FEA.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)  # Set the MKL Pardiso solver for the system.

# Simulation loop.
while vis.Run():
    vis.BeginScene()  # Begin the scene.
    vis.Render()  # Render the scene.
    vis.EndScene()  # End the scene.
    sys.DoStepDynamics(0.001)  # Perform one step of simulation with a step size of 0.001 seconds.