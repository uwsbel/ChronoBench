import pychrono as chrono                 # Import the main PyChrono module
import pychrono.fea as fea               # Import the finite element analysis (FEA) module
import pychrono.pardisomkl as mkl       # Import the Pardiso MKL solver module
import pychrono.irrlicht as chronoirr    # Import the Irrlicht visualization module
import math                             # Import the math module for mathematical operations

# Initialize the physical system                  (Step 1)
sys = chrono.ChSystemSMC()

# Add a truss body to which beams will be attached  (Step 2)
truss = chrono.ChBody()
truss.SetFixed(True)  # Set the truss as a fixed body
sys.Add(truss)  # Add the truss to the physical system

# Create a mesh to hold the FEM elements              (Step 3)
mesh = fea.ChMesh()

# Create a section property for beams which defines their cross-section properties (Step 4)
msection = fea.ChBeamSectionEulerAdvanced()  # Create a beam section object
# Set the beam's moments of inertia and other properties
msection.SetAsCircularSection(0.015)  # Set the section as a circular cross-section with a radius of 0.015 m
msection.SetYoungModulus(0.01e9)  # Set the Young's modulus (elasticity) to 0.01 GPa
msection.SetRayleighDamping(0.000)  # Set Rayleigh damping to zero
msection.SetShearModulus(0.01e9*0.3)  # Set the shear modulus, typically G = E*ν/(1+ν)
msection.SetSectionStrainForm(2)  # Set the strain formulation method

# Create nodes for the FEM mesh. Nodes represent points where beams connect (Step 5)
# Create 7 nodes at specified coordinates
node_1 = fea.ChNodeFEAxyz(chrono.ChVector3d(0,0,0))
node_2 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.2,0,0))
node_3 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.4,0,0))
node_4 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.6,0,0))
node_5 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.4,-0.3,0))
node_6 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.6,-0.3,0))
node_7 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.8,-0.6,0))
# Add nodes to the mesh
mesh.AddNode(node_1)
mesh.AddNode(node_2)
mesh.AddNode(node_3)
mesh.AddNode(node_4)
mesh.AddNode(node_5)
mesh.AddNode(node_6)
mesh.AddNode(node_7)

# Create beam elements and set their nodes and sections (Step 6)
belement_1 = fea.ChElementBeamEuler()
belement_1.SetNodes(node_1,node_2)
belement_1.AddSection(msection)

belement_2 = fea.ChElementBeamEuler()
belement_2.SetNodes(node_2,node_3)
belement_2.AddSection(msection)

belement_3 = fea.ChElementBeamEuler()
belement_3.SetNodes(node_3,node_4)
belement_3.AddSection(msection)

belement_4 = fea.ChElementBeamEuler()
belement_4.SetNodes(node_3,node_5)
belement_4.AddSection(msection)

belement_5 = fea.ChElementBeamEuler()
belement_5.SetNodes(node_4,node_6)
belement_5.AddSection(msection)

belement_6 = fea.ChElementBeamEuler()
belement_6.SetNodes(node_5,node_6)
belement_6.AddSection(msection)

belement_7 = fea.ChElementBeamEuler()
belement_7.SetNodes(node_6,node_7)
belement_7.AddSection(msection)

# Add beam elements to the mesh (Step 7)
mesh.AddElement(belement_1)
mesh.AddElement(belement_2)
mesh.AddElement(belement_3)
mesh.AddElement(belement_4)
mesh.AddElement(belement_5)
mesh.AddElement(belement_6)
mesh.AddElement(belement_7)

# Apply boundary conditions by fixing certain nodes (Step 8)
# Fix node_1 and node_7 to restrict their movement
node_1.SetFixed(True)
node_7.SetFixed(True)
# Create a load container to manage forces and loads applied to the FEM elements (Step 9)
load_container = fea.ChLoadContainer()
sys.Add(load_container)  # Add the load container to the system

# Apply forces to nodes (Step 10)
# Create a force object and apply it to node_5
force = fea.ChLoadNodeForce()
force.SetForce(chrono.ChVector3d(0,1.0,0))  # Apply a force of 1 N in the positive Y direction
force.SetPointAtAbs(chrono.ChVector3d(0.4,-0.15,0))  # Specify the application point of the force
load_container.Add(force)  # Add the force to the load container
force.Loadable.AppendFootNode(node_5)  # Associate the force with node_5

# Add visualization for FEM beams (Step 11)
# Create a visualization object for the beam frames
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Set data type for moment along the beam
mvisualizebeamA.SetColorscaleMinMax(-0.4,0.4)  # Set color scale range
mvisualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
mvisualizebeamA.SetDrawBeams(True)  # Enable drawing of beams
mesh.AddVisualShapeFEA(mvisualizebeamA)  # Add the visualization shape to the mesh
    
# Add visualization for nodes (Step 12)
# Create a visualization object for node positions
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Set glyph type to dots at node positions
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No FEM data to visualize
mvisualizebeamC.SetSymbolsThickness(0.006)  # Set thickness of symbols
mvisualizebeamC.SetSymbolsScale(0.01)  # Set scale of symbols
mvisualizebeamC.SetZbufferHide(False)  # Ensure symbols are visible
mesh.AddVisualShapeFEA(mvisualizebeamC)  # Add the visualization shape to the mesh

# Finalize the mesh by adding it to the physical system (Step 14)
sys.Add(mesh)

# Set up the Irrlicht visualization system (Step 15)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the physical system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('FEA beams')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for background
vis.AddCamera(chrono.ChVector3d(0.5,0.2,1.3), chrono.ChVector3d(0.4,0.0,0.3))  # Add and position the camera
vis.AddTypicalLights()  # Add typical lights for better visualization

# Set the solver for the system (Step 16)
msolver = mkl.ChSolverPardisoMKL()  # Create a Pardiso MKL solver object
sys.SetSolver(msolver)  # Set the solver for the system

# Define the time step for the simulation (Step 17)
timestep = 1e-3  # Set the time step to 0.001 seconds

# Run the simulation loop (Step 18)
while vis.Run():
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    sys.DoStepDynamics(timestep)  # Perform a simulation step with the defined time step