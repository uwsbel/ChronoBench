import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
# Initialize PyChrono
# ---------------------------------------------------------------------

# Create a Chrono physical system
sys = chrono.ChSystemSMC() # Using SMC for consistency, though Direct solvers are often better for pure FEA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create an FEA mesh
my_mesh = fea.ChMesh()
sys.Add(my_mesh)

# ---------------------------------------------------------------------
# Define Beam Properties
# ---------------------------------------------------------------------

# Material properties (e.g., steel)
mat_density = 7850.0  # kg/m^3
mat_E = 210e9        # Young's modulus (Pa)
mat_nu = 0.3         # Poisson's ratio
mat_G = mat_E / (2 * (1 + mat_nu)) # Shear modulus (Pa)
mat_specific_weight = mat_density * 9.81 # N/m^3

# Beam cross-section properties (e.g., rectangular)
beam_section_h = 0.1  # height (m) - along local y
beam_section_w = 0.05 # width (m)  - along local z

# Create a beam section profile
# For ChElementBeamEuler, we use ChBeamSectionEulerAdvanced
# It requires material properties and cross-section geometry.
# It also manages the computation of Izz, Iyy, J (torsional constant), Area, etc.
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetDensity(mat_density)
beam_section.SetYoungModulus(mat_E)
beam_section.SetGshearModulus(mat_G) # Optional: if not set, G is computed from E and nu
beam_section.SetBeamRaleyghDamping(0.000) # Optional: structural damping
beam_section.SetAsRectangularSection(beam_section_w, beam_section_h)

# ---------------------------------------------------------------------
# Create Nodes and Beam Elements
# ---------------------------------------------------------------------

num_elements = 10
beam_length = 2.0  # Total length of the beam (m)
delta_x = beam_length / num_elements

nodes = []
beam_elements = []

# Create nodes
for i in range(num_elements + 1):
    x_pos = i * delta_x
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(x_pos, 0, 0))) # Node at (x_pos, 0, 0)
    my_mesh.AddNode(node)
    nodes.append(node)

# Create beam elements
for i in range(num_elements):
    nodeA = nodes[i]
    nodeB = nodes[i+1]
    
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodeA, nodeB)
    beam_element.SetSection(beam_section)
    
    # Set the orientation of the beam's cross-section.
    # The beam's local x-axis is along the line from nodeA to nodeB.
    # We need to define the direction of the beam's local y-axis.
    # For a horizontal beam along global X, a local y-axis along global Y is common.
    beam_element.SetRestEllipticInertiaProducts(False) # Assume principal axes of inertia are aligned with section axes
    # For Euler beams, this orientation is typically set implicitly by the default ChBeamSectionEuler or explicitly
    # by providing a rotation matrix if the section isn't aligned with global axes.
    # For this simple horizontal beam, default orientation often works.
    # If more control is needed, one can use: beam_element.SetNodeAreferenceRot(chrono.ChMatrix33D(chrono.QUNIT))
    # or ensure the section's principal axes are defined correctly relative to its reference frame.
    # For ChBeamSectionEulerAdvanced, the definition SetAsRectangularSection implies local y is h, local z is w.

    my_mesh.AddElement(beam_element)
    beam_elements.append(beam_element)

# ---------------------------------------------------------------------
# Apply Boundary Conditions and Loads
# ---------------------------------------------------------------------

# Fix the first node (cantilever support)
nodes[0].SetFixed(True)

# Apply a vertical force to the last node
tip_force_y = -500.0  # N (downwards)
nodes[-1].SetForce(chrono.ChVector3d(0, tip_force_y, 0))
# Alternatively, to see dynamic effects better:
# nodes[-1].SetMass(10) # Add some mass to the tip for dynamic response

# ---------------------------------------------------------------------
# Configure Solver
# ---------------------------------------------------------------------

# Use a suitable solver for FEA (e.g., MINRES or SparseLU)
# sys.SetSolverType(chrono.ChSolver.Type_MINRES) # Old way
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

# Set up time integrator (implicit HHT is good for FEA)
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT) # Old way
stepper = chrono.ChTimestepperHHT(sys) # New way
sys.SetTimestepper(stepper)
stepper.SetAlpha(-0.2) # Set HHT alpha parameter (controls numerical damping, range -1/3 to 0)
stepper.SetMaxiters(10)
stepper.SetAbsoler(1e-6)
stepper.SetMode(chrono.ChTimestepperHHT.POSITION) # Solve for position (other option:ACCELERATION)
stepper.SetScaling(True)
stepper.SetStepControl(False) # Disable step control for simpler demo
stepper.SetVerbose(False)


# ---------------------------------------------------------------------
# Visualization Setup (Irrlicht)
# ---------------------------------------------------------------------

# Create an Irrlicht application
application = chronoirr.ChIrrApp(sys, "Beam FEM Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(beam_length * 0.5, beam_length * 0.3, -beam_length * 0.8), # Camera position
                             chronoirr.vector3df(beam_length * 0.5, 0, 0))      # Camera look at point

# Create a visual shape for the FEA mesh
# This will render the beam elements
vis_fea_mesh = fea.ChVisualShapeFEA(my_mesh)
vis_fea_mesh.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE) # Don't show specific scalar data by default
vis_fea_mesh.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NONE) # No special glyphs for nodes/elements
vis_fea_mesh.SetSymbolsScale(0.01)
vis_fea_mesh.SetWireframe(False) # Render as solid
vis_fea_mesh.SetDefault moždaBeamColor(chronoirr.SColor(255, 0, 100, 100)) # Blue-ish color for beams
vis_fea_mesh.SetBeamSolidShape(fea.ChVisualShapeFEA.BeamSolidShape_RECTANGULAR_SECTION) # Match section
my_mesh.AddVisualShapeFEA(vis_fea_mesh)


# Optional: Add markers for nodes to see their positions clearly
# vis_nodes = fea.ChVisualShapeFEA(my_mesh)
# vis_nodes.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
# vis_nodes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
# vis_nodes.SetSymbolsScale(0.05) # Size of the dots
# vis_nodes.SetDefaultSymbolsColor(chronoirr.SColor(255, 255, 0, 0)) # Red dots
# my_mesh.AddVisualShapeFEA(vis_nodes)


# Bind visualization assets
application.AssetBindAll()
application.AssetUpdateAll()


# ---------------------------------------------------------------------
# Simulation Loop
# ---------------------------------------------------------------------
application.SetTimestep(0.01)       # Simulation time step
application.SetTryRealtime(False)   # Try to run in real-time (can slow down if computation is heavy)

# Number of simulation steps
# For a static analysis, you might just run a few steps until equilibrium.
# For dynamics, you'd run for a longer duration.
# Let's make it settle to a quasi-static solution if gravity is off or small.
# If you enable gravity, it will oscillate.

print("Starting simulation...")

simulation_time = 0
max_simulation_time = 5.0 # seconds

# If you want a "quasi-static" solution by ramping up the load slowly:
# This is more complex and often involves applying the load incrementally over several steps.
# For this demo, we apply it instantly and let the dynamics settle (or HHT damping helps).

while application.GetDevice().run():
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190)) # Background color
    application.DrawAll()
    
    # Perform a simulation step
    sys.DoStepDynamics(application.GetTimestep())
    
    simulation_time += application.GetTimestep()
    
    # Optional: Print tip deflection
    if int(simulation_time / application.GetTimestep()) % 10 == 0: # Print every 10 steps
        tip_node_pos = nodes[-1].GetPos()
        print(f"Time: {simulation_time:.2f} s, Tip Y-Deflection: {tip_node_pos.y:.4f} m")

    application.EndScene()

    if simulation_time > max_simulation_time:
        break
        
print("Simulation finished.")
application.GetDevice().closeDevice()