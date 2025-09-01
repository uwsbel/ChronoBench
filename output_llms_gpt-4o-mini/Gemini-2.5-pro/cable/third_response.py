import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of ANCF cable elements, each hinged at one end
# and connected to a ChBody (box) at the other end, moving under gravity
# and applied forces.
# This model demonstrates dynamic creation of multiple FEA structures
# and their interaction with rigid bodies.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # To store the ChBody objects at the end of each chain

        # Define beam section properties (shared by all chains)
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  # Diameter of the cable section (15 mm)
        msection_cable.SetYoungModulus(0.01e9)  # Young's modulus (0.01 GPa)
        msection_cable.SetRayleighDamping(0.0)  # Set Rayleigh damping to zero

        beam_length = 0.5
        chain_spacing_x = 0.3  # Spacing between start points of chains along X-axis
        start_y_offset = 0.5   # Initial Y offset for the start of the beams

        for i in range(self.n_chains):
            # Create a ChBuilderCableANCF helper object for each chain's beam
            builder = fea.ChBuilderCableANCF()

            # Number of ANCF elements: increasing with each chain
            num_elements = 5 + i * 2

            # Define start and end points for the current chain's beam
            start_point = chrono.ChVector3d(i * chain_spacing_x, start_y_offset, -0.1)
            end_point = chrono.ChVector3d(i * chain_spacing_x + beam_length, start_y_offset, -0.1)

            # Build the beam using ANCF elements and add to the common mesh
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            # --- Boundary Conditions and Connections ---

            # 1. Hinge the start node of the beam to a fixed point in space
            start_node_beam = builder.GetBeamNodes().front()

            # Create a fixed body to act as the anchor for the hinge
            fixed_anchor_body = chrono.ChBody()
            fixed_anchor_body.SetFixed(True)
            fixed_anchor_body.SetPos(start_node_beam.GetPos()) # Anchor at the beam's start position
            self.system.Add(fixed_anchor_body)

            # Create a hinge constraint (spherical joint) between the start node and the fixed anchor
            constraint_hinge = fea.ChLinkPointFrame()
            constraint_hinge.Initialize(start_node_beam, fixed_anchor_body)
            self.system.Add(constraint_hinge)

            # 2. Connect the end node of the beam to a ChBody (a small box)
            end_node_beam = builder.GetBeamNodes().back()

            box_size = 0.05  # Size of the box (5 cm side length)
            box_density = 2000  # Density of the box material (kg/m^3)
            
            box_initial_pos = end_node_beam.GetPos()

            box_body = chrono.ChBodyEasyBox(
                box_size, box_size, box_size,
                box_density,
                True,  # Enable visualization
                True   # Enable collision
            )
            box_body.SetPos(box_initial_pos)
            self.system.Add(box_body)
            self.end_bodies.append(box_body)

            # Create a constraint to connect the beam's end node to the center of the box
            link_node_to_box = fea.ChLinkPointFrame()
            link_node_to_box.Initialize(end_node_beam, box_body)
            self.system.Add(link_node_to_box)

            # 3. Apply an external force to the box_body
            force_magnitude = 0.5 + i * 0.1 
            applied_force_vector = chrono.ChVector3d(0, -force_magnitude, 0)
            
            # Create a ChLoadBodyForce object to apply this constant force to the box's COG
            constant_force_load = chrono.ChLoadBodyForce(
                box_body,                    # The body to apply the force to
                applied_force_vector,        # Force vector (in global coordinates)
                False,                       # False: force is in global coordinates
                chrono.ChFrameMovingD()      # Apply at COG (default relative frame)
            )
            self.system.AddLoad(constant_force_load)

    def PrintBodyPositions(self):
        # Prints the positions of the end bodies of each chain
        print(f"Time: {self.system.GetChTime():.4f}")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"  Chain {i} Box Position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")

# --- Main Simulation Setup ---

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0)) # Set system gravity

mesh = fea.ChMesh()
# Note: Mesh is populated by Model1, then added to the system.
# Or, add mesh to system first, then Model1 populates it. Both work.
# sys.Add(mesh) # Option 1: Add mesh to system here

# Create the model, which will generate chains, bodies, and constraints
# This will use the default n_chains=6 unless specified otherwise
model = Model1(sys, mesh) 

sys.Add(mesh) # Option 2: Add mesh to system after Model1 populates it (as in original script)


# --- Visualization Setup ---

# FEM mesh visualization
# Visualization for beam element data (e.g., bending moment Mz)
vis_beam_elements = chrono.ChVisualShapeFEA(mesh)
vis_beam_elements.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_elements.SetColorscaleMinMax(-0.4, 0.4) 
vis_beam_elements.SetSmoothFaces(True)
vis_beam_elements.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_elements)

# Visualization for FEA nodes (e.g., as dots)
vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)
vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_nodes.SetSymbolsThickness(0.006)
vis_beam_nodes.SetSymbolsScale(0.01) 
vis_beam_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_nodes)

# Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
# Adjusted camera for better view of multiple chains
vis.AddCamera(chrono.ChVector3d(0.8, 0.6, -2.0)) 
vis.AddTypicalLights()

# --- Solver and Timestepper Setup ---

# Solver
solver = chrono.ChSolverMINRES() # Choose MINRES solver
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)  # Important for EULER_IMPLICIT_LINEARIZED
solver.SetVerbose(False)

# Timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# --- Simulation Loop ---
time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    model.PrintBodyPositions() # Call the function to print positions
    vis.EndScene()
    sys.DoStepDynamics(time_step)