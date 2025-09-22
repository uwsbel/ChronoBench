import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of beam elements with connected bodies
# This model demonstrates the use of multiple ANCF cable elements with bodies
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # Store end bodies for position printing
        
        # Create multiple chains
        for i in range(n_chains):
            self.create_chain(i)
    
    def create_chain(self, chain_index):
        # Create a section for each chain
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable.SetRayleighDamping(0.0001)  # Set Rayleigh damping
        
        # Create a ChBuilderCableANCF helper object
        builder = fea.ChBuilderCableANCF()
        
        # Calculate positions to avoid overlap
        x_offset = chain_index * 0.15  # Horizontal spacing between chains
        start_point = chrono.ChVector3d(x_offset, 0, 0)
        end_point = chrono.ChVector3d(x_offset, -0.5, 0)
        
        # Number of elements increases with each chain (minimum 5)
        num_elements = 5 + chain_index * 2
        
        # Build the beam
        builder.BuildBeam(
            self.mesh,  # The mesh to which the created nodes and elements will be added
            msection_cable,  # The beam section properties to use
            num_elements,  # Number of ANCF elements to create along the beam
            start_point,  # Starting point of the beam
            end_point   # Ending point of the beam
        )
        
        # Create a truss body (fixed reference frame) for each chain
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        mtruss.SetPos(start_point)
        self.system.Add(mtruss)
        
        # Create and initialize a hinge constraint to fix beam's start point to the truss
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
        self.system.Add(constraint_hinge)
        
        # Create a box body at the end of each chain
        end_body = chrono.ChBody()
        end_body.SetMass(0.1)
        end_body.SetPos(end_point + chrono.ChVector3d(0, -0.05, 0))
        
        # Set inertia for the box
        end_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
        
        # Add visual shape to the box
        box_shape = chrono.ChVisualShapeBox(0.02, 0.02, 0.02)
        box_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
        end_body.AddVisualShape(box_shape)
        
        # Add collision shape to the box
        end_body.EnableCollision(True)
        end_body.GetCollisionModel().ClearModel()
        end_body.GetCollisionModel().AddBox(0.01, 0.01, 0.01)
        end_body.GetCollisionModel().BuildModel()
        
        self.system.Add(end_body)
        self.end_bodies.append(end_body)
        
        # Connect the beam's end node to the box body
        constraint_end = fea.ChLinkNodeFrame()
        constraint_end.Initialize(builder.GetLastBeamNodes().back(), end_body)
        self.system.Add(constraint_end)
        
        # Apply force to the end body
        end_body.SetForce(chrono.ChVector3d(0, -0.1, 0))
    
    def PrintBodyPositions(self):
        """Print the positions of the end bodies of each chain"""
        print(f"Time: {self.system.GetChTime():.3f}s")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"  Chain {i}: Position = ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")
        print()

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with multiple chains and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Add visualization for the FEM mesh:
# This allows visualization of the forces/moments in the beam elements:
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visualization shape to the mesh

# Add visualization for node positions:
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
mesh.AddVisualShapeFEA(visualizebeamB)  # Add the node visualization to the mesh

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach Irrlicht to the Chrono system
vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
vis.SetWindowTitle('FEA cables - Multiple Chains')  # Set the title of the rendering window
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0.5, 0.6, -1))  # Add a camera with specific position
vis.AddTypicalLights()  # Add typical lights for better illumination

# Set solver type and settings
solver = chrono.ChSolverMINRES()  # Choose MINRES solver
print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
solver.SetVerbose(False)

# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
step_count = 0
print_interval = 50  # Print positions every 50 steps

while vis.Run():
    vis.BeginScene()  # Begin scene rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene rendering
    sys.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds
    
    # Print body positions periodically
    if step_count % print_interval == 0:
        model.PrintBodyPositions()
    
    step_count += 1