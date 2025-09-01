import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of multiple chains of ANCF cable elements, each chain
# connected to a fixed truss body. Each chain is loaded with forces and hinged
# to a fixed point. The model demonstrates the use of ChBuilderCableANCF for
# creating cable elements and ChLinkNodeFrame for creating hinge constraints.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.system = system
        self.mesh = mesh
        self.visualizations = []
        
        # Create material properties for the cable section
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  # 15 mm diameter
        msection_cable.SetYoungModulus(0.01e9)  # 0.01 GPa
        msection_cable.SetRayleighDamping(0.0001)
        
        # Create a helper object for building ANCF beams
        self.builder = fea.ChBuilderCableANCF()
        
        # Create meshes for each chain
        self.meshes = []
        for i in range(self.n_chains):
            chain_mesh = fea.ChMesh()
            self.meshes.append(chain_mesh)
            self.system.Add(chain_mesh)
            self.meshes[i].AddVisualShapeFEA(self.create_beam_visualization(i))
        
        # Generate each chain
        for i in range(self.n_chains):
            # Create a truss body for the hinge
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)
            
            # Create the beam for the current chain
            num_elements = 10 + i  # Increase number of elements per chain
            start_point = chrono.ChVector3d(0.0, 0.0, -0.1 - 0.1 * i)
            end_point = chrono.ChVector3d(0.5, 0.0, -0.1 - 0.1 * i)
            
            # Build the ANCF beam
            self.builder.BuildBeam(
                self.meshes[i],
                msection_cable,
                num_elements,
                start_point,
                end_point
            )
            
            # Apply forces and constraints
            last_node = self.builder.GetLastBeamNodes().back()
            last_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Create hinge constraint
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(last_node, mtruss)
            self.system.Add(constraint)
            
            # Add visualization for the current chain
            self.meshes[i].AddVisualShapeFEA(self.create_beam_visualization(i))
            self.meshes[i].AddVisualShapeFEA(self.create_node_visualization(i))
            
            # Print initial positions
            self.print_body_positions(i)
            
    def create_beam_visualization(self, chain_id):
        visualize = chrono.ChVisualShapeFEA(self.meshes[chain_id])
        visualize.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        visualize.SetColorscaleMinMax(-0.4, 0.4)
        visualize.SetSmoothFaces(True)
        visualize.SetWireframe(False)
        return visualize
        
    def create_node_visualization(self, chain_id):
        visualize = chrono.ChVisualShapeFEA(self.meshes[chain_id])
        visualize.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        visualize.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        visualize.SetSymbolsThickness(0.006)
        visualize.SetSymbolsScale(0.01)
        visualize.SetZbufferHide(False)
        return visualize
        
    def print_body_positions(self, chain_id):
        end_body = self.builder.GetLastBeamNodes().back()
        pos = end_body.GetPosition()
        print(f"Chain {chain_id + 1} end body position: {pos.x}, {pos.y}, {pos.z}")
        
    def __init__(self, system, mesh):
        pass  # This __init__ method is replaced by the one above

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Add visualization for the FEM mesh:
# This allows visualization of the forces/moments in the beam elements:
model.mesh.AddVisualShapeFEA(chrono.ChVisualShapeFEA(mesh))
model.mesh.AddVisualShapeFEA(chrono.ChVisualShapeFEA(mesh))

# Add visualization for node positions:
model.mesh.AddVisualShapeFEA(model.create_node_visualization(0))

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach Irrlicht to the Chrono system
vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
vis.SetWindowTitle('FEA cables')  # Set the title of the rendering window
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  # Add a camera with specific position
vis.AddTypicalLights()  # Add typical lights for better illumination

# Set solver type and settings
solver = chrono.ChSolverMINRES()  # Choose SparseQR solver
if solver.GetType() == chrono.ChSolver.Type_MINRES:
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
while vis.Run():
    vis.BeginScene()  # Begin scene rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene rendering
    sys.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds

    # Print positions of all end bodies
    for i in range(model.n_chains):
        model.print_body_positions(i)