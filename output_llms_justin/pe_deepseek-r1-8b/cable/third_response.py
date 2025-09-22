import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple beam chains composed of ANCF cable elements, each connected to a truss
# Each chain has an increasing number of ANCF elements and is connected to a fixed truss body
# The model demonstrates multiple chains with different lengths and forces applied
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chain_length = [5, 7, 9, 11, 13, 15]  # Number of ANCF elements per chain
        
        # Create a section for the cable elements
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  # 15 mm diameter
        msection_cable.SetYoungModulus(0.01e9)  # 0.01 GPa Young's modulus
        msection_cable.SetRayleighDamping(0.0001)  # Very low Rayleigh damping
        
        # ChBuilderCableANCF helper object
        builder = fea.ChBuilderCableANCF()
        
        # Create each chain
        for i in range(self.n_chains):
            # Create a truss body for each chain
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body
            
            # Create a hinge constraint between the chain's end and the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            
            # Position for the hinge constraint (relative to the chain's end point)
            pos_hinge = chrono.ChVector3d(0, 0, 0.1 * (i + 1))  # Raise each chain slightly
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            # Create the beam for the current chain
            beam = builder.BuildBeam(
                mesh,
                msection_cable,
                self.chain_length[i],
                chrono.ChVector3d(0, 0, -0.1 - 0.1 * i),  # Starting position for each chain
                chrono.ChVector3d(0.5, 0, -0.1 - 0.1 * i)  # Ending position for each chain
            )
            
            # Apply forces to the front node of the beam
            beam.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Add visualization for the current chain
            self.AddVisualShapeFEA(beam, i)
            
            # Add the truss body to the system
            system.Add(mtruss)
            
            # Print initial positions of the end bodies
            self.PrintBodyPositions()
            
        # Create visualization for the nodes and beams
        self.visualize_beam = chrono.ChVisualShapeFEA(mesh)
        self.visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        self.visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
        self.visualize_beam.SetSmoothFaces(True)
        self.visualize_beam.SetWireframe(False)
        mesh.AddVisualShapeFEA(self.visualize_beam)
        
        # Create visualization for nodes
        self.visualize_nodes = chrono.ChVisualShapeFEA(mesh)
        self.visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        self.visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        self.visualize_nodes.SetSymbolsThickness(0.006)
        self.visualize_nodes.SetSymbolsScale(0.01)
        self.visualize_nodes.SetZbufferHide(False)
        mesh.AddVisualShapeFEA(self.visualize_nodes)
    
    def AddVisualShapeFEA(self, beam, chain_id):
        # Create visualization for the current beam
        vis_shape = chrono.ChVisualShapeFEA(beam)
        vis_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        vis_shape.SetColorscaleMinMax(-0.4, 0.4)
        vis_shape.SetSmoothFaces(True)
        vis_shape.SetWireframe(False)
        beam.AddVisualShapeFEA(vis_shape)
        
        # Create visualization for the nodes of the current beam
        vis_nodes = chrono.ChVisualShapeFEA(beam)
        vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        vis_nodes.SetSymbolsThickness(0.006)
        vis_nodes.SetSymbolsScale(0.01)
        vis_nodes.SetZbufferHide(False)
        beam.AddVisualShapeFEA(vis_nodes)
    
    def PrintBodyPositions(self):
        print("Printing body positions for each chain:")
        for i in range(self.n_chains):
            end_body = self.chain[i].GetLastBeamNodes().back().GetAttachedBody()
            pos = end_body.GetPos()
            print(f"Chain {i+1}: Position = {pos.x}, {pos.y}, {pos.z}")
    
    def __init__(self, system, mesh):
        pass

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
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

    # Print positions of end bodies for demonstration
    model.PrintBodyPositions()