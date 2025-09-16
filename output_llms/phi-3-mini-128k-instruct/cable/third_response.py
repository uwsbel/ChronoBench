import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of beam elements with connected bodies
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chains = []
        self.system = system
        self.mesh = mesh

        # Create and initialize chains
        for i in range(self.n_chains):
            # Create a section, i.e. define thickness and material properties for the cable beam
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
            msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
            msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section
            # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
            builder = fea.ChBuilderCableANCF()
            # Use BuildBeam to create a beam structure consisting of ANCF elements:
            elements_per_chain = 10 + i * 5  # Increase the number of ANCF elements with each chain
            builder.BuildBeam(
                mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                elements_per_chain,  # Number of ANCF elements to create along the beam
                chrono.ChVector3d(0, 0, -0.1 + i * 0.05),  # Starting point ('A' point) of the beam
                chrono.ChVector3d(0.5 + i * 0.1, 0, -0.1 + i * 0.05)  # Ending point ('B' point) of the beam
            )

            # Retrieve the end nodes of the beam and apply boundary conditions and loads:
            end_nodes = builder.GetLastBeamNodes().front()
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  # Apply forces to the front node

            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_nodes, mtruss)
            self.system.Add(constraint_hinge)  # Add the constraint to the system

            # Create a box body to connect the beam's endpoint to
            mbox = chrono.ChBody()
            mbox.SetBodyFixed(True)  # Fix the box body
            mbox.SetBodyPosition(end_nodes.GetPosition())
            self.system.Add(mbox)

            # Establish constraints between the beam's endpoint and the box
            constraint_box = fea.ChLinkFrame()
            constraint_box.Initialize(end_nodes, mbox)
            self.system.Add(constraint_box)

            # Add visualization for the FEM mesh and the connected box
            visualizebeamA = chrono.ChVisualShapeFEA(mesh)
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
            visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
            visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
            visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
            mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visualization shape to the mesh

            visualizebeamB = chrono.ChVisualShapeFEA(mesh)
            visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
            visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
            visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
            visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
            visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
            mesh.AddVisualShapeFEA(visualizebeamB)  # Add the node visualization to the mesh

            # Create the Irrlicht visualization for rendering
            vis = chronoirr.ChVisualSystemIrrlicht()
            vis.AttachSystem(self.system)  # Attach Irrlicht to the Chrono system
            vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
            vis.SetWindowTitle('FEA cables')  # Set the title of the rendering window
            vis.Initialize()  # Initialize the visualization
            vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
            vis.AddSkyBox()  # Add a skybox for better aesthetics
            vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  # Add a camera with specific position
            vis.AddTypicalLights()  # Add typical lights for better illumination

            # Add visualization for the connected box
            visualizebox = chrono.ChVisualShapeFEA(mesh)
            visualizebox.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BOX_MZ)  # Display moments along the box
            visualizebox.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
            visualizebox.SetSmoothFaces(True)  # Enable smooth faces for better visualization
            visualizebox.SetWireframe(False)  # Set to non-wireframe mode
            mesh.AddVisualShapeFEA(visualizebox)  # Add the visualization shape to the mesh

            # Add visualization for node positions
            visualizenode = chrono.ChVisualShapeFEA(mesh)
            visualizenode.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
            visualizenode.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
            visualizenode.SetSymbolsThickness(0.006)  # Set thickness of symbols
            visualizenode.SetSymbolsScale(0.01)  # Set scale of symbols
            visualizenode.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
            mesh.AddVisualShapeFEA(visualizenode)  # Add the node visualization to the mesh

            # Print body positions function
            def PrintBodyPositions():
                for chain in self.chains:
                    print(f"Chain {chain.index}: Box position = {chain.box.GetBodyPosition()}, Truss position = {chain.truss.GetBodyPosition()}")

        # Add the PrintBodyPositions function to the system
        self.system.Add(self.PrintBodyPositions)

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

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
    model.PrintBodyPositions()  # Print body positions at each step

# Test the PrintBodyPositions function
model.PrintBodyPositions()