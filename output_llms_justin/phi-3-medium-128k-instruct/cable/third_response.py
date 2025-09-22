import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of beams composed of ANCF cable elements, with one end hinged to
# ground, moving under gravity alone.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.sys = system
        self.mesh = mesh
        self.visualizebeamA = None
        self.visualizebeamB = None
        self.truss_bodies = []
        self.chain_lengths = [0.5 + i * 0.1 for i in range(n_chains)]  # Different lengths for each chain
        self.end_positions = [chrono.ChVector3d(0, 0, -0.1)  # Starting point for all chains

        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section

        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()

        # Create visualization for the FEM mesh
        self.visualizebeamA = chrono.ChVisualShapeFEA(mesh)
        self.visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
        self.visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
        self.visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
        self.visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
        mesh.AddVisualShapeFEA(self.visualizebeamA)  # Add the visualization shape to the mesh

        # Create visualization for node positions
        self.visualizebeamB = chrono.ChVisualShapeFEA(mesh)
        self.visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
        self.visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
        self.visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
        self.visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
        self.visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
        mesh.AddVisualShapeFEA(self.visualizebeamB)  # Add the node visualization to the mesh

        # Create the Irrlicht visualization for rendering
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(self.sys)  # Attach Irrlicht to the Chrono system
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
            self.sys.SetSolver(solver)
            solver.SetMaxIterations(200)
            solver.SetTolerance(1e-10)
            solver.EnableDiagonalPreconditioner(True)
            solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
            solver.SetVerbose(False)
        # Set the timestepper for the simulation
        self.ts = chrono.ChTimestepperEulerImplicitLinearized(self.sys)
        self.sys.SetTimestepper(self.ts)

        # Create the model and add the mesh to the system
        self.model = self.CreateModel()
        self.sys.Add(self.model)  # Remember to add the mesh to the physical system

    def CreateModel(self):
        # Create the physical system and mesh container:
        sys = chrono.ChSystemSMC()
        mesh = fea.ChMesh()

        # Create the model and add the mesh to the system
        model = Model1(sys, mesh)
        sys.Add(mesh)

        # Add visualization for the FEM mesh
        self.visualizebeamA = chrono.ChVisualShapeFEA(mesh)
        self.visualizebeamB = chrono.ChVisualShapeFEA(mesh)

        # Create the Irrlicht visualization for rendering
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle('FEA cables')
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
        vis.AddTypicalLights()

        # Set solver type and settings
        solver = chrono.ChSolverMINRES()
        self.sys.SetSolver(solver)
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableDiagonalPreconditioner(True)
        solver.EnableWarmStart(True)
        self.sys.SetTimestepper(self.ts)

        return model

    def PrintBodyPositions(self):
        for i in range(self.n_chains):
            print(f"Chain {i+1} end body position: {self.truss_bodies[i].GetPos()}")

    def RunSimulation(self):
        while vis.Run():
            vis.BeginScene()  # Begin scene rendering
            vis.Render()  # Render the scene
            vis.EndScene()  # End scene rendering
            self.sys.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds
            self.PrintBodyPositions()  # Print positions of the end bodies of each chain

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model
model = Model1(sys, mesh)

# Run the simulation
model.RunSimulation()