import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.mesh = mesh
        self.system = system

        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section

        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()
        # Initialize variables for loop
        start_pos = chrono.ChVectorD(0, 0, -0.1)
        end_pos = chrono.ChVectorD(0.5, 0, -0.1)
        total_length = (end_pos - start_pos).Length()

        # Loop to create multiple chains
        for i in range(n_chains):
            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)

            # Set the starting position for the next chain
            start_pos = end_pos

            # Create a new end position for the next chain
            end_pos = start_pos + chrono.ChVectorD(total_length / n_chains * (i + 1), 0, 0)

            # Calculate the new length of the chain
            new_length = (end_pos - start_pos).Length()

            # Build the beam structure with the new length
            builder.BuildBeam(
                self.mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                int(new_length / (total_length / n_chains)),  # Number of ANCF elements to create along the beam
                start_pos,  # Starting point ('A' point) of the beam
                end_pos  # Ending point ('B' point) of the beam
            )

            # Add the constraint to the system
            system.Add(constraint_hinge)

            # Apply load to the front node of the chain
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVectorD(0, -0.7, 0))

        # Create a visualization object for the FEM mesh
        visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
        visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
        visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
        visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
        self.mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visualization shape to the mesh

        # Create a visualization object for node positions
        visualizebeamB = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
        visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
        visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
        visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
        visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
        self.mesh.AddVisualShapeFEA(visualizebeamB)  # Add the node visualization to the mesh

        # Create the Irrlicht visualization for rendering
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)  # Attach Irrlicht to the Chrono system
        vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
        vis.SetWindowTitle('FEA cables')  # Set the title of the rendering window
        vis.Initialize()  # Initialize the visualization
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
        vis.AddSkyBox()  # Add a skybox for better aesthetics
        vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))  # Add a camera with specific position
        vis.AddTypicalLights()  # Add typical lights for better illumination

        # Set solver type and settings
        solver = chrono.ChSolverMINRES()  # Choose SparseQR solver
        if solver.GetType() == chrono.ChSolver.Type_MINRES:
            print("Using MINRES solver")
            system.SetSolver(solver)
            solver.SetMaxIterations(200)
            solver.SetTolerance(1e-10)
            solver.EnableDiagonalPreconditioner(True)
            solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
            solver.SetVerbose(False)

        # Set the timestepper for the simulation
        ts = chrono.ChTimestepperEulerImplicitLinearized(system)
        system.SetTimestepper(ts)

        # Simulation loop
        while vis.Run():
            vis.BeginScene()  # Begin scene rendering
            vis.Render()  # Render the scene
            vis.EndScene()  # End scene rendering
            system.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds

    def PrintBodyPositions(self):
        for i in range(self.n_chains):
            print(f"Chain {i+1} end position: {self.mesh.GetNodePos(self.mesh.GetLastNode(i))}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Call the PrintBodyPositions function to print the positions of the end bodies in each chain
model.PrintBodyPositions()