import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of multiple ANCF cable elements, with each chain
# connected to a fixed reference frame, moving under gravity alone.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains  # Number of chains
        self.chains = []  # Store the truss bodies for each chain

        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section

        # Loop to create multiple chains
        for i in range(self.n_chains):
            builder = fea.ChBuilderCableANCF()
            num_elements = 10 + i  # Increase the number of elements with each chain

            # Define starting and ending points for each chain
            start_point = chrono.ChVector3d(0, 0, -0.1 - i * 0.1)  # Offset each chain vertically
            end_point = chrono.ChVector3d(0.5, 0, -0.1 - i * 0.1)

            # Use BuildBeam to create a beam structure consisting of ANCF elements:
            builder.BuildBeam(
                mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                num_elements,  # Number of ANCF elements to create along the beam
                start_point,  # Starting point ('A' point) of the beam
                end_point  # Ending point ('B' point) of the beam
            )

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body
            self.chains.append(mtruss)  # Store the truss body

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  # Add the constraint to the system

            # Apply forces to the front node of the beam
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Connect the end of the beam to a box body
            box_body = chrono.ChBody()
            box_body.SetPos(end_point)  # Position the box at the end of the beam
            system.Add(box_body)  # Add the box body to the system

            # Create a hinge constraint between the beam's end node and the box
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(builder.GetLastBeamNodes().back(), box_body)
            system.Add(constraint_box)  # Add the box constraint to the system

        # Add the mesh to the physical system
        system.Add(mesh)

    def PrintBodyPositions(self):
        for i, chain in enumerate(self.chains):
            print(f"Chain {i+1} end position: {chain.GetPos()}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Add visualization for the FEM mesh:
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
    model.PrintBodyPositions()  # Print the positions of the end bodies of each chain