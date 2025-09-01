import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Creates multiple chains of beam elements. Each chain consists of 
# a series of ANCF cable elements connected to a truss body. The endpoint 
# of each beam is further connected to a chrono body (box).
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system  # Store the system as an attribute
        self.mesh = mesh      # Store the mesh as an attribute
        self.bodies = []      # List to store the end bodies of each chain

        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section
       
        # Loop to create multiple chains
        for i in range(n_chains):
            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body
            self.system.Add(mtruss)

            # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
            builder = fea.ChBuilderCableANCF()

            # Define the number of elements for the current beam
            num_elements = 10 + i * 2  # Increase the number of elements with each chain

            # Calculate starting and ending points to avoid overlap
            start_x = i * 0.2  # Adjust spacing between chains
            start_point = chrono.ChVector3d(start_x, 0, -0.1)
            end_point = chrono.ChVector3d(start_x + 0.5, 0, -0.1)

            # Use BuildBeam to create a beam structure consisting of ANCF elements:
            builder.BuildBeam(
                self.mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                num_elements,  # Number of ANCF elements to create along the beam
                start_point,  # Starting point ('A' point) of the beam
                end_point  # Ending point ('B' point) of the beam
            )

            # Apply boundary conditions and loads:
            # Retrieve the end nodes of the beam and apply load/constraints
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  # Apply forces to the front node

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetFirstBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)  # Add the constraint to the system

            # Create a chrono body (box) to connect to the beam's endpoint
            box_size = chrono.ChVector3d(0.05, 0.05, 0.05)  # Size of the box
            box_body = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000, True, True)  # Create box body
            box_body.SetPos(end_point)  # Set initial position of the box
            self.system.Add(box_body)  # Add the box to the system
            self.bodies.append(box_body)  # Store the box body

            # Connect the beam endpoint to the box
            constraint_ball = chrono.ChLinkLockSpherical()  # Use a spherical joint for connection
            constraint_ball.Initialize(builder.GetLastBeamNodes().front(), box_body)  # Connect beam to box
            self.system.Add(constraint_ball)

    def PrintBodyPositions(self):
        """Prints the positions of the end bodies of each chain."""
        for i, body in enumerate(self.bodies):
            print(f"Chain {i+1} End Body Position: {body.GetPos()}")


# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
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
solver = chrono.ChSolverMINRES() # Choose SparseQR solver
if solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
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
    model.PrintBodyPositions() # Print positions at each step