import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Model1: A beam composed of 10 ANCF cable element, with one end hinged to
# ground, moving under gravity alone.
# This model demonstrates the use of the utility class ChBuilderCableANCF.

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.end_bodies = []
        self.n_chains = n_chains
        for n in range(self.n_chains) :
            # Create a section, i.e. define thickness and material properties for the cable beam
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
            msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
            msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section
            # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
            builder = fea.ChBuilderCableANCF()
            # Use BuildBeam to create a beam structure consisting of ANCF elements:
            builder.BuildBeam(
                mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                5+ n,  # Number of ANCF elements to create along the beam
                chrono.ChVector3d(0, 0, -0.1),  # Starting point ('A' point) of the beam
                chrono.ChVector3d(0.5, 0, -0.1)  # Ending point ('B' point) of the beam
            )

            # Apply boundary conditions and loads:
            # Retrieve the end nodes of the beam and apply load/constraints
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  # Apply forces to the front node

            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  # Add the constraint to the system

            # Now create the second beam and connect it to the first one
            # Create a body for the second beam to connect
            mbody = chrono.ChBodyEasyBox(0.2, 0.02, 0.02, 1000)
            mbody.SetPos(chrono.ChVector3d(
                0.6, 0.01 * (n - self.n_chains / 2), -0.1))  # Position the body in 3D space
            mbody.SetFixed(False)  # Set the body as dynamic (not fixed)
            system.AddBody(mbody)  # Add the body to the system

            # Create and initialize a hinge constraint to connect the beam's end point to the body
            constraint_hinge_11 = fea.ChLinkNodeFrame()
            constraint_hinge_11.Initialize(builder.GetLastBeamNodes().back(), mbody)
            system.Add(constraint_hinge_11)  # Add the constraint to the system

            # Create and initialize another hinge constraint to connect a point on the second body to the beam nodes
            constraint_hinge_12 = fea.ChLinkNodeFrame()
            constraint_hinge_12.Initialize(
                mbody, builder.GetLastBeamNodes().back() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(constraint_hinge_12)  # Add the constraint to the system

            if n != self.n_chains -1:
                # If not the last chain, create a body to connect to the next chain
                mbody_to_connect = chrono.ChBodyEasyBox(0.2, 0.02, 0.02, 1000)
                mbody_to_connect.SetPos(chrono.ChVector3d(
                    0.6, 0.01 * (n + 1 - self.n_chains / 2), -0.1))  # Position the connecting body
                mbody_to_connect.SetFixed(False)  # Set the body as dynamic
                system.AddBody(mbody_to_connect)  # Add the body to the system

                # Create and initialize a hinge constraint to connect the second body's point to the next connecting body
                constraint_hinge_21 = fea.ChLinkNodeFrame()
                constraint_hinge_21.Initialize(
                    mbody, mbody_to_connect + chrono.ChVector3d(0.1, 0, 0))
                system.Add(constraint_hinge_21)  # Add the constraint to the system

                # Create and initialize another hinge constraint to fix the next connecting body to the beam nodes
                constraint_hinge_22 = fea.ChLinkNodeFrame()
                constraint_hinge_22.Initialize(
                    builder.GetLastBeamNodes().back(), mbody_to_connect)
                system.Add(constraint_hinge_22)  # Add the constraint to the system

            else:
                # If last chain, just save the end body
                self.end_bodies.append(mbody)

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

    def PrintBodyPositions(self):
        for b in self.end_bodies:
            print(b.GetPos())
# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

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