import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A set of beam elements with connected bodies, each with different
#         number of ANCF cable elements.
# ----------------------------------------------------------------------------

class Model1:
	def __init__(self, system, mesh, n_chains=6):
		self.bodies = []  # [chrono.ChBodyEasyBox for i in range(n_chains)]
		msection_cable2 = fea.ChBeamSectionCable()
		msection_cable2.SetDiameter(0.015)
		msection_cable2.SetYoungModulus(0.01e9)
		msection_cable2.SetRayleighDamping(0.0001)

		for j in range(n_chains):
			mtruss = chrono.ChBody()
			mtruss.SetFixed(True)
			system.Add(mtruss)

			builder = fea.ChBuilderCableANCF()
			builder.BuildBeam(mesh,  # the mesh where to put the created nodes and elements
							  msection_cable2,  # beam section properties to use
							  1 + j,  # number of ANCF elements to create along the beam
							  chrono.ChVector3d(0, 0, -0.1 * j),  # starting point ('A' point) of the beam
							  chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j)  # ending point ('B' point) of the beam
							  )

			builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))

			constraint_hinge = fea.ChLinkNodeFrame()
			constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
			system.Add(constraint_hinge)

			msphere = chrono.ChVisualShapeSphere(0.02)
			constraint_hinge.AddVisualShape(msphere)

			mbox = chrono.ChVisualShapeBox(0.2, 0.04, 0.04)
			mbox.SetColor(chrono.ChColor(0.6, 0, 0))
			mbox.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
			mtruss.AddVisualShape(mbox)

			# make a box and connect it
			mbody = chrono.ChBodyEasyBox(0.2, 0.04, 0.04)
			mbody.SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
			system.Add(mbody)

			constraint_pos = fea.ChLinkNodeFrame()
			constraint_pos.Initialize(builder.GetLastBeamNodes().back(), mbody)
			system.Add(constraint_pos)

			constraint_rot = fea.ChLinkNodeFrame()
			constraint_rot.Initialize(builder.GetLastBeamNodes().back(), mbody)
			constraint_rot.SetRelativeCoords(chrono.ChFramed(
				chrono.ChVector3d(0.1, 0, 0), chrono.QuatFromAngleAxis(chrono.CH_PI / 24, chrono.ChVector3d(0, 1, 0))))
			system.Add(constraint_rot)

			self.bodies.append(mbody)

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