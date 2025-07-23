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

		# for each chain
		for i in range(n_chains):
			# create a truss body
			mtruss = chrono.ChBody()
			mtruss.SetPos(chrono.ChVector3d(0, 0, -0.1*i))
			mtruss.SetFixed(True)
			system.Add(mtruss)

			# builder for creating a beam from ANCF elements
			builder = fea.ChBuilderCableANCF()

			# Use BuildBeam to create a beam structure consisting of ANCF elements:
			builder.BuildBeam(
				mesh,  # The mesh to which the created nodes and elements will be added
				msection_cable2,  # The beam section properties to use
				i + 1,  # number of ANCF elements to create along the beam
				chrono.ChVector3d(0, 0, -0.1*(i+1)),  # Starting point ('A' point) of the beam
				chrono.ChVector3d(0.1*(i + 1), 0, -0.1*(i+1))  # Ending point ('B' point) of the beam
			)

			# apply boundary conditions and loads:
			# retrieve the end nodes of the beam and apply load/constraints
			builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

			# make a box and connect it to the beam
			self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04))
			# position it
			self.bodies[i].SetPos(builder.GetLastBeamNodes().back().GetPos() +
								  chrono.ChVector3d(0.1, 0, 0))
			system.Add(self.bodies[i])

			# make a hinge between box and beam
			constraint_hinge = fea.ChLinkNodeFrame()
			constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), self.bodies[i])
			system.Add(constraint_hinge)

			# make a constraint between box and truss
			constraint_truss = chrono.ChLinkMateGeneric()
			constraint_truss.Initialize(self.bodies[i], mtruss, False,
										chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos()))
			system.Add(constraint_truss)

			constraint_truss.SetConstrainedCoords(True, True, True, False, False, False)

			# make a box and connect it to the beam
			self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04))
			# position it
			self.bodies[i + 1].SetPos(builder.GetLastBeamNodes().back().GetPos() +
									  chrono.ChVector3d(0.1, 0, 0))
			system.Add(self.bodies[i + 1])

			# make a hinge between box and beam
			constraint_hinge = fea.ChLinkNodeFrame()
			constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), self.bodies[i + 1])
			system.Add(constraint_hinge)

			# make a constraint between box and truss
			constraint_truss = chrono.ChLinkMateGeneric()
			constraint_truss.Initialize(self.bodies[i + 1], mtruss, False,
										chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos()))
			system.Add(constraint_truss)

			constraint_truss.SetConstrainedCoords(True, True, True, False, False, False)

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


# function to print position of end bodies
def  PrintBodyPositions():
	for i in range(len(model.bodies)//2):
		print( "[" , i+1 ,  "]  " , model.bodies[i].GetPos() , "  " , model.bodies[i+1].GetPos() )

# Simulation loop
while vis.Run():
    vis.BeginScene()  # Begin scene rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene rendering
    sys.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds
    PrintBodyPositions()  # call  function  to  print  body   positions