import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
	def __init__(self, system, mesh, n_chains=6):
		self.bodies = []  
		msection_cable2 = fea.ChBeamSectionCable()
		msection_cable2.SetDiameter(0.015)
		msection_cable2.SetYoungModulus(0.01e9)
		msection_cable2.SetRayleighDamping(0.0001)

		mtruss = chrono.ChBody()
		mtruss.SetFixed(True)

		for j in range(n_chains):
			builder = fea.ChBuilderCableANCF()

			
			builder.BuildBeam(mesh,  
							  msection_cable2,  
							  1 + j,  
							  chrono.ChVector3d(0, 0, -0.1 * j),  
							  chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j)  
							  )

			builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.2, 0))

			constraint_hinge = fea.ChLinkNodeFrame()
			constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
			system.Add(constraint_hinge)

			msphere = chrono.ChVisualShapeSphere(0.02)
			constraint_hinge.AddVisualShape(msphere)

			
			mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)
			mbox.SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
			system.Add(mbox)

			constraint_pos = fea.ChLinkNodeFrame()
			constraint_pos.Initialize(builder.GetLastBeamNodes().back(), mbox)
			system.Add(constraint_pos)

			constraint_dir = fea.ChLinkNodeSlopeFrame()
			constraint_dir.Initialize(builder.GetLastBeamNodes().back(), mbox)
			constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
			system.Add(constraint_dir)

			
			builder.BuildBeam(
				mesh,  
				msection_cable2,  
				1 + (n_chains - j),  
				chrono.ChVector3d(mbox.GetPos().x + 0.1, 0, -0.1 * j),  
				chrono.ChVector3d(mbox.GetPos().x + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j)  
			)

			constraint_pos2 = fea.ChLinkNodeFrame()
			constraint_pos2.Initialize(builder.GetLastBeamNodes().front(), mbox)
			system.Add(constraint_pos2)

			constraint_dir2 = fea.ChLinkNodeSlopeFrame()
			constraint_dir2.Initialize(builder.GetLastBeamNodes().front(), mbox)
			constraint_dir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
			system.Add(constraint_dir2)

			
			self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000))
			self.bodies[j].SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
			system.Add(self.bodies[j])

			constraint_pos3 = fea.ChLinkNodeFrame()
			constraint_pos3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
			system.Add(constraint_pos3)

			constraint_dir3 = fea.ChLinkNodeSlopeFrame()
			constraint_dir3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
			constraint_dir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
			system.Add(constraint_dir3)

	
	def PrintBodyPositions(self):
		print("Time: " + str(self.bodies[0].GetChTime()))
		for body in self.bodies:
			print("  " << body.GetPos())

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)  



visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
visualizebeamA.SetSmoothFaces(True)  
visualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualizebeamA)  


visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
visualizebeamB.SetSymbolsThickness(0.006)  
visualizebeamB.SetSymbolsScale(0.01)  
visualizebeamB.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(visualizebeamB)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA cables')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  
vis.AddTypicalLights()  


solver = chrono.ChSolverMINRES() 
if solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
	sys.SetSolver(solver)
	solver.SetMaxIterations(200)
	solver.SetTolerance(1e-10)
	solver.EnableDiagonalPreconditioner(True)
	solver.EnableWarmStart(True)  
	solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)