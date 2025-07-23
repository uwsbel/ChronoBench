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

		
		for i in range(n_chains):
			
			mtruss = chrono.ChBody()
			mtruss.SetPos(chrono.ChVector3d(0, 0, -0.1*i))
			mtruss.SetFixed(True)
			system.Add(mtruss)

			
			builder = fea.ChBuilderCableANCF()

			
			builder.BuildBeam(
				mesh,  
				msection_cable2,  
				i + 1,  
				chrono.ChVector3d(0, 0, -0.1*(i+1)),  
				chrono.ChVector3d(0.1*(i + 1), 0, -0.1*(i+1))  
			)

			
			
			builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

			
			self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04))
			
			self.bodies[i].SetPos(builder.GetLastBeamNodes().back().GetPos() +
								  chrono.ChVector3d(0.1, 0, 0))
			system.Add(self.bodies[i])

			
			constraint_hinge = fea.ChLinkNodeFrame()
			constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), self.bodies[i])
			system.Add(constraint_hinge)

			
			constraint_truss = chrono.ChLinkMateGeneric()
			constraint_truss.Initialize(self.bodies[i], mtruss, False,
										chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos()))
			system.Add(constraint_truss)

			constraint_truss.SetConstrainedCoords(True, True, True, False, False, False)

			
			self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04))
			
			self.bodies[i + 1].SetPos(builder.GetLastBeamNodes().back().GetPos() +
									  chrono.ChVector3d(0.1, 0, 0))
			system.Add(self.bodies[i + 1])

			
			constraint_hinge = fea.ChLinkNodeFrame()
			constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), self.bodies[i + 1])
			system.Add(constraint_hinge)

			
			constraint_truss = chrono.ChLinkMateGeneric()
			constraint_truss.Initialize(self.bodies[i + 1], mtruss, False,
										chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos()))
			system.Add(constraint_truss)

			constraint_truss.SetConstrainedCoords(True, True, True, False, False, False)


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
if solver.GetType() == chrono.ChSolver.Type_MINRES:
	print("Using MINRES solver")
	sys.SetSolver(solver)
	solver.SetMaxIterations(200)
	solver.SetTolerance(1e-10)
	solver.EnableDiagonalPreconditioner(True)
	solver.EnableWarmStart(True)  
	solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)



def  PrintBodyPositions():
	for i in range(len(model.bodies)//2):
		print( "[" , i+1 ,  "]  " , model.bodies[i].GetPos() , "  " , model.bodies[i+1].GetPos() )


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)  
    PrintBodyPositions()