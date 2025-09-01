import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system  
        self.mesh = mesh      
        self.bodies = []      

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  
       
        
        for i in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            self.system.Add(mtruss)

            
            builder = fea.ChBuilderCableANCF()

            
            num_elements = 10 + i * 2  

            
            start_x = i * 0.2  
            start_point = chrono.ChVector3d(start_x, 0, -0.1)
            end_point = chrono.ChVector3d(start_x + 0.5, 0, -0.1)

            
            builder.BuildBeam(
                self.mesh,  
                msection_cable2,  
                num_elements,  
                start_point,  
                end_point  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetFirstBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)  

            
            box_size = chrono.ChVector3d(0.05, 0.05, 0.05)  
            box_body = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000, True, True)  
            box_body.SetPos(end_point)  
            self.system.Add(box_body)  
            self.bodies.append(box_body)  

            
            constraint_ball = chrono.ChLinkLockSpherical()  
            constraint_ball.Initialize(builder.GetLastBeamNodes().front(), box_body)  
            self.system.Add(constraint_ball)

    def PrintBodyPositions(self):
        
        for i, body in enumerate(self.bodies):
            print(f"Chain {i+1} End Body Position: {body.GetPos()}")



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
    model.PrintBodyPositions()