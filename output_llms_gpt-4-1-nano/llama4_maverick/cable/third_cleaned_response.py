import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np








class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []
        
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        for i in range(self.n_chains):
            
            builder = fea.ChBuilderCableANCF()
            
            
            num_elements = 10 + i  
            start_point = chrono.ChVector3d(i * 0.1, 0, -0.1)  
            end_point = chrono.ChVector3d(i * 0.1 + 0.5, 0, -0.1)
            builder.BuildBeam(
                self.mesh,  
                msection_cable2,  
                num_elements,  
                start_point,  
                end_point  
            )

            
            builder.GetLastBeamNodes().front().SetFixed(True)  
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            mtruss.SetPos(start_point)
            self.system.Add(mtruss)  

            
            constraint_hinge_start = fea.ChLinkNodeFrame()
            constraint_hinge_start.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            self.system.Add(constraint_hinge_start)  

            
            end_body = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)
            end_body.SetPos(end_point)
            self.system.Add(end_body)
            self.end_bodies.append(end_body)

            constraint_hinge_end = fea.ChLinkNodeBody()
            constraint_hinge_end.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_hinge_end)

        
        visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
        visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
        visualizebeamA.SetSmoothFaces(True)  
        visualizebeamA.SetWireframe(False)  
        self.mesh.AddVisualShapeFEA(visualizebeamA)  

        
        visualizebeamB = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
        visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
        visualizebeamB.SetSymbolsThickness(0.006)  
        visualizebeamB.SetSymbolsScale(0.01)  
        visualizebeamB.SetZbufferHide(False)  
        self.mesh.AddVisualShapeFEA(visualizebeamB)  

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i+1} end body position: {body.GetPos()}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)  


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