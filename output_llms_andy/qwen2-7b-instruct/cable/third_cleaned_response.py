import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.mesh = mesh
        self.system = system

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()
        
        start_pos = chrono.ChVectorD(0, 0, -0.1)
        end_pos = chrono.ChVectorD(0.5, 0, -0.1)
        total_length = (end_pos - start_pos).Length()

        
        for i in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)

            
            start_pos = end_pos

            
            end_pos = start_pos + chrono.ChVectorD(total_length / n_chains * (i + 1), 0, 0)

            
            new_length = (end_pos - start_pos).Length()

            
            total_length += new_length
            builder.BuildBeam(
                self.mesh,  
                msection_cable2,  
                int(total_length / new_length),  
                start_pos,  
                end_pos  
            )

            
            system.Add(constraint_hinge)

            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVectorD(0, -0.7, 0))

            
            system.Add(mtruss)

        
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

        
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(self.system)  
        vis.SetWindowSize(1024, 768)  
        vis.SetWindowTitle('FEA cables')  
        vis.Initialize()  
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
        vis.AddSkyBox()  
        vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))  
        vis.AddTypicalLights()  

        
        solver = chrono.ChSolverMINRES()  
        if solver.GetType() == chrono.ChSolver.Type_MINRES:
            print("Using MINRES solver")
        self.system.SetSolver(solver)
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableDiagonalPreconditioner(True)
        solver.EnableWarmStart(True)  
        solver.SetVerbose(False)
        
        ts = chrono.ChTimestepperEulerImplicitLinearized(self.system)
        self.system.SetTimestepper(ts)

        
        while vis.Run():
            vis.BeginScene()  
            vis.Render()  
            vis.EndScene()  
            self.system.DoStepDynamics(0.01)  

    def PrintBodyPositions(self):
        for i in range(self.n_chains):
            print(f"Chain {i + 1} end body position: {self.system.GetBody(i).GetPos()}")


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
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))  
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


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)  


model.PrintBodyPositions()