import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        
        self.n_chains = n_chains
        self.chains = []
        self.end_body_positions = []

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  
        
        builder = fea.ChBuilderCableANCF()

        
        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                i + 1,  
                chrono.ChVector3d(0, 0, -0.1 + 0.01 * i),  
                chrono.ChVector3d(0.5, 0, -0.1 + 0.01 * i)  
            )

            
            end_body = chrono.ChBody()
            end_body.SetPos(chrono.ChVector3d(0, 0, -0.1 + 0.01 * i + 0.05 * i)
            end_body.SetSize(0.1, 0.1, 0.1)
            system.Add(end_body)
            constraint_box = fea.ChLinkFrame()
            constraint_box.Initialize(builder.GetLastBeamNodes().back(), end_body)
            system.Add(constraint_box)

            
            visualizebeamA = chrono.ChVisualShapeFEA(mesh)
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
            visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
            visualizebeamA.SetSmoothFaces(True)  
            visualizebeamA.SetWireframe(False)  
            mesh.AddVisualShapeFEA(visualizebeamA)  

            
            visualizebeamB = chrono.ChVisualShapeFEA(mesh)
            visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
            visualizebeamB.SetSymbolsThickness(0.006)  
            visualizebeamB.SetSymbolsScale(0.01)  
            visualizebeamB.SetZbufferHide(False)  
            mesh.AddVisualShapeFEA(visualizebeamB)  

            
            visualizetruss = chrono.ChVisualShapeFEA(end_body)
            visualizetruss.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BOX)
            visualizetruss.SetColorscaleMinMax(0, 1)  
            visualizetruss.SetSmoothFaces(True)
            visualizetruss.SetWireframe(False)
            end_body.AddVisualShapeFEA(visualizetruss)

            
            visualizeend_body = chrono.ChVisualShapeFEA(end_body)
            visualizeend_body.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DOT)
            visualizeend_body.SetSymbolsThickness(0.01)
            visualizeend_body.SetSymbolsScale(0.02)
            visualizeend_body.SetZbufferHide(False)
            end_body.AddVisualShapeFEA(visualizeend_body)

            
            visualizebox = chrono.ChVisualShapeFEA(end_body)
            visualizebox.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BOX)
            visualizebox.SetColorscaleMinMax(0, 1)
            visualizebox.SetSmoothFaces(True)
            visualizebox.SetWireframe(False)
            end_body.AddVisualShapeFEA(visualizebox)

            
            self.chains.append((visualizebeamA, visualizebeamB, visualizetruss, visualizeend_body, visualizebox))
            self.end_body_positions.append(end_body.GetPosition())

    def PrintBodyPositions(self):
        for i, pos in enumerate(self.end_body_positions):
            print(f"Chain {i+1} end body position: {pos}")


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


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    model.PrintBodyPositions()  
    sys.DoStepDynamics(0.01)
    vis.EndScene()


model.PrintBodyPositions()