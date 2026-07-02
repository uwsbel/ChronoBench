import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.boxes = []

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()

        
        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            
            start_pos = chrono.ChVector3d(i * 0.6, 0, -0.1)
            end_pos = chrono.ChVector3d(i * 0.6 + 0.5, 0, -0.1)

            mbox = chrono.ChBody()
            mbox.SetBodyFixed(False)
            mbox.SetCollide(False)
            mbox.SetMass(1.0)
            mbox.SetInertiaXX(0.1, 0.1, 0.1)
            mbox.SetPos(end_pos)
            system.Add(mbox)
            self.boxes.append(mbox)

            
            num_elements = 10 + i
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                num_elements,  
                start_pos,  
                end_pos  
            )

            
            nodes = builder.GetLastBeamNodes()
            start_node = nodes.front()
            end_node = nodes.back()

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(start_node, mtruss)
            system.Add(constraint_hinge)

            
            start_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            constraint_end = fea.ChLinkNodeFrame()
            constraint_end.Initialize(end_node, mbox)
            system.Add(constraint_end)

        
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

        
        for box in self.boxes:
            visualizebox = chrono.ChVisualShape(chrono.VISUALIZATION_TYPE_BOX)
            visualizebox.SetBoxSize(0.05, 0.05, 0.05)
            visualizebox.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
            box.AddVisualShape(visualizebox)

    def PrintBodyPositions(self):
        for box in self.boxes:
            pos = box.GetPos()
            print(f"Box position: {pos}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
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


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)  
    model.PrintBodyPositions()