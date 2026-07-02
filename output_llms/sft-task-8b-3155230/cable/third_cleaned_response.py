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
        
        for chain_idx in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetName(f"Truss_{chain_idx}")
            
            box = chrono.ChBody()
            box.SetFixed(False)
            box.GetCollisionModel().Clear()
            box.GetCollisionModel().AddBoxGeometry(0.05, 0.05, 0.05)
            box.EnableCollision(False)
            box.SetName(f"Box_{chain_idx}")
            
            system.Add(box)
            self.boxes.append(box)
            
            builder = fea.ChBuilderCableANCF()
            
            y_offset = chain_idx * 0.2
            start_point = chrono.ChVector3d(0, y_offset, -0.1)
            end_point = chrono.ChVector3d(0.5, y_offset, -0.1)
            
            num_elements = 10 + chain_idx * 5
            
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_point,
                end_point
            )
            
            end_node = builder.GetLastBeamNodes().back()
            end_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)
            
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(end_node, box)
            system.Add(constraint_box)
            
            visualizebox = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            visualizebox.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
            box.AddVisualShape(visualizebox)
        
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

    def PrintBodyPositions(self):
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i} box position: ({pos.x}, {pos.y}, {pos.z})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
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