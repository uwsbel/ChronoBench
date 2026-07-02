import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.end_bodies = []
        for i in range(n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  
            
            builder = fea.ChBuilderCableANCF()
            
            y_offset = i * 0.1
            A = chrono.ChVector3d(0, y_offset, -0.1)  
            B = chrono.ChVector3d(0.5, y_offset, -0.1)  
            
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                10 + i,  
                A,
                B
            )

            
            front_node = builder.GetLastBeamNodes().front()
            back_node = builder.GetLastBeamNodes().back()
            front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            system.Add(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(back_node, mtruss)
            system.Add(constraint_hinge)  

            
            box_body = chrono.ChBody()
            box_body.SetPos(A)  
            system.Add(box_body)

            
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(front_node, box_body)
            system.Add(constraint_box)

            
            box_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)  
            box_body.AddVisualShape(box_shape)

            
            self.end_bodies.append(box_body)

    def PrintBodyPositions(self):
        
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i} end body position: {body.GetPos()}")



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