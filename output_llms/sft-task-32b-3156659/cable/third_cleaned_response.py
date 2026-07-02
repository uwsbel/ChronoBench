import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.end_bodies = []  
        self.n_chains = n_chains

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()

        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            mtruss.SetName(f"Truss_{i}")
            system.AddBody(mtruss)

            
            y_offset = i * 0.1
            start_point = chrono.ChVector3d(0, y_offset, -0.1)
            end_point = chrono.ChVector3d(0.5, y_offset, -0.1)

            
            num_elements = 10 + i  
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                num_elements,  
                start_point,  
                end_point  
            )

            
            
            front_node = builder.GetLastBeamNodes().front()
            front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            back_node = builder.GetLastBeamNodes().back()
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(back_node, mtruss)
            system.AddLink(constraint_hinge)

            
            box_body = chrono.ChBody()
            box_body.SetBodyFixed(False)
            box_body.SetCollide(True)
            box_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)  
            box_body.AddVisualShape(box_shape)
            box_body.SetName(f"Box_{i}")
            
            box_body.SetPos(end_point + chrono.ChVector3d(0, 0, 0.1))
            system.AddBody(box_body)
            self.end_bodies.append(box_body)

            
            constraint_box = fea.ChLinkNodeBody()
            constraint_box.Initialize(back_node, box_body)
            system.AddLink(constraint_box)

    def PrintBodyPositions(self):
        
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i} end body position: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")



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
vis.SetWindowTitle('FEA cables with multiple chains')  
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