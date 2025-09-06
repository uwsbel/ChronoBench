import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.end_bodies = []  
        
        
        spacing = 0.6  
        for i in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetPos(chrono.ChVector3d(i * spacing, 0, 0))
            mtruss.SetFixed(True)
            system.Add(mtruss)
            
            
            box_truss = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            box_truss.SetColor(chrono.ChColor(1, 0, 0))  
            mtruss.AddVisualShape(box_truss)
            
            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)
            
            
            builder = fea.ChBuilderCableANCF()
            
            
            start_point = chrono.ChVector3d(i * spacing, 0, 0)
            end_point = chrono.ChVector3d(i * spacing + 0.5 + i*0.1, 0, -0.2 - i*0.05)
            num_elements = 10 + i  
            
            
            builder.BuildBeam(
                mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )
            
            
            node_first = builder.GetLastBeamNodes().front()
            node_last = builder.GetLastBeamNodes().back()
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(node_first, mtruss)
            system.Add(constraint_hinge)
            
            
            end_body = chrono.ChBody()
            end_body.SetPos(end_point)
            end_body.SetMass(0.5)
            end_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
            system.Add(end_body)
            self.end_bodies.append(end_body)  
            
            
            box_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
            box_shape.SetColor(chrono.ChColor(0, 1, 0))  
            end_body.AddVisualShape(box_shape)
            
            
            constraint_end = fea.ChLinkNodeFrame()
            constraint_end.Initialize(node_last, end_body)
            system.Add(constraint_end)
    
    def PrintBodyPositions(self):
        print(f"\nTime: {self.system.GetChTime():.4f}")
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
vis.SetWindowTitle('Multiple Chain FEA Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 1, -2))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    
    if step % 10 == 0:
        model.PrintBodyPositions()
    step += 1