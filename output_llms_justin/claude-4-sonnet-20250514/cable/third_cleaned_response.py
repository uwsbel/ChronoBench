import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  
        
        
        for i in range(n_chains):
            self.create_chain(i)
    
    def create_chain(self, chain_index):
        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)  
        
        
        builder = fea.ChBuilderCableANCF()
        
        
        x_offset = chain_index * 0.15  
        start_point = chrono.ChVector3d(x_offset, 0, 0)
        end_point = chrono.ChVector3d(x_offset, -0.5, 0)
        
        
        num_elements = 5 + chain_index * 2
        
        
        builder.BuildBeam(
            self.mesh,  
            msection_cable,  
            num_elements,  
            start_point,  
            end_point   
        )
        
        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        mtruss.SetPos(start_point)
        self.system.Add(mtruss)
        
        
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
        self.system.Add(constraint_hinge)
        
        
        end_body = chrono.ChBody()
        end_body.SetMass(0.1)
        end_body.SetPos(end_point + chrono.ChVector3d(0, -0.05, 0))
        
        
        end_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
        
        
        box_shape = chrono.ChVisualShapeBox(0.02, 0.02, 0.02)
        box_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
        end_body.AddVisualShape(box_shape)
        
        
        end_body.EnableCollision(True)
        end_body.GetCollisionModel().ClearModel()
        end_body.GetCollisionModel().AddBox(0.01, 0.01, 0.01)
        end_body.GetCollisionModel().BuildModel()
        
        self.system.Add(end_body)
        self.end_bodies.append(end_body)
        
        
        constraint_end = fea.ChLinkNodeFrame()
        constraint_end.Initialize(builder.GetLastBeamNodes().back(), end_body)
        self.system.Add(constraint_end)
        
        
        end_body.SetForce(chrono.ChVector3d(0, -0.1, 0))
    
    def PrintBodyPositions(self):
        
        print(f"Time: {self.system.GetChTime():.3f}s")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"  Chain {i}: Position = ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")
        print()


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
vis.SetWindowTitle('FEA cables - Multiple Chains')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0.5, 0.6, -1))  
vis.AddTypicalLights()  


solver = chrono.ChSolverMINRES()  
print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)  
solver.SetVerbose(False)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


step_count = 0
print_interval = 50  

while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)  
    
    
    if step_count % print_interval == 0:
        model.PrintBodyPositions()
    
    step_count += 1