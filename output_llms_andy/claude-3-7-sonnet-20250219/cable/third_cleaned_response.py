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
            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)  
            msection_cable.SetYoungModulus(0.01e9)  
            msection_cable.SetRayleighDamping(0.0001)
            
            
            n_elements = 10 + i * 2
            
            
            x_offset = i * 0.2
            z_offset = -0.1 - i * 0.1
            
            
            builder = fea.ChBuilderCableANCF()
            
            
            builder.BuildBeam(
                mesh,
                msection_cable,
                n_elements,
                chrono.ChVector3d(x_offset, 0, z_offset),  
                chrono.ChVector3d(x_offset + 0.5, 0, z_offset)  
            )
            
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            
            mbox = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)  
            mbox.SetPos(chrono.ChVector3d(x_offset, 0, z_offset))
            system.Add(mbox)
            self.end_bodies.append(mbox)  
            
            
            constraint_box = fea.ChLinkNodeBody()
            constraint_box.Initialize(builder.GetLastBeamNodes().front(), mbox)
            system.Add(constraint_box)
            
            
            mbox.SetForce(chrono.ChVector3d(0, -0.7 * (i + 1), 0))  

    def PrintBodyPositions(self):
        
        print("Chain end body positions:")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")


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


step_count = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)  
    
    
    step_count += 1
    if step_count % 20 == 0:
        model.PrintBodyPositions()