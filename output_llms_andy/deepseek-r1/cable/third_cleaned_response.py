import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_boxes = []  
        
        for i in range(n_chains):
            
            start_pos = chrono.ChVector3d(i * 0.6, 0.0, -0.1)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(start_pos)
            system.Add(mtruss)
            
            
            truss_shape = chrono.ChBoxShape()
            truss_shape.GetBoxGeometry().Size = chrono.ChVector3d(0.02, 0.02, 0.02)
            truss_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
            mtruss.AddVisualShape(truss_shape)
            
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)
            msection_cable2.SetYoungModulus(0.01e9)
            msection_cable2.SetRayleighDamping(0.0001)
            
            
            builder = fea.ChBuilderCableANCF()
            end_pos = chrono.ChVector3d(i * 0.6 + 0.5, 0.0, -0.1)
            num_elements = 10 + i  
            
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_pos,
                end_pos
            )
            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)  
            box.SetPos(end_pos)
            system.Add(box)
            self.end_boxes.append(box)  
            
            
            box_shape = chrono.ChBoxShape()
            box_shape.GetBoxGeometry().Size = chrono.ChVector3d(0.05, 0.05, 0.05)
            box_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
            box.AddVisualShape(box_shape)
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_hinge)
    
    def PrintBodyPositions(self):
        
        for i, box in enumerate(self.end_boxes):
            pos = box.GetPos()
            print(f"Chain {i}: End body position = ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")


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
vis.SetWindowTitle('Multiple FEA Chains with End Bodies')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.5, -2))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    
    if step % 10 == 0:
        model.PrintBodyPositions()
    step += 1