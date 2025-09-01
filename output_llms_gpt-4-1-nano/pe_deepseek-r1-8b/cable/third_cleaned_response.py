import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chain_length = [5, 7, 9, 11, 13, 15]  
        
        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)  
        
        
        builder = fea.ChBuilderCableANCF()
        
        
        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            
            
            pos_hinge = chrono.ChVector3d(0, 0, 0.1 * (i + 1))  
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            
            beam = builder.BuildBeam(
                mesh,
                msection_cable,
                self.chain_length[i],
                chrono.ChVector3d(0, 0, -0.1 - 0.1 * i),  
                chrono.ChVector3d(0.5, 0, -0.1 - 0.1 * i)  
            )
            
            
            beam.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            self.AddVisualShapeFEA(beam, i)
            
            
            system.Add(mtruss)
            
            
            self.PrintBodyPositions()
            
        
        self.visualize_beam = chrono.ChVisualShapeFEA(mesh)
        self.visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        self.visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
        self.visualize_beam.SetSmoothFaces(True)
        self.visualize_beam.SetWireframe(False)
        mesh.AddVisualShapeFEA(self.visualize_beam)
        
        
        self.visualize_nodes = chrono.ChVisualShapeFEA(mesh)
        self.visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        self.visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        self.visualize_nodes.SetSymbolsThickness(0.006)
        self.visualize_nodes.SetSymbolsScale(0.01)
        self.visualize_nodes.SetZbufferHide(False)
        mesh.AddVisualShapeFEA(self.visualize_nodes)
    
    def AddVisualShapeFEA(self, beam, chain_id):
        
        vis_shape = chrono.ChVisualShapeFEA(beam)
        vis_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        vis_shape.SetColorscaleMinMax(-0.4, 0.4)
        vis_shape.SetSmoothFaces(True)
        vis_shape.SetWireframe(False)
        beam.AddVisualShapeFEA(vis_shape)
        
        
        vis_nodes = chrono.ChVisualShapeFEA(beam)
        vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        vis_nodes.SetSymbolsThickness(0.006)
        vis_nodes.SetSymbolsScale(0.01)
        vis_nodes.SetZbufferHide(False)
        beam.AddVisualShapeFEA(vis_nodes)
    
    def PrintBodyPositions(self):
        print("Printing body positions for each chain:")
        for i in range(self.n_chains):
            end_body = self.chain[i].GetLastBeamNodes().back().GetAttachedBody()
            pos = end_body.GetPos()
            print(f"Chain {i+1}: Position = {pos.x}, {pos.y}, {pos.z}")
    
    def __init__(self, system, mesh):
        pass


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