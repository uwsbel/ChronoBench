import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh):
        self.n_chains = 6  
        self.chain_elements = [10, 15, 20, 15, 10, 10]  
        
        
        self.visualize_beams = []
        self.visualize_nodes = []
        self.bodies = []
        
        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)  
        
        
        builder = fea.ChBuilderCableANCF()
        
        
        self.truss_body = chrono.ChBody()
        self.truss_body.SetFixed(True)
        self.bodies.append(self.truss_body)
        
        
        for i in range(self.n_chains):
            
            chain_truss = chrono.ChBody()
            chain_truss.SetFixed(True)
            self.bodies.append(chain_truss)
            
            
            beam = builder.BuildBeam(
                mesh,
                msection_cable,
                self.chain_elements[i],
                chrono.ChVector3d(0, 0, -0.1 - (i * 0.5)),  
                chrono.ChVector3d(0.5, 0, -0.1 - (i * 0.5))  
            )
            
            
            
            last_nodes = beam.GetLastBeamNodes()
            
            last_nodes.front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(last_nodes.back(), chain_truss)
            system.Add(constraint)
            
            
            visualize_beam = chrono.ChVisualShapeFEA(mesh)
            visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
            visualize_beam.SetSmoothFaces(True)
            visualize_beam.SetWireframe(False)
            mesh.AddVisualShapeFEA(visualize_beam)
            
            
            visualize_node = chrono.ChVisualShapeFEA(mesh)
            visualize_node.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualize_node.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualize_node.SetSymbolsThickness(0.006)
            visualize_node.SetSymbolsScale(0.01)
            visualize_node.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(visualize_node)
            
            
            system.Add(beam)
            system.Add(last_nodes)
            
        
        system.Add(self.truss_body)
    
    def PrintBodyPositions(self):
        
        while vis.Run():
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            
            for body in self.bodies:
                pos = body.GetPosition()
                print(f"Chain {self.bodies.index(body)+1} end position: {pos}")
            vis.DoStepDynamics(0.01)





sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

model = Model1(sys, mesh)
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