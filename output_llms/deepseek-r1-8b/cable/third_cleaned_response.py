import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr








class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.system = system
        self.mesh = mesh
        self.visualizations = []
        
        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)
        
        
        self.builder = fea.ChBuilderCableANCF()
        
        
        self.meshes = []
        for i in range(self.n_chains):
            chain_mesh = fea.ChMesh()
            self.meshes.append(chain_mesh)
            self.system.Add(chain_mesh)
            self.meshes[i].AddVisualShapeFEA(self.create_beam_visualization(i))
        
        
        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)
            
            
            num_elements = 10 + i  
            start_point = chrono.ChVector3d(0.0, 0.0, -0.1 - 0.1 * i)
            end_point = chrono.ChVector3d(0.5, 0.0, -0.1 - 0.1 * i)
            
            
            self.builder.BuildBeam(
                self.meshes[i],
                msection_cable,
                num_elements,
                start_point,
                end_point
            )
            
            
            last_node = self.builder.GetLastBeamNodes().back()
            last_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(last_node, mtruss)
            self.system.Add(constraint)
            
            
            self.meshes[i].AddVisualShapeFEA(self.create_beam_visualization(i))
            self.meshes[i].AddVisualShapeFEA(self.create_node_visualization(i))
            
            
            self.print_body_positions(i)
            
    def create_beam_visualization(self, chain_id):
        visualize = chrono.ChVisualShapeFEA(self.meshes[chain_id])
        visualize.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        visualize.SetColorscaleMinMax(-0.4, 0.4)
        visualize.SetSmoothFaces(True)
        visualize.SetWireframe(False)
        return visualize
        
    def create_node_visualization(self, chain_id):
        visualize = chrono.ChVisualShapeFEA(self.meshes[chain_id])
        visualize.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        visualize.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        visualize.SetSymbolsThickness(0.006)
        visualize.SetSymbolsScale(0.01)
        visualize.SetZbufferHide(False)
        return visualize
        
    def print_body_positions(self, chain_id):
        end_body = self.builder.GetLastBeamNodes().back()
        pos = end_body.GetPosition()
        print(f"Chain {chain_id + 1} end body position: {pos.x}, {pos.y}, {pos.z}")
        
    def __init__(self, system, mesh):
        pass  


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  



model.mesh.AddVisualShapeFEA(chrono.ChVisualShapeFEA(mesh))
model.mesh.AddVisualShapeFEA(chrono.ChVisualShapeFEA(mesh))


model.mesh.AddVisualShapeFEA(model.create_node_visualization(0))


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

    
    for i in range(model.n_chains):
        model.print_body_positions(i)