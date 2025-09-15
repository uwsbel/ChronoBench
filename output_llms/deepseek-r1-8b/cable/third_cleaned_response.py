import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.solver = None
        self.timestepper = None
        self.vis = None
        
        
        self.msections = []
        for _ in range(n_chains):
            msection = fea.ChBeamSectionCable()
            msection.SetDiameter(0.015)  
            msection.SetYoungModulus(0.01e9)
            msection.SetRayleighDamping(0.0001)
            self.msections.append(msection)
        
        
        self.visualize_beams = []
        self.visualize_nodes = []
        for _ in range(n_chains):
            visualize_beam = chrono.ChVisualShapeFEA(mesh)
            visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
            visualize_beam.SetSmoothFaces(True)
            visualize_beam.SetWireframe(False)
            mesh.AddVisualShapeFEA(visualize_beam)
            self.visualize_beams.append(visualize_beam)
            
            visualize_node = chrono.ChVisualShapeFEA(mesh)
            visualize_node.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualize_node.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualize_node.SetSymbolsThickness(0.006)
            visualize_node.SetSymbolsScale(0.01)
            visualize_node.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(visualize_node)
            self.visualize_nodes.append(visualize_node)
            
        
        self.solver = chrono.ChSolverMINRES()
        if self.solver.GetType() == chrono.ChSolver.Type_MINRES:
            print("Using MINRES solver")
            system.SetSolver(self.solver)
            self.solver.SetMaxIterations(200)
            self.solver.SetTolerance(1e-10)
            self.solver.EnableDiagonalPreconditioner(True)
            self.solver.EnableWarmStart(True)
            self.solver.SetVerbose(False)
        
        self.timestepper = chrono.ChTimestepperEulerImplicitLinearized(system)
        system.SetTimestepper(self.timestepper)
        
        
        self.print_body_positions = True

    def __init__(self, system, mesh):
        pass  

    def PrintBodyPositions(self, step):
        if not self.print_body_positions:
            return
        for chain in range(self.n_chains):
            end_body = self.chains[chain].GetLastBeamNodes().back().GetNode()
            pos = end_body.GetPosition()
            print(f"Chain {chain+1} end body position: {pos.x}, {pos.y}, {pos.z}")
            
    def __post__init__(self):
        
        self.chains = []
        for _ in range(self.n_chains):
            builder = fea.ChBuilderCableANCF()
            
            num_elements = 3 + 3*(len(self.chains))
            builder.BuildBeam(
                mesh,
                self.msections[chain],
                num_elements,
                chrono.ChVector3d(0, 0, -0.1),
                chrono.ChVector3d(0.5, 0, -0.1)
            )
            chain_model = ChainModel(builder.GetLastBeamNodes(), mesh, self.msections[chain])
            self.chains.append(chain_model)
            
            chain_model.ApplyBoundaryConditions()
            
            system.Add(chain_model)
            
            mesh.AddVisualShapeFEA(self.visualize_beams[chain])
            mesh.AddVisualShapeFEA(self.visualize_nodes[chain])
            
        
        for chain in range(self.n_chains):
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(self.chains[chain].GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            system.Add(mtruss)
            
        
        self.chains[0].GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

class ChainModel:
    def __init__(self, nodes, mesh, section):
        self.nodes = nodes
        self.mesh = mesh
        self.section = section
        self.force = chrono.ChVector3d(0, -0.7, 0)
        
    def ApplyBoundaryConditions(self):
        self.nodes.front().SetForce(self.force)
        self.nodes.back().SetConstraint(constraint_hinge)


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
    
    if model.print_body_positions:
        model.PrintBodyPositions(sys.GetTime())