import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.core.solver as solver







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.beams = []
        self.truss_bodies = []
        self.hinge_constraints = []
        self.visual_shapes = []
        self.print_body_positions = True

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        self.vis_beam = chrono.ChVisualShapeFEA(mesh)
        self.vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        self.vis_beam.SetColorscaleMinMax(-0.4, 0.4)
        self.vis_beam.SetSmoothFaces(True)
        self.vis_beam.SetWireframe(False)
        
        self.vis_nodes = chrono.ChVisualShapeFEA(mesh)
        self.vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        self.vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        self.vis_nodes.SetSymbolsThickness(0.006)
        self.vis_nodes.SetSymbolsScale(0.01)
        self.vis_nodes.SetZbufferHide(False)

        
        self.solver = chrono.ChSolverSparseQR(system)
        self.solver.SetMaxIterations(200)
        self.solver.SetTolerance(1e-10)
        self.solver.EnableDiagonalPreconditioner(True)
        self.solver.EnableWarmStart(True)
        self.solver.SetVerbose(False)

        
        self.ts = chrono.ChTimestepperEulerImplicitLinearized(system)
        system.SetTimestepper(self.ts)

    def __init__(self, system, mesh):
        Model1.__init__(self, system, mesh, 6)
        self.print_body_positions = True

    def PrintBodyPositions(self):
        if self.print_body_positions:
            print("Printing end body positions for each chain...")
            for i, (beam, truss, hinge) in enumerate(zip(self.beams, self.truss_bodies, self.hinge_constraints)):
                pos = truss.GetPosition()
                print(f"Chain {i+1}: End body position = {pos}")
            self.print_body_positions = False

    def __init__(self, system, mesh):
        
        pass

    def create_chain(self, chain_num):
        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        self.truss_bodies.append(mtruss)
        
        
        builder = fea.ChBuilderCableANCF()
        beam = builder.BuildBeam(
            mesh,
            msection_cable2,
            2*chain_num,  
            chrono.ChVector3d(0, 0, -0.1*chain_num),  
            chrono.ChVector3d(0.5, 0, -0.1*chain_num)  
        )
        self.beams.append(beam)
        
        
        front_node = builder.GetLastBeamNodes().front()
        front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
        
        
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(front_node, mtruss)
        self.hinge_constraints.append(constraint_hinge)
        system.Add(constraint_hinge)
        
        
        self.visual_shapes.append(self.vis_beam.Clone())
        self.visual_shapes[chain_num].SetPosition(mtruss.GetPosition())
        mesh.AddVisualShapeFEA(self.visual_shapes[chain_num])
        
        
        self.visual_shapes.append(self.vis_nodes.Clone())
        self.visual_shapes[chain_num + 1].SetPosition(front_node.GetPosition())
        mesh.AddVisualShapeFEA(self.visual_shapes[chain_num + 1])


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)


visualizebeamA = model.vis_beam
visualizebeamB = model.vis_nodes
for vs in model.visual_shapes:
    mesh.AddVisualShapeFEA(vs)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()


solver = sys.GetSolver()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    solver = Model1.solver
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    if model.PrintBodyPositions:
        model.PrintBodyPositions()