import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math






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
            
            
            builder = fea.ChBuilderCableANCF()
            
            
            angle = (i * 2 * math.pi) / n_chains
            radius = 0.3  
            start_x = radius * math.cos(angle)
            start_y = radius * math.sin(angle)
            
            
            num_elements = 10 + i  
            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                chrono.ChVector3d(start_x, start_y, -0.1),
                chrono.ChVector3d(start_x + 0.5, start_y, -0.1)
            )
            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            
            box = chrono.ChBody()
            box.SetBodyFixed(False)
            box.SetMass(1.0)
            box.SetPos(builder.GetLastBeamNodes().back().GetPos())
            box_shape = chrono.ChBoxShape()
            box_shape.SetDimensions(chrono.ChVector3d(0.05, 0.05, 0.05))
            box.AddVisualShape(box_shape)
            system.Add(box)
            
            
            constraint_lock = fea.ChLinkLockRevolute()
            constraint_lock.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_lock)
            
            
            visualize_beam = chrono.ChVisualShapeFEA(mesh)
            visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
            visualize_beam.SetSmoothFaces(True)
            visualize_beam.SetWireframe(False)
            mesh.AddVisualShapeFEA(visualize_beam)
            
            
            visualize_nodes = chrono.ChVisualShapeFEA(mesh)
            visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualize_nodes.SetSymbolsThickness(0.006)
            visualize_nodes.SetSymbolsScale(0.01)
            visualize_nodes.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(visualize_nodes)
            
            
            self.end_bodies.append(box)
            
    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i+1} end position: {body.GetPos()}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Cables with Multiple Chains')
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