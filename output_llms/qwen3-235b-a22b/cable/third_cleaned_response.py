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
            
            nelements = 5 + i  
            x_offset = i * 0.6  

            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)

            
            builder = fea.ChBuilderCableANCF()
            start = chrono.ChVector3d(x_offset, 0, -0.1)
            end = chrono.ChVector3d(x_offset + 0.5, 0, -0.1)
            builder.BuildBeam(mesh, msection_cable, nelements, start, end)

            
            nodes = builder.GetLastBeamNodes()
            front_node = nodes.front()
            back_node = nodes.back()

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            
            hinge_front = fea.ChLinkNodeFrame()
            hinge_front.Initialize(front_node, mtruss)
            system.Add(hinge_front)

            
            box_body = chrono.ChBody()
            box_body.SetMass(1.0)
            box_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
            box_body.SetPos(back_node.GetPos())
            box_body.SetFixed(False)

            
            box_vis = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
            box_vis.SetColor(chrono.ChColor(0, 0, 1))
            box_body.AddVisualShape(box_vis)
            system.Add(box_body)

            
            hinge_back = fea.ChLinkNodeFrame()
            hinge_back.Initialize(back_node, box_body)
            system.Add(hinge_back)

            
            box_body.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            self.end_bodies.append(box_body)

    def PrintBodyPositions(self):
        
        for idx, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {idx+1} end body position: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)


vis_beam_moments = chrono.ChVisualShapeFEA(mesh)
vis_beam_moments.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_moments.SetColorscaleMinMax(-0.4, 0.4)
vis_beam_moments.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam_moments)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(vis_nodes)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Cables with Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.2, -2))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()