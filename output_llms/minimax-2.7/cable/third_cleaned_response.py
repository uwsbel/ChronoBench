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
            
            msection = fea.ChBeamSectionCable()
            msection.SetDiameter(0.015)                
            msection.SetYoungModulus(0.01e9)            
            msection.SetRayleighDamping(0.0001)

            
            num_elements = 10 + i                       

            
            offset_x = i * 0.8
            pointA = chrono.ChVector3d(offset_x, 0.0, -0.1)
            pointB = chrono.ChVector3d(offset_x + 0.5, 0.0, -0.1)

            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, msection, num_elements, pointA, pointB)

            
            beam_nodes = builder.GetLastBeamNodes()
            
            
            node_front = beam_nodes[0]
            node_back = beam_nodes[-1]

            
            truss = chrono.ChBody()
            truss.SetFixed(True)
            system.Add(truss)

            
            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(node_front, truss)
            system.Add(hinge)

            
            node_front.SetForce(chrono.ChVector3d(0.0, -0.7, 0.0))

            
            box = chrono.ChBody()
            box.SetMass(1.0)
            box.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            box.SetPos(node_back.GetPos())               
            system.Add(box)

            
            link_node_box = fea.ChLinkNodeFrame()
            link_node_box.Initialize(node_back, box)
            system.Add(link_node_box)

            
            box_vis = chrono.ChBoxShape(0.05, 0.05, 0.05)
            box.AddVisualShape(box_vis)

            
            self.end_bodies.append(box)

    
    
    
    def PrintBodyPositions(self):
        for idx, body in enumerate(self.end_bodies):
            p = body.GetPos()
            print(f"Chain {idx:2d} – end‑body pos: ({p.x:8.4f}, {p.y:8.4f}, {p.z:8.4f})")






sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)


sys.Add(mesh)




vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.4, 0.4)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_node = chrono.ChVisualShapeFEA(mesh)
vis_node.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_node.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_node.SetSymbolsThickness(0.006)
vis_node.SetSymbolsScale(0.01)
vis_node.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_node)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple ANCF cable chains')
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




timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(timestepper)




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(0.01)

    
    model.PrintBodyPositions()