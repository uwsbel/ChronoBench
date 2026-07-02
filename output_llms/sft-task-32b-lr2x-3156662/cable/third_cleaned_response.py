import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.end_bodies = []  
        self.chain_visualizations = []  

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)

        for chain_idx in range(n_chains):
            builder = fea.ChBuilderCableANCF()
            n_elements = 10 + chain_idx  

            
            y_offset = chain_idx * 0.2
            start_point = chrono.ChVector3d(0, y_offset, -0.1)
            end_point = chrono.ChVector3d(0.5, y_offset, -0.1)

            
            builder.BuildBeam(
                self.mesh,
                msection_cable2,
                n_elements,
                start_point,
                end_point
            )

            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetName(f"Truss Chain {chain_idx}")
            system.AddBody(mtruss)

            
            endpoint_box = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000, True, True)
            endpoint_box.SetName(f"Endpoint Box Chain {chain_idx}")
            endpoint_box.SetPos(end_point)
            endpoint_box.SetFixed(False)
            system.AddBody(endpoint_box)
            self.end_bodies.append(endpoint_box)

            
            endpoint_node = builder.GetLastBeamNodes().back()
            endpoint_pos = endpoint_node.GetPos()
            endpoint_box.SetPos(endpoint_pos)

            
            revolute = chrono.ChLinkLockRevolute()
            revolute.Initialize(endpoint_node, endpoint_box, chrono.ChFramed(endpoint_pos))
            system.AddLink(revolute)

            
            vis_beam = chrono.ChVisualShapeFEA()
            vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            vis_beam.SetColorscaleMinMax(-0.4, 0.4)
            vis_beam.SetSmoothFaces(True)
            vis_beam.SetWireframe(False)
            self.mesh.AddVisualShapeFEA(vis_beam)
            self.chain_visualizations.append(vis_beam)

            
            vis_nodes = chrono.ChVisualShapeFEA()
            vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            vis_nodes.SetSymbolsThickness(0.006)
            vis_nodes.SetSymbolsScale(0.01)
            vis_nodes.SetZbufferHide(False)
            self.mesh.AddVisualShapeFEA(vis_nodes)
            self.chain_visualizations.append(vis_nodes)

    def PrintBodyPositions(self):
        
        print("Chain Endpoint Positions:")
        for idx, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {idx}: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")
        print()


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Multiple FEA Cable Chains with Endpoint Bodies')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.0, -2.5))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
sys.SetSolver(solver)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    model.PrintBodyPositions()  
    sys.DoStepDynamics(step)