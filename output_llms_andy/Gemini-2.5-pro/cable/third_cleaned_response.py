import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr









class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0)  

        beam_length = 0.5
        chain_spacing_x = 0.3  
        start_y_offset = 0.5   

        for i in range(self.n_chains):
            
            builder = fea.ChBuilderCableANCF()

            
            num_elements = 5 + i * 2

            
            start_point = chrono.ChVector3d(i * chain_spacing_x, start_y_offset, -0.1)
            end_point = chrono.ChVector3d(i * chain_spacing_x + beam_length, start_y_offset, -0.1)

            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            

            
            start_node_beam = builder.GetBeamNodes().front()

            
            fixed_anchor_body = chrono.ChBody()
            fixed_anchor_body.SetFixed(True)
            fixed_anchor_body.SetPos(start_node_beam.GetPos()) 
            self.system.Add(fixed_anchor_body)

            
            constraint_hinge = fea.ChLinkPointFrame()
            constraint_hinge.Initialize(start_node_beam, fixed_anchor_body)
            self.system.Add(constraint_hinge)

            
            end_node_beam = builder.GetBeamNodes().back()

            box_size = 0.05  
            box_density = 2000  
            
            box_initial_pos = end_node_beam.GetPos()

            box_body = chrono.ChBodyEasyBox(
                box_size, box_size, box_size,
                box_density,
                True,  
                True   
            )
            box_body.SetPos(box_initial_pos)
            self.system.Add(box_body)
            self.end_bodies.append(box_body)

            
            link_node_to_box = fea.ChLinkPointFrame()
            link_node_to_box.Initialize(end_node_beam, box_body)
            self.system.Add(link_node_to_box)

            
            force_magnitude = 0.5 + i * 0.1 
            applied_force_vector = chrono.ChVector3d(0, -force_magnitude, 0)
            
            
            constant_force_load = chrono.ChLoadBodyForce(
                box_body,                    
                applied_force_vector,        
                False,                       
                chrono.ChFrameMovingD()      
            )
            self.system.AddLoad(constant_force_load)

    def PrintBodyPositions(self):
        
        print(f"Time: {self.system.GetChTime():.4f}")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"  Chain {i} Box Position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")




sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0)) 

mesh = fea.ChMesh()






model = Model1(sys, mesh) 

sys.Add(mesh) 






vis_beam_elements = chrono.ChVisualShapeFEA(mesh)
vis_beam_elements.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_elements.SetColorscaleMinMax(-0.4, 0.4) 
vis_beam_elements.SetSmoothFaces(True)
vis_beam_elements.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_elements)


vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)
vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_nodes.SetSymbolsThickness(0.006)
vis_beam_nodes.SetSymbolsScale(0.01) 
vis_beam_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_nodes)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

vis.AddCamera(chrono.ChVector3d(0.8, 0.6, -2.0)) 
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


time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    model.PrintBodyPositions() 
    vis.EndScene()
    sys.DoStepDynamics(time_step)