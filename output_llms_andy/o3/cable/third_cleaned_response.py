import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr









class Model1:
    def __init__(self, system, mesh, n_chains: int = 6):
        self.system = system
        self.mesh   = mesh
        self.n_chains = n_chains
        self.end_bodies = []            

        
        
        
        cable_section = fea.ChBeamSectionCable()
        cable_section.SetDiameter(0.015)        
        cable_section.SetYoungModulus(0.01e9)   
        cable_section.SetRayleighDamping(0.0001)

        
        
        
        for i in range(self.n_chains):

            
            truss = chrono.ChBody()
            truss.SetFixed(True)
            truss.SetPos(chrono.ChVector3d(0, 0.20 * i, 0))      
            self.system.Add(truss)

            
            nel   = 6 + 2 * i                                    
            Apos  = chrono.ChVector3d(0,           0.20 * i, 0)
            Bpos  = chrono.ChVector3d(0.50 + 0.05 * i,
                                      0.20 * i,
                                      0)

            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(self.mesh,                 
                              cable_section,             
                              nel,                       
                              Apos,                      
                              Bpos)                      

            
            last_nodes = builder.GetLastBeamNodes()
            first_node = last_nodes[0]
            last_node  = last_nodes[1]

            
            first_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(first_node, truss)
            self.system.Add(hinge)

            
            box_size = 0.04
            box = chrono.ChBodyEasyBox(box_size, box_size, box_size,
                                       1000,         
                                       True,         
                                       True)         
            
            box.SetPos(chrono.ChVector3d(Bpos.x + box_size * 0.5,
                                         Bpos.y,
                                         Bpos.z))
            self.system.Add(box)

            
            pt2frame = fea.ChLinkPointFrame()
            pt2frame.Initialize(last_node, box)
            self.system.Add(pt2frame)

            
            self.end_bodies.append(box)

    
    
    
    def PrintBodyPositions(self):
        print("---- End-box positions ----")
        for k, body in enumerate(self.end_bodies):
            p = body.GetPos()
            print(f"  Chain {k}: (x={p.x: .3f}, y={p.y: .3f}, z={p.z: .3f})")
        print("----------------------------------------------------------------")




sys  = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)                     


vis_beam_moment = chrono.ChVisualShapeFEA(mesh)
vis_beam_moment.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_moment.SetColorscaleMinMax(-0.4, 0.4)
vis_beam_moment.SetSmoothFaces(True)
vis_beam_moment.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_moment)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)


vsys = chronoirr.ChVisualSystemIrrlicht()
vsys.AttachSystem(sys)
vsys.SetWindowSize(1024, 768)
vsys.SetWindowTitle("Multiple FEA cable chains")
vsys.Initialize()
vsys.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vsys.AddSkyBox()
vsys.AddCamera(chrono.ChVector3d(0.7, 0.4, 1.2))
vsys.AddTypicalLights()


solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

sys.SetTimestepper(chrono.ChTimestepperEulerImplicitLinearized(sys))


step_size = 0.01
frame = 0
while vsys.Run():
    vsys.BeginScene()
    vsys.Render()
    vsys.EndScene()

    sys.DoStepDynamics(step_size)

    
    if frame % 20 == 0:
        model.PrintBodyPositions()
    frame += 1