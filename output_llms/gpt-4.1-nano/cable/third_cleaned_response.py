import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  

        
        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            mtruss.SetPos(chrono.ChVector3d(0.2 * i, 0, 0))
            self.system.Add(mtruss)

            
            section_cable = fea.ChBeamSectionCable()
            section_cable.SetDiameter(0.015)  
            section_cable.SetYoungModulus(0.01e9)  
            section_cable.SetRayleighDamping(0.0001)

            
            builder = fea.ChBuilderCableANCF()

            
            start_point = chrono.ChVector3d(0, 0, -0.1 + i * 0.05)  
            end_point = chrono.ChVector3d(0.5 + i * 0.2, 0, -0.1 + i * 0.05)

            
            num_elements = 10 + i  

            
            builder.BuildBeam(
                self.mesh,
                section_cable,
                num_elements,
                start_point,
                end_point
            )

            
            last_nodes = builder.GetLastBeamNodes()
            end_node = last_nodes.back()

            
            end_body = chrono.ChBody()
            end_body.SetPos(end_node.GetPos())
            end_body.SetRot(chrono.QUNIT)
            self.system.Add(end_body)

            
            
            link_end = fea.ChLinkNodeBody()
            link_end.Initialize(end_node, end_body)
            self.system.Add(link_end)

            
            self.end_bodies.append(end_body)

            
            
            last_node = last_nodes.front()  
            last_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            

            
            

    def PrintBodyPositions(self):
        
        print("End body positions:")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i}: Position = ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")



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
vis.SetWindowTitle('FEA cables with multiple chains')
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
    
    model.PrintBodyPositions()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(0.01)