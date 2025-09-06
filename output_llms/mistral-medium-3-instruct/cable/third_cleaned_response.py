import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.system = system
        self.mesh = mesh
        self.bodies = []  

        
        for i in range(n_chains):
            self.create_chain(i)

    def create_chain(self, chain_index):
        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)

        
        builder = fea.ChBuilderCableANCF()

        
        start_point = chrono.ChVector3d(0, chain_index * 0.2, -0.1)
        end_point = chrono.ChVector3d(0.5, chain_index * 0.2, -0.1)

        
        n_elements = 5 + chain_index * 2

        
        builder.BuildBeam(
            self.mesh,  
            msection_cable,  
            n_elements,  
            start_point,  
            end_point  
        )

        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        self.system.Add(mtruss)

        
        end_body = chrono.ChBody()
        end_body.SetPos(end_point)
        end_body.SetMass(0.1)
        end_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
        end_body.SetCollide(True)
        end_body.GetCollisionModel().AddBox(0.05, 0.05, 0.05)
        self.system.Add(end_body)
        self.bodies.append(end_body)

        
        link_beam_body = fea.ChLinkNodeFrame()
        link_beam_body.Initialize(builder.GetLastBeamNodes().back(), end_body)
        self.system.Add(link_beam_body)

        
        link_truss = fea.ChLinkNodeFrame()
        link_truss.Initialize(builder.GetLastBeamNodes().front(), mtruss)
        self.system.Add(link_truss)

        
        end_body.SetForce(chrono.ChVector3d(0, -0.7, 0))

    def PrintBodyPositions(self):
        for i, body in enumerate(self.bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")


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
vis.SetWindowTitle('FEA Multiple Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, -2))
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

    
    if sys.GetChTime() % 0.1 < 0.01:
        model.PrintBodyPositions()

    vis.EndScene()
    sys.DoStepDynamics(0.01)