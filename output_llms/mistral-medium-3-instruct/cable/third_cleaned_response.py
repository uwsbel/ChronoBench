import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chains = []
        self.end_bodies = []

        
        for i in range(n_chains):
            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)  
            msection_cable.SetYoungModulus(0.01e9)  
            msection_cable.SetRayleighDamping(0.0001)

            
            builder = fea.ChBuilderCableANCF()

            
            x_pos = i * 0.2  
            num_elements = 10 + i * 2  

            
            builder.BuildBeam(
                mesh,
                msection_cable,
                num_elements,
                chrono.ChVector3d(x_pos, 0, -0.1),  
                chrono.ChVector3d(x_pos + 0.5, 0, -0.1)  
            )

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            
            end_body = chrono.ChBody()
            end_body.SetPos(chrono.ChVector3d(x_pos + 0.5, 0, -0.1))
            end_body.SetMass(0.1)
            end_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            end_body.SetCollide(True)
            end_body.GetCollisionModel().AddBox(0.05, 0.05, 0.05)
            system.Add(end_body)

            
            box_vis = chrono.ChVisualShapeBox(0.05)
            box_vis.SetColor(chrono.ChColor(0.5, 0.2 + i*0.1, 0.1 + i*0.15))
            end_body.AddVisualShape(box_vis)

            
            link = fea.ChLinkNodeFrame()
            link.Initialize(builder.GetLastBeamNodes().back(), end_body)
            system.Add(link)

            
            end_body.SetForce(chrono.ChVector3d(0, -0.5 - i*0.1, 0))

            
            self.chains.append(builder)
            self.end_bodies.append(end_body)

    def PrintBodyPositions(self):
        print("\nCurrent end body positions:")
        for i, body in enumerate(self.end_bodies):
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
vis.SetWindowTitle('FEA Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.6, -1.5))
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


step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if step % 10 == 0:
        model.PrintBodyPositions()

    sys.DoStepDynamics(0.01)
    step += 1