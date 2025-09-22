import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.end_bodies = []

        
        for i in range(n_chains):
            
            x_offset = i * 0.1 - (n_chains-1)*0.1/2  
            start_point = chrono.ChVector3d(x_offset, 0, -0.1)
            end_point = chrono.ChVector3d(x_offset + 0.5, 0, -0.1)

            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)

            
            builder = fea.ChBuilderCableANCF()

            
            num_elements = 5 + i*2

            
            builder.BuildBeam(
                mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            
            end_body = chrono.ChBody()
            end_body.SetPos(end_point)
            end_body.SetMass(0.1)
            end_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            end_body.SetCollide(True)
            end_body.GetCollideModel().AddBox(0.05, 0.05, 0.05)
            system.Add(end_body)
            self.end_bodies.append(end_body)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), end_body)
            system.Add(constraint_hinge)

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(start_point)

            
            constraint_hinge_start = fea.ChLinkNodeFrame()
            constraint_hinge_start.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge_start)

            
            end_body.SetForce(chrono.ChVector3d(0, -0.1*(i+1), 0))

    def PrintBodyPositions(self):
        print("\nEnd body positions at current step:")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1}: X={pos.x:.3f}, Y={pos.y:.3f}, Z={pos.z:.3f}")


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


for body in model.end_bodies:
    vis_box = chrono.ChVisualShapeBox(0.05)
    vis_box.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    body.AddVisualShape(vis_box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1.5))
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

    
    if step % 10 == 0:
        model.PrintBodyPositions()

    vis.EndScene()
    sys.DoStepDynamics(0.01)
    step += 1