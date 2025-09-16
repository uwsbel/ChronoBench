import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chain_bodies = []  

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)

        
        for i in range(n_chains):
            
            builder = fea.ChBuilderCableANCF()

            
            x_pos = i * 0.1  
            start_point = chrono.ChVector3d(x_pos, 0, -0.1)
            end_point = chrono.ChVector3d(x_pos + 0.5, 0, -0.1)

            
            n_elements = 5 + i * 2  

            
            builder.BuildBeam(
                mesh,
                msection_cable,
                n_elements,
                start_point,
                end_point
            )

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            
            
            constraint_fix = fea.ChLinkNodeFrame()
            constraint_fix.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_fix)

            
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            box_body = chrono.ChBody()
            box_body.SetPos(end_point)
            box_body.SetMass(0.1)
            box_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            box_body.SetCollide(True)
            box_body.GetCollisionModel().AddBox(0.05, 0.05, 0.05)
            system.Add(box_body)

            
            link_node_body = fea.ChLinkNodeBody()
            link_node_body.Initialize(builder.GetLastBeamNodes().back(), box_body)
            system.Add(link_node_body)

            
            self.chain_bodies.append(box_body)

            
            box_vis = chrono.ChBoxShape()
            box_vis.GetBoxGeometry().SetLengths(chrono.ChVector3d(0.05, 0.05, 0.05))
            box_body.AddVisualShape(box_vis)

    def PrintBodyPositions(self):
        print("\nCurrent positions of end bodies:")
        for i, body in enumerate(self.chain_bodies):
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


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.6, -1))
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