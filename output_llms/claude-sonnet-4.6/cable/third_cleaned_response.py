import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        
        self.end_bodies = []  

        for i in range(n_chains):
            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)          
            msection_cable.SetYoungModulus(0.01e9)     
            msection_cable.SetRayleighDamping(0.0001)

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)          

            
            z_offset = -i * 0.2        
            pt_A = chrono.ChVector3d(0,   0, z_offset)   
            pt_B = chrono.ChVector3d(0.5, 0, z_offset)   

            
            num_elements = i + 1       
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh,
                msection_cable,
                num_elements,
                pt_A,   
                pt_B    
            )

            
            builder.GetLastBeamNodes().front().SetForce(
                chrono.ChVector3d(0, -0.7, 0)
            )

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(
                builder.GetLastBeamNodes().back(),   
                mtruss
            )
            system.Add(constraint_hinge)

            
            end_node = builder.GetLastBeamNodes().front()

            box_body = chrono.ChBody()
            box_body.SetMass(0.1)
            box_body.SetPos(end_node.GetPos())   
            system.Add(box_body)

            
            box_shape = chrono.ChVisualShapeBox(0.02, 0.02, 0.02)
            box_body.AddVisualShape(box_shape)

            
            constraint_end = fea.ChLinkNodeFrame()
            constraint_end.Initialize(end_node, box_body)
            system.Add(constraint_end)

            
            self.end_bodies.append(box_body)

    def PrintBodyPositions(self):
        
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"  Chain {i+1:2d} end-body pos: "
                  f"x={pos.x:+.4f}  y={pos.y:+.4f}  z={pos.z:+.4f}")





sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)




visualizebeamA = fea.ChVisualShapeFEA(mesh)          
visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)


visualizebeamB = fea.ChVisualShapeFEA(mesh)          
visualizebeamB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables – multiple chains')
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




step_number = 0
print_every  = 100   

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)

    step_number += 1
    if step_number % print_every == 0:
        print(f"\n--- t = {sys.GetChTime():.2f} s ---")
        model.PrintBodyPositions()