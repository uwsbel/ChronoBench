import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.end_bodies = []  
        self.n_chains = n_chains
        self.mesh = mesh

        
        self.visualizations = []

        for i in range(n_chains):
            
            n_elements = 10 + i  

            
            y_offset = i * 0.05  

            
            start = chrono.ChVector3d(0, y_offset, -0.1)
            end = chrono.ChVector3d(0.5, y_offset, -0.1)

            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)

            
            mtruss = chrono.ChBody()
            mtruss.SetBodyFixed(True)
            mtruss.SetPos(start)
            system.Add(mtruss)

            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh,
                msection_cable,
                n_elements,
                start,
                end
            )

            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            
            box_size = chrono.ChVector3d(0.03, 0.03, 0.03)
            mbox = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z,  
                                        1000,  
                                        True,  
                                        True)  
            
            end_node = builder.GetLastBeamNodes().back()
            mbox.SetPos(end_node.GetPos())
            system.Add(mbox)

            
            constraint_box = fea.ChLinkNodeBody()
            constraint_box.Initialize(end_node, mbox)
            system.Add(constraint_box)

            
            self.end_bodies.append(mbox)

            
            visualizebeamA = fea.ChVisualShapeFEA(mesh)
            visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
            visualizebeamA.SetSmoothFaces(True)
            visualizebeamA.SetWireframe(False)
            mesh.AddVisualShapeFEA(visualizebeamA)
            self.visualizations.append(visualizebeamA)

            
            visualizebeamB = fea.ChVisualShapeFEA(mesh)
            visualizebeamB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualizebeamB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
            visualizebeamB.SetSymbolsThickness(0.006)
            visualizebeamB.SetSymbolsScale(0.01)
            visualizebeamB.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(visualizebeamB)
            self.visualizations.append(visualizebeamB)

    def PrintBodyPositions(self):
        print("End body positions:")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"  Chain {i+1}: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables - Multiple Chains')
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


step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    if step % 20 == 0:
        model.PrintBodyPositions()
    step += 1