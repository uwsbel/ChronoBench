import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        
        self.end_bodies = []

        
        for i in range(n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)             
            msection_cable2.SetYoungModulus(0.01e9)        
            msection_cable2.SetRayleighDamping(0.0001)     

            
            builder = fea.ChBuilderCableANCF()

            
            n_elements = 1 + i

            
            z_offset = -0.1 * i

            
            builder.BuildBeam(
                mesh,                                       
                msection_cable2,                            
                n_elements,                                 
                chrono.ChVector3d(0, 0, z_offset),          
                chrono.ChVector3d(0.1 + 0.1 * i, 0, z_offset)  
            )

            
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            
            mbox = chrono.ChBodyEasyBox(0.04, 0.1, 0.04, 1000)
            end_node = builder.GetLastBeamNodes().back()
            mbox.SetPos(end_node.GetPos() + chrono.ChVector3d(0.02, 0, 0))
            system.Add(mbox)
            self.end_bodies.append(mbox)

            
            constraint_pos = fea.ChLinkNodeFrame()
            constraint_pos.Initialize(end_node, mbox)
            system.Add(constraint_pos)

            
            constraint_dir = fea.ChLinkNodeSlopeFrame()
            constraint_dir.Initialize(end_node, mbox)
            constraint_dir.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir)

    def PrintBodyPositions(self):
        
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print("Chain {0} end body position: x={1:.4f}, y={2:.4f}, z={3:.4f}".format(
                i, pos.x, pos.y, pos.z))



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
vis.SetWindowTitle('FEA cables')
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
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    model.PrintBodyPositions()