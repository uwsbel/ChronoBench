import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        for i in range(n_chains):
            
            builder = fea.ChBuilderCableANCF()

            
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                3+i,  
                chrono.ChVector3d(0, 0, -0.1 * i),  
                chrono.ChVector3d(0.5 + 0.1 * i, 0, -0.1 * i)  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            mbox = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
            mbox.SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(mbox)

            
            constraint_hinge2 = fea.ChLinkNodeFrame()
            constraint_hinge2.Initialize(builder.GetLastBeamNodes().back(), mbox)
            system.Add(constraint_hinge2)  

            
            constraint_frame = fea.ChLinkNodeFrame()
            constraint_frame.Initialize(
                builder.GetLastBeamNodes().back(),
                mbox,
                False,  
                chrono.ChFramed(builder.GetLastBeamNodes().back()).GetCoord().TransformDirectionLocalToParent(
                    chrono.ChVector3d(1, 0, 0)
                ),
                chrono.ChFramed(mbox).GetCoord().TransformDirectionLocalToParent(chrono.ChVector3d(1, 0, 0))
            )
            system.Add(constraint_frame)  

            
            constraint_frame2 = fea.ChLinkNodeFrame()
            constraint_frame2.Initialize(
                builder.GetLastBeamNodes().back(),
                mbox,
                False,  
                chrono.ChFramed(builder.GetLastBeamNodes().back()).GetCoord().TransformDirectionLocalToParent(
                    chrono.ChVector3d(0, 0, 1)
                ),
                chrono.ChFramed(mbox).GetCoord().TransformDirectionLocalToParent(chrono.ChVector3d(0, 0, 1))
            )
            system.Add(constraint_frame2)  

            
            mbox2 = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
            mbox2.SetPos(builder.GetLastBeamNodes().front().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(mbox2)

            
            constraint_hinge3 = fea.ChLinkNodeFrame()
            constraint_hinge3.Initialize(builder.GetLastBeamNodes().front(), mbox2)
            system.Add(constraint_hinge3)  

            
            constraint_frame3 = fea.ChLinkNodeFrame()
            constraint_frame3.Initialize(
                builder.GetLastBeamNodes().front(),
                mbox2,
                False,  
                chrono.ChFramed(builder.GetLastBeamNodes().front()).GetCoord().TransformDirectionLocalToParent(
                    chrono.ChVector3d(0, 0, 1)
                ),
                chrono.ChFramed(mbox2).GetCoord().TransformDirectionLocalToParent(chrono.ChVector3d(0, 0, 1))
            )
            system.Add(constraint_frame3)  

    def PrintBodyPositions(self, my_system):
        print("Body positions:")
        for body in my_system.GetBodies():
            print(f"  {body.GetName()}: {body.GetPos()}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
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
if solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
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