import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()

        
        end_bodies = []

        
        for i in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                10 + i,  
                chrono.ChVector3d(0, 0, -0.1 * (i + 1)),  
                chrono.ChVector3d(0.5 + 0.5 * i, 0, -0.1 * (i + 1))  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            end_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
            end_body.SetPos(builder.GetLastBeamNodes().back().GetPos())  
            end_body.SetFixed(False)  
            end_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  
            system.Add(end_body)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(builder.GetLastBeamNodes().back(), end_body)
            system.Add(constraint)  

            
            constraint2 = fea.ChLinkNodeFrame()
            constraint2.Initialize(builder.GetLastBeamNodes().back(), end_body)
            system.Add(constraint2)  

            
            end_bodies.append(end_body)

        
        
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

    def PrintBodyPositions(self, end_bodies):
        
        for i, body in enumerate(end_bodies):
            pos = body.GetPos()
            print(f"Chain {i + 1} end body position: {pos.x}, {pos.y}, {pos.z}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)  


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
    model.PrintBodyPositions(model.end_bodies)
    sys.DoStepDynamics(0.01)