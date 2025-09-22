import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.end_bodies = []
        self.n_chains = n_chains
        for n in range(self.n_chains) :
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  
            
            builder = fea.ChBuilderCableANCF()
            
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                5+ n,  
                chrono.ChVector3d(0, 0, -0.1),  
                chrono.ChVector3d(0.5, 0, -0.1)  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            
            mbody = chrono.ChBodyEasyBox(0.2, 0.02, 0.02, 1000)
            mbody.SetPos(chrono.ChVector3d(
                0.6, 0.01 * (n - self.n_chains / 2), -0.1))  
            mbody.SetFixed(False)  
            system.AddBody(mbody)  

            
            constraint_hinge_11 = fea.ChLinkNodeFrame()
            constraint_hinge_11.Initialize(builder.GetLastBeamNodes().back(), mbody)
            system.Add(constraint_hinge_11)  

            
            constraint_hinge_12 = fea.ChLinkNodeFrame()
            constraint_hinge_12.Initialize(
                mbody, builder.GetLastBeamNodes().back() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(constraint_hinge_12)  

            if n != self.n_chains -1:
                
                mbody_to_connect = chrono.ChBodyEasyBox(0.2, 0.02, 0.02, 1000)
                mbody_to_connect.SetPos(chrono.ChVector3d(
                    0.6, 0.01 * (n + 1 - self.n_chains / 2), -0.1))  
                mbody_to_connect.SetFixed(False)  
                system.AddBody(mbody_to_connect)  

                
                constraint_hinge_21 = fea.ChLinkNodeFrame()
                constraint_hinge_21.Initialize(
                    mbody, mbody_to_connect + chrono.ChVector3d(0.1, 0, 0))
                system.Add(constraint_hinge_21)  

                
                constraint_hinge_22 = fea.ChLinkNodeFrame()
                constraint_hinge_22.Initialize(
                    builder.GetLastBeamNodes().back(), mbody_to_connect)
                system.Add(constraint_hinge_22)  

            else:
                
                self.end_bodies.append(mbody)

            
            
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

    def PrintBodyPositions(self):
        for b in self.end_bodies:
            print(b.GetPos())

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
    sys.DoStepDynamics(0.01)