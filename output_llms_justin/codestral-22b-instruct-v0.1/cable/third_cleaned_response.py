import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.system = system
        self.mesh = mesh
        self.end_bodies = []

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()

        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            mbox = chrono.ChBody()
            mbox.SetMass(1.0)
            mbox.SetPos(chrono.ChVector3d(0.5 + i * 0.2, 0, -0.1))

            
            builder.BuildBeam(
                self.mesh,  
                msection_cable,  
                10 + i * 2,  
                chrono.ChVector3d(0, 0, -0.1 - i * 0.1),  
                chrono.ChVector3d(0.5 + i * 0.2, 0, -0.1 - i * 0.1)  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)  

            
            constraint_box = chrono.ChLinkLockPrismatic()
            constraint_box.Initialize(builder.GetLastBeamNodes().back(), mbox)
            self.system.Add(constraint_box)  

            self.end_bodies.append(mbox)

        
        
        visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
        visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
        visualizebeamA.SetSmoothFaces(True)  
        visualizebeamA.SetWireframe(False)  
        self.mesh.AddVisualShapeFEA(visualizebeamA)  

        
        visualizebeamB = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
        visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
        visualizebeamB.SetSymbolsThickness(0.006)  
        visualizebeamB.SetSymbolsScale(0.01)  
        visualizebeamB.SetZbufferHide(False)  
        self.mesh.AddVisualShapeFEA(visualizebeamB)  

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"End body {i+1} position: {body.GetPos()}")


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
    model.PrintBodyPositions()