import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chain_bodies = []  

        for i in range(self.n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  

            
            builder = fea.ChBuilderCableANCF()

            
            num_elements = 10 + i  
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                num_elements,  
                chrono.ChVector3d(0, 0, -0.1 + i * 0.1),  
                chrono.ChVector3d(0.5 + i * 0.1, 0, -0.1 + i * 0.1),  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            body_box = chrono.ChBody()
            body_box.SetMass(1)
            body_box.SetPos(chrono.ChVector3d(0.5 + i * 0.1, 0, 0))
            body_box.EnableCollision(True)
            system.Add(body_box)

            
            joint_revolute = chrono.ChLinkLockRevolute()
            joint_revolute.Initialize(builder.GetLastBeamNodes().back(), body_box, chrono.ChCoordsysD(chrono.ChVector3d(0.5 + i * 0.1, 0, 0), chrono.QUNIT))
            system.Add(joint_revolute)

            self.chain_bodies.append(body_box)  

    def PrintBodyPositions(self):
        for i, body in enumerate(self.chain_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: x={pos.x}, y={pos.y}, z={pos.z}")



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
vis.AddLogo(chrono.GetChonoDataFile('logo_pychrono_alpha.png'))  
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