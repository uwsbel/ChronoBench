import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chains = []
        self.system = system
        self.mesh = mesh

        
        for i in range(self.n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  
            
            builder = fea.ChBuilderCableANCF()
            
            elements_per_chain = 10 + i * 5  
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                elements_per_chain,  
                chrono.ChVector3d(0, 0, -0.1 + i * 0.05),  
                chrono.ChVector3d(0.5 + i * 0.1, 0, -0.1 + i * 0.05)  
            )

            
            end_nodes = builder.GetLastBeamNodes().front()
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_nodes, mtruss)
            self.system.Add(constraint_hinge)  

            
            mbox = chrono.ChBody()
            mbox.SetBodyFixed(True)  
            mbox.SetBodyPosition(end_nodes.GetPosition())
            self.system.Add(mbox)

            
            constraint_box = fea.ChLinkFrame()
            constraint_box.Initialize(end_nodes, mbox)
            self.system.Add(constraint_box)

            
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
            vis.AttachSystem(self.system)  
            vis.SetWindowSize(1024, 768)  
            vis.SetWindowTitle('FEA cables')  
            vis.Initialize()  
            vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
            vis.AddSkyBox()  
            vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  
            vis.AddTypicalLights()  

            
            visualizebox = chrono.ChVisualShapeFEA(mesh)
            visualizebox.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BOX_MZ)  
            visualizebox.SetColorscaleMinMax(-0.4, 0.4)  
            visualizebox.SetSmoothFaces(True)  
            visualizebox.SetWireframe(False)  
            mesh.AddVisualShapeFEA(visualizebox)  

            
            visualizenode = chrono.ChVisualShapeFEA(mesh)
            visualizenode.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
            visualizenode.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
            visualizenode.SetSymbolsThickness(0.006)  
            visualizenode.SetSymbolsScale(0.01)  
            visualizenode.SetZbufferHide(False)  
            mesh.AddVisualShapeFEA(visualizenode)  

            
            def PrintBodyPositions():
                for chain in self.chains:
                    print(f"Chain {chain.index}: Box position = {chain.box.GetBodyPosition()}, Truss position = {chain.truss.GetBodyPosition()}")

        
        self.system.Add(self.PrintBodyPositions)


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)  


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


model.PrintBodyPositions()