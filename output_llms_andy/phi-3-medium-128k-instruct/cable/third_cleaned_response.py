import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.sys = system
        self.mesh = mesh
        self.visualizebeamA = None
        self.visualizebeamB = None
        self.truss_bodies = []
        self.chain_lengths = [0.5 + i * 0.1 for i in range(n_chains)]  
        self.end_positions = [chrono.ChVector3d(0, 0, -0.1)  

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()

        
        self.visualizebeamA = chrono.ChVisualShapeFEA(mesh)
        self.visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
        self.visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
        self.visualizebeamA.SetSmoothFaces(True)  
        self.visualizebeamA.SetWireframe(False)  
        mesh.AddVisualShapeFEA(self.visualizebeamA)  

        
        self.visualizebeamB = chrono.ChVisualShapeFEA(mesh)
        self.visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
        self.visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
        self.visualizebeamB.SetSymbolsThickness(0.006)  
        self.visualizebeamB.SetSymbolsScale(0.01)  
        self.visualizebeamB.SetZbufferHide(False)  
        mesh.AddVisualShapeFEA(self.visualizebeamB)  

        
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(self.sys)  
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
            self.sys.SetSolver(solver)
            solver.SetMaxIterations(200)
            solver.SetTolerance(1e-10)
            solver.EnableDiagonalPreconditioner(True)
            solver.EnableWarmStart(True)  
            solver.SetVerbose(False)
        
        self.ts = chrono.ChTimestepperEulerImplicitLinearized(self.sys)
        self.sys.SetTimestepper(self.ts)

        
        self.model = self.CreateModel()
        self.sys.Add(self.model)  

    def CreateModel(self):
        
        sys = chrono.ChSystemSMC()
        mesh = fea.ChMesh()

        
        model = Model1(sys, mesh)
        sys.Add(mesh)

        
        self.visualizebeamA = chrono.ChVisualShapeFEA(mesh)
        self.visualizebeamB = chrono.ChVisualShapeFEA(mesh)

        
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
        self.sys.SetSolver(solver)
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableDiagonalPreconditioner(True)
        solver.EnableWarmStart(True)
        self.sys.SetTimestepper(self.ts)

        return model

    def PrintBodyPositions(self):
        for i in range(self.n_chains):
            print(f"Chain {i+1} end body position: {self.truss_bodies[i].GetPos()}")

    def RunSimulation(self):
        while vis.Run():
            vis.BeginScene()  
            vis.Render()  
            vis.EndScene()  
            self.sys.DoStepDynamics(0.01)  
            self.PrintBodyPositions()  


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)


model.RunSimulation()