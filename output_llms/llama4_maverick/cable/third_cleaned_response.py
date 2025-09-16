import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr






class Model1:
    def __init__(self, n_chains=6):
        self.sys = chrono.ChSystemSMC()
        self.mesh = fea.ChMesh()
        self.sys.Add(self.mesh)
        self.n_chains = n_chains
        self.end_bodies = []

        for i in range(n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  

            
            builder = fea.ChBuilderCableANCF()

            
            num_elements = 5 + i  
            builder.BuildBeam(
                self.mesh,  
                msection_cable2,  
                num_elements,  
                chrono.ChVector3d(i * 0.1, 0, -0.1),  
                chrono.ChVector3d(i * 0.1 + 0.5, 0, -0.1)  
            )

            
            builder.GetLastBeamNodes().front().SetFixed(True)  
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            self.sys.Add(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            self.sys.Add(constraint_hinge)  

            
            body = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)
            body.SetPos(builder.GetLastBeamNodes().back().GetPos())
            self.sys.Add(body)
            self.end_bodies.append(body)

            
            constraint = chrono.ChLinkMateGeneric()
            constraint.Initialize(builder.GetLastBeamNodes().back(), body)
            self.sys.Add(constraint)

            
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
            print(f"Chain {i+1} end body position: {body.GetPos()}")

def main():
    model = Model1(n_chains=6)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(model.sys)  
    vis.SetWindowSize(1024, 768)  
    vis.SetWindowTitle('FEA cables')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  
    vis.AddTypicalLights()  

    
    solver = chrono.ChSolverMINRES()  
    model.sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)  
    solver.SetVerbose(False)  

    
    ts = chrono.ChTimestepperEulerImplicitLinearized(model.sys)
    model.sys.SetTimestepper(ts)

    
    while vis.Run():
        vis.BeginScene()  
        vis.Render()  
        vis.EndScene()  
        model.sys.DoStepDynamics(0.01)  
        model.PrintBodyPositions()

if __name__ == "__main__":
    main()