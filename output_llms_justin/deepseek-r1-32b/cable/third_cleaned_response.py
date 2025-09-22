import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  

        
        for i in range(self.n_chains):
            
            angle = i * (2 * math.pi / self.n_chains)
            
            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)
            
            
            builder = fea.ChBuilderCableANCF()
            
            
            num_elements = 10 + i  
            
            
            radius = 0.2  
            start_point = chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), -0.1)
            end_point = chrono.ChVector3d((radius + 0.5) * math.cos(angle), (radius + 0.5) * math.sin(angle), -0.1)
            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )
            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)
            
            
            end_body = chrono.ChBody()
            end_body.SetMass(0.1)  
            end_body.SetBodyFixed(False)
            
            
            end_body_vis = chrono.ChVisualShape()
            end_body_vis.SetGeometry(chrono.ChBoxGeometry(0.05, 0.05, 0.05))
            end_body_vis.SetMaterialSurface(chrono.ChMaterialSurface())
            end_body.AddVisualShape(end_body_vis)
            
            
            constraint_beam_body = fea.ChLinkNodeBody()
            constraint_beam_body.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_beam_body)
            
            
            self.system.Add(end_body)
            self.end_bodies.append(end_body)
            
            
            self.system.Add(mtruss)

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: ({pos.x}, {pos.y}, {pos.z})")


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
vis.SetWindowTitle('FEA Cables with Multiple Chains')
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