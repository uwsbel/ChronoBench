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
            
            angle = 2 * math.pi * i / self.n_chains
            radius = 0.3
            
            
            start_point = chrono.ChVector3d(
                radius * math.cos(angle),
                radius * math.sin(angle),
                -0.1
            )
            end_point = chrono.ChVector3d(
                radius * math.cos(angle) + 0.5,
                radius * math.sin(angle),
                -0.1
            )
            
            
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)
            
            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                10 + i,  
                start_point,
                end_point
            )
            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            end_body = chrono.ChBody()
            end_body.SetBodyType(chrono.ChBodyType.BODY_TYPE_RIGID)
            end_body.SetMass(1.0)
            end_body.SetPos(end_point)
            
            
            end_body_shape = chrono.ChVisualShape()
            end_body_shape.SetGeometry(chrono.ChBoxGeometry(0.05, 0.05, 0.05))
            end_body_shape.SetMaterialSurface(chrono.ChVisualMaterial())
            end_body_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
            end_body.AddVisualShape(end_body_shape)
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_hinge)
            
            constraint_fixed = chrono.ChLinkLockFrameNode()
            constraint_fixed.Initialize(mtruss, end_body)
            self.system.Add(constraint_fixed)
            
            
            self.end_bodies.append(end_body)
            
            
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
            
            
            self.system.Add(end_body)
            
    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: ({pos.x}, {pos.y}, {pos.z})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Multiple Chains')
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