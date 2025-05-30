import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_boxes = []

        for i in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(chrono.ChVector3d(0, i * 0.2, 0))
            system.Add(mtruss)

            
            msection = fea.ChBeamSectionCable()
            msection.SetDiameter(0.015)
            msection.SetYoungModulus(0.01e9)
            msection.SetRayleighDamping(0.0001)

            
            builder = fea.ChBuilderCableANCF()
            start = chrono.ChVector3d(0, i * 0.2, 0)
            end = chrono.ChVector3d(0.5, i * 0.2, 0)
            n_elements = 10 + i
            builder.BuildBeam(mesh, msection, n_elements, start, end)

            
            front_node = builder.GetLastBeamNodes().front()
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(front_node, mtruss)
            system.Add(constraint_hinge)

            
            back_node = builder.GetLastBeamNodes().back()
            back_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            box = chrono.ChBody()
            box.SetPos(end)
            box.SetMass(0.1)
            box.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            system.Add(box)

            
            box_shape = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            box_shape.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
            box.AddVisualShape(box_shape)

            
            node_frame_constraint = fea.ChLinkNodeFrame()
            node_frame_constraint.Initialize(back_node, box)
            system.Add(node_frame_constraint)

            self.end_boxes.append(box)

    def PrintBodyPositions(self):
        for i, box in enumerate(self.end_boxes):
            pos = box.GetPos()
            print(f"Chain {i} end position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multi-chain FEA Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.8, 0.8, -1.2))
vis.AddTypicalLights()


beam_vis = chrono.ChVisualShapeFEA(mesh)
beam_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_vis.SetColorscaleMinMax(-0.4, 0.4)
beam_vis.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(beam_vis)

node_vis = chrono.ChVisualShapeFEA(mesh)
node_vis.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
node_vis.SetSymbolsThickness(0.006)
node_vis.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(node_vis)


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
sys.SetSolver(solver)


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()