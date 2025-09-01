import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.boxes = []

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  
        msection_cable.SetYoungModulus(0.01e9)  
        msection_cable.SetRayleighDamping(0.0001)

        for i in range(n_chains):
            
            start_x = i * 0.6
            start = chrono.ChVector3d(start_x, 0, -0.1)
            end = chrono.ChVector3d(start_x + 0.5, 0, -0.1)
            num_elements = 5 + i  

            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, msection_cable, num_elements, start, end)

            
            nodes = builder.GetLastBeamNodes()
            front_node = nodes.front()
            back_node = nodes.back()

            
            front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(start)
            system.Add(mtruss)

            
            constraint_truss = fea.ChLinkNodeFrame()
            constraint_truss.Initialize(front_node, mtruss)
            system.Add(constraint_truss)

            
            mbox = chrono.ChBody()
            mbox.SetMass(1.0)
            mbox.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
            mbox.SetPos(end)

            
            box_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
            box_shape.SetColor(chrono.ChColor(1, 0, 0))  
            mbox.AddVisualShape(box_shape)

            system.Add(mbox)
            self.boxes.append(mbox)

            
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(back_node, mbox)
            system.Add(constraint_box)

    def PrintBodyPositions(self):
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx} end body position: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")






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