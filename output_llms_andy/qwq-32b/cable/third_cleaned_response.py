import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.boxes = []  

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        spacing = 0.1  
        for i in range(n_chains):
            
            y_offset = (i - (n_chains - 1)/2) * spacing
            start_point = chrono.ChVectorD(0, y_offset, -0.1)
            end_point = chrono.ChVectorD(0.5, y_offset, -0.1)
            num_elements = 10 + i  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_point,
                end_point
            )

            
            first_node = builder.GetLastBeamNodes().front()
            first_node.SetForce(chrono.ChVectorD(0, -0.7, 0))

            
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
            system.Add(box)
            self.boxes.append(box)

            
            link_node_to_box = fea.ChLinkNodeBody()
            link_node_to_box.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(link_node_to_box)

            
            link_box_to_truss = chrono.ChLinkLockPointFrame()
            link_box_to_truss.Initialize(box, mtruss)
            system.Add(link_box_to_truss)

    def PrintBodyPositions(self):
        
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx} box position: {pos}")


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
vis.SetWindowTitle('FEA Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
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