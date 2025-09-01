import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  

        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        
        builder = fea.ChBuilderCableANCF()

        radius = 0.2  
        length = 0.5  

        for i in range(n_chains):
            
            theta = i * 2 * math.pi / n_chains

            
            truss_pos = chrono.ChVector3d(radius * math.cos(theta), radius * math.sin(theta), 0)

            
            truss = chrono.ChBody()
            truss.SetPos(truss_pos)
            truss.SetFixed(True)
            system.Add(truss)

            
            n_elements = 10 + 2 * i

            
            builder.BuildBeam(
                mesh,
                msection_cable2,
                n_elements,
                truss_pos,
                chrono.ChVector3d(truss_pos.x, truss_pos.y, length)
            )

            
            first_node = builder.GetFirstBeamNodes().front()
            last_node = builder.GetLastBeamNodes().back()

            
            constraint_start = fea.ChLinkNodeFrame()
            constraint_start.Initialize(first_node, truss)
            system.AddLink(constraint_start)

            
            box = chrono.ChBody()
            box.SetPos(chrono.ChVector3d(truss_pos.x, truss_pos.y, length))
            box_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.05, 0.05, 0.1))
            box.AddVisualShape(box_shape)
            system.Add(box)
            self.boxes.append(box)

            
            constraint_end = fea.ChLinkNodeFrame()
            constraint_end.Initialize(last_node, box)
            system.AddLink(constraint_end)

    def PrintBodyPositions(self):
        
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i+1} end position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")


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
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
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