import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.end_boxes = []  

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0001)

        spacing = 0.6  

        for i in range(n_chains):
            
            start = chrono.ChVector3d(i * spacing, 0, -0.1)
            end = start + chrono.ChVector3d(0.5, 0, 0)
            num_elements = i + 1  

            
            truss = chrono.ChBody()
            truss.SetPos(start)
            truss.SetFixed(True)
            system.Add(truss)

            
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh,
                msection_cable,
                num_elements,
                start,
                end
            )

            
            first_node = builder.GetLastBeamNodes().front()
            first_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)  
            box.SetPos(end)
            system.Add(box)

            
            last_node = builder.GetLastBeamNodes().back()
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(last_node, box)
            system.Add(constraint)

            self.end_boxes.append(box)

    def PrintBodyPositions(self):
        for i, box in enumerate(self.end_boxes):
            pos = box.GetPos()
            print(f"Chain {i} end position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)


visbeam = chrono.ChVisualShapeFEA(mesh)
visbeam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visbeam.SetColorscaleMinMax(-0.4, 0.4)
visbeam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visbeam)

visnodes = chrono.ChVisualShapeFEA(mesh)
visnodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visnodes.SetSymbolsThickness(0.006)
visnodes.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(visnodes)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Chains')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 2, -3))
vis.AddTypicalLights()


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
sys.SetSolver(solver)


step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    
    if step % 10 == 0:
        model.PrintBodyPositions()
    step += 1