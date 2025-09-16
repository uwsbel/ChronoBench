import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()

        
        spacing = 0.1  
        base_start = chrono.ChVectorD(0.0, 0.0, -0.1)
        base_end = chrono.ChVectorD(0.5, 0.0, -0.1)

        for i in range(n_chains):
            
            y_offset = i * spacing
            start_point = chrono.ChVectorD(base_start.x, base_start.y + y_offset, base_start.z)
            end_point = chrono.ChVectorD(base_end.x, base_end.y + y_offset, base_end.z)

            
            truss = chrono.ChBody()
            truss.SetBodyFixed(True)
            truss.SetPos(end_point)
            self.system.Add(truss)

            
            num_elements = 10 + i  

            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            
            nodes = builder.GetLastBeamNodes()

            
            nodes.front().SetForce(chrono.ChVectorD(0, -0.7, 0))

            
            box = chrono.ChBody()
            box.SetMass(1.0)  
            box.SetPos(start_point)
            self.system.Add(box)

            
            box_shape = chrono.ChVisualShape()
            box_shape.SetDimensions(chrono.ChVectorD(0.05, 0.05, 0.05))
            box_shape.SetColor(chrono.ChColor(0.5, 0, 0))
            box.AddVisualShape(box_shape)

            
            link = fea.ChLinkNodeBody()
            link.Initialize(nodes.front(), box, chrono.ChFrameD())
            self.system.Add(link)

            self.boxes.append(box)  

    def PrintBodyPositions(self):
        
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx}: Position = ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()


visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
visualize_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualize_beam)


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

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()

        
        spacing = 0.1  
        base_start = chrono.ChVectorD(0.0, 0.0, -0.1)
        base_end = chrono.ChVectorD(0.5, 0.0, -0.1)

        for i in range(n_chains):
            y_offset = i * spacing
            start_point = chrono.ChVectorD(base_start.x, base_start.y + y_offset, base_start.z)
            end_point = chrono.ChVectorD(base_end.x, base_end.y + y_offset, base_end.z)

            
            truss = chrono.ChBody()
            truss.SetBodyFixed(True)
            truss.SetPos(end_point)
            self.system.Add(truss)

            
            num_elements = 10 + i  

            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            nodes = builder.GetLastBeamNodes()
            nodes.front().SetForce(chrono.ChVectorD(0, -0.7, 0))  

            
            box = chrono.ChBody()
            box.SetMass(1.0)
            box.SetPos(start_point)
            self.system.Add(box)

            
            box_shape = chrono.ChVisualShape()
            box_shape.SetDimensions(chrono.ChVectorD(0.05, 0.05, 0.05))
            box_shape.SetColor(chrono.ChColor(0.5, 0, 0))
            box.AddVisualShape(box_shape)

            
            link = fea.ChLinkNodeBody()
            link.Initialize(nodes.front(), box, chrono.ChFrameD())
            self.system.Add(link)

            self.boxes.append(box)

    def PrintBodyPositions(self):
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx}: Position = ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()


visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
visualize_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualize_beam)


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