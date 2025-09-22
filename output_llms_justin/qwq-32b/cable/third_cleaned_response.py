import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.end_bodies = []  

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0)  

        builder = fea.ChBuilderCableANCF()
        spacing = 0.1  

        for i in range(n_chains):
            
            start_y = i * spacing
            start_point = chrono.ChVectorD(0, start_y, -0.1)
            end_point = chrono.ChVectorD(0.5, start_y, -0.1)
            
            
            num_elements = 10 + i

            
            builder.BuildBeam(mesh, msection_cable, num_elements, start_point, end_point)
            nodes = builder.GetLastBeamNodes()
            start_node = nodes.front()
            end_node = nodes.back()

            
            mtruss = chrono.ChBody()
            mtruss.SetBodyFixed(True)
            mtruss.SetPos(end_point)
            system.Add(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_node, mtruss)
            system.Add(constraint_hinge)

            
            start_node.SetForce(chrono.ChVectorD(0, -0.7, 0))

            
            box = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)
            box.SetPos(start_node.GetPos())
            system.Add(box)
            self.end_bodies.append(box)

            
            constraint_box = chrono.ChLinkLockSpherical()
            constraint_box.Initialize(start_node.GetFrame(), box)
            system.Add(constraint_box)

            
            box_shape = chrono.ChVisualShape(box.GetFrame(), chrono.ChVectorD(0.05, 0.05, 0.05), False)
            box.AddVisualShape(box_shape)

    def PrintBodyPositions(self):
        for idx, body in enumerate(self.end_bodies):
            print(f"Chain {idx+1} end body position: {body.GetPos()}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)


model = Model1(sys, mesh, n_chains=6)


visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(visualizebeamB)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
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
    model.PrintBodyPositions()  
    sys.DoStepDynamics(0.01)