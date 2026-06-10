import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr












class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains

        self.trusses = []
        self.end_bodies = []
        self.constraints = []
        self.end_nodes = []

        
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0005)

        
        
        
        msection_cable.SetDensity(1000.0)

        builder = fea.ChBuilderCableANCF()

        for i in range(n_chains):
            
            n_elements = 6 + 2 * i

            
            
            z_offset = -0.30 + i * 0.12
            y_offset = 0.0
            length = 0.35 + 0.04 * i

            start_point = chrono.ChVector3d(0.0, y_offset, z_offset)
            end_point = chrono.ChVector3d(length, y_offset, z_offset)

            
            
            
            mtruss = chrono.ChBody()
            mtruss.SetName(f"fixed_truss_{i}")
            mtruss.SetFixed(True)
            mtruss.SetPos(start_point)
            system.Add(mtruss)
            self.trusses.append(mtruss)

            
            
            
            builder.BuildBeam(
                mesh,
                msection_cable,
                n_elements,
                start_point,
                end_point
            )

            
            
            beam_nodes = builder.GetLastBeamNodes()
            start_node = beam_nodes[0]
            end_node = beam_nodes[len(beam_nodes) - 1]

            self.end_nodes.append(end_node)

            
            
            
            
            root_constraint = fea.ChLinkNodeFrame()
            root_constraint.Initialize(start_node, mtruss)
            system.Add(root_constraint)
            self.constraints.append(root_constraint)

            
            
            
            box_size = 0.045
            box_density = 1000.0

            end_body = chrono.ChBodyEasyBox(
                box_size,
                box_size,
                box_size,
                box_density,
                True,    
                False    
            )
            end_body.SetName(f"end_box_{i}")
            end_body.SetPos(end_point)
            end_body.SetFixed(False)
            system.Add(end_body)
            self.end_bodies.append(end_body)

            
            
            
            
            end_constraint = fea.ChLinkNodeFrame()
            end_constraint.Initialize(end_node, end_body)
            system.Add(end_constraint)
            self.constraints.append(end_constraint)

            
            
            
            
            force_magnitude = -0.25 * (i + 1)
            end_node.SetForce(chrono.ChVector3d(0.0, force_magnitude, 0.0))

    def PrintBodyPositions(self, time=None):
        
        prefix = f"t = {time:8.4f} s | " if time is not None else ""

        body_strings = []
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            body_strings.append(
                f"chain {i}: ({pos.x: .5f}, {pos.y: .5f}, {pos.z: .5f})"
            )

        print(prefix + " | ".join(body_strings))






sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)


sys.Add(mesh)







visualizebeamA = fea.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)


visualizebeamB = fea.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA multiple ANCF cable chains with end bodies")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()


vis.AddCamera(
    chrono.ChVector3d(0.75, 0.45, -1.35),
    chrono.ChVector3d(0.35, -0.10, 0.0)
)

vis.AddTypicalLights()






solver = chrono.ChSolverMINRES()

if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(300)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)






time_step = 0.005

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    
    model.PrintBodyPositions(sys.GetChTime())