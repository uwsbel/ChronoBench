import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.boxes = []  # List to store end boxes for each chain

        # Create section for all beams
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        spacing = 0.1  # Spacing between chains along y-axis
        for i in range(n_chains):
            # Calculate y-offset for the ith chain
            y_offset = (i - (n_chains - 1)/2) * spacing
            start_point = chrono.ChVectorD(0, y_offset, -0.1)
            end_point = chrono.ChVectorD(0.5, y_offset, -0.1)
            num_elements = 10 + i  # Increasing elements per chain

            # Create fixed truss body
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            # Create builder and build beam
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_point,
                end_point
            )

            # Apply force to front node
            first_node = builder.GetLastBeamNodes().front()
            first_node.SetForce(chrono.ChVectorD(0, -0.7, 0))

            # Create end box and constraints
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
            system.Add(box)
            self.boxes.append(box)

            # Link beam's last node to box
            link_node_to_box = fea.ChLinkNodeBody()
            link_node_to_box.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(link_node_to_box)

            # Link box to truss (fixed constraint)
            link_box_to_truss = chrono.ChLinkLockPointFrame()
            link_box_to_truss.Initialize(box, mtruss)
            system.Add(link_box_to_truss)

    def PrintBodyPositions(self):
        """Print positions of end boxes for each chain."""
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx} box position: {pos}")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh)
sys.Add(mesh)

# Add FEA visualization
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

# Setup Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()

# Configure solver and timestepper
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

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()  # Print positions every step