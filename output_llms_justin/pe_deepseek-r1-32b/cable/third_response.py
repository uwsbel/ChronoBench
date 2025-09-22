import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  # To store the box bodies for position tracking

        # Create a section for the cable beams
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        # Create the ChBuilderCableANCF helper
        builder = fea.ChBuilderCableANCF()

        radius = 0.2  # Radius for arranging trusses around the origin
        length = 0.5  # Length of each beam

        for i in range(n_chains):
            # Calculate the angle for each chain's position
            theta = i * 2 * math.pi / n_chains

            # Position of the truss body
            truss_pos = chrono.ChVector3d(radius * math.cos(theta), radius * math.sin(theta), 0)

            # Create and initialize the truss body
            truss = chrono.ChBody()
            truss.SetPos(truss_pos)
            truss.SetFixed(True)
            system.Add(truss)

            # Number of elements for this chain's beam (increasing with each chain)
            n_elements = 10 + 2 * i

            # Build the beam for this chain
            builder.BuildBeam(
                mesh,
                msection_cable2,
                n_elements,
                truss_pos,
                chrono.ChVector3d(truss_pos.x, truss_pos.y, length)
            )

            # Get the first and last nodes of the beam
            first_node = builder.GetFirstBeamNodes().front()
            last_node = builder.GetLastBeamNodes().back()

            # Create and initialize the hinge constraint for the start node
            constraint_start = fea.ChLinkNodeFrame()
            constraint_start.Initialize(first_node, truss)
            system.AddLink(constraint_start)

            # Create the box body at the end of the beam
            box = chrono.ChBody()
            box.SetPos(chrono.ChVector3d(truss_pos.x, truss_pos.y, length))
            box_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.05, 0.05, 0.1))
            box.AddVisualShape(box_shape)
            system.Add(box)
            self.boxes.append(box)

            # Create and initialize the constraint between the last node and the box
            constraint_end = fea.ChLinkNodeFrame()
            constraint_end.Initialize(last_node, box)
            system.AddLink(constraint_end)

    def PrintBodyPositions(self):
        """Prints the positions of the end bodies of each chain."""
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i+1} end position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

# Initialize the system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with 6 chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Add visualizations for the FEM mesh
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

# Set up the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# Set solver and timestepper
solver = chrono.ChSolverMINRES()
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
    model.PrintBodyPositions()  # Print positions at each step