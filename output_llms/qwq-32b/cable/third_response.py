import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  # List to store the end boxes of each chain

        # Create a common section for all beams
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()

        # Define parameters for chain placement and element count
        spacing = 0.1  # Y-axis spacing between chains
        base_start = chrono.ChVectorD(0.0, 0.0, -0.1)
        base_end = chrono.ChVectorD(0.5, 0.0, -0.1)

        for i in range(n_chains):
            # Calculate positions for this chain
            y_offset = i * spacing
            start_point = chrono.ChVectorD(base_start.x, base_start.y + y_offset, base_start.z)
            end_point = chrono.ChVectorD(base_end.x, base_end.y + y_offset, base_end.z)

            # Create fixed truss body at the beam's end point
            truss = chrono.ChBody()
            truss.SetBodyFixed(True)
            truss.SetPos(end_point)
            self.system.Add(truss)

            # Determine number of elements for this chain (increasing with each chain)
            num_elements = 10 + i  # Starting at 10 elements and increasing by 1 per chain

            # Build the beam
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            # Retrieve nodes for this beam
            nodes = builder.GetLastBeamNodes()

            # Apply force to the front node (start of the beam)
            nodes.front().SetForce(chrono.ChVectorD(0, -0.7, 0))

            # Create a box connected to the start node (free end)
            box = chrono.ChBody()
            box.SetMass(1.0)  # Assign some mass to the box
            box.SetPos(start_point)
            self.system.Add(box)

            # Add visualization for the box
            box_shape = chrono.ChVisualShape()
            box_shape.SetDimensions(chrono.ChVectorD(0.05, 0.05, 0.05))
            box_shape.SetColor(chrono.ChColor(0.5, 0, 0))
            box.AddVisualShape(box_shape)

            # Create a constraint between the start node and the box
            link = fea.ChLinkNodeBody()
            link.Initialize(nodes.front(), box, chrono.ChFrameD())
            self.system.Add(link)

            self.boxes.append(box)  # Store the box for position tracking

    def PrintBodyPositions(self):
        """Print positions of all end boxes at each simulation step."""
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx}: Position = ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")

# Initialize simulation system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()

# Add FEA mesh visualization
visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
visualize_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualize_beam)

# Simulation settings
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()  # Print positions at each step

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  # Store end boxes for position tracking

        # Create a common section for all beams
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()

        # Define chain parameters
        spacing = 0.1  # Y-axis spacing between chains
        base_start = chrono.ChVectorD(0.0, 0.0, -0.1)
        base_end = chrono.ChVectorD(0.5, 0.0, -0.1)

        for i in range(n_chains):
            y_offset = i * spacing
            start_point = chrono.ChVectorD(base_start.x, base_start.y + y_offset, base_start.z)
            end_point = chrono.ChVectorD(base_end.x, base_end.y + y_offset, base_end.z)

            # Create fixed truss at beam's end
            truss = chrono.ChBody()
            truss.SetBodyFixed(True)
            truss.SetPos(end_point)
            self.system.Add(truss)

            # Varying number of elements per chain
            num_elements = 10 + i  # Start at 10, increase by 1 per chain

            # Build the beam
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            nodes = builder.GetLastBeamNodes()
            nodes.front().SetForce(chrono.ChVectorD(0, -0.7, 0))  # Apply force to start node

            # Create box connected to beam's start node
            box = chrono.ChBody()
            box.SetMass(1.0)
            box.SetPos(start_point)
            self.system.Add(box)

            # Add visualization to box
            box_shape = chrono.ChVisualShape()
            box_shape.SetDimensions(chrono.ChVectorD(0.05, 0.05, 0.05))
            box_shape.SetColor(chrono.ChColor(0.5, 0, 0))
            box.AddVisualShape(box_shape)

            # Link box to start node
            link = fea.ChLinkNodeBody()
            link.Initialize(nodes.front(), box, chrono.ChFrameD())
            self.system.Add(link)

            self.boxes.append(box)

    def PrintBodyPositions(self):
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx}: Position = ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")

# Initialize simulation
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()

# Add FEA visualization
visualize_beam = chrono.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
visualize_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualize_beam)

# Simulation settings
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
    model.PrintBodyPositions()