import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.end_bodies = []  # Store boxes for each chain

        # Create a common section for all beams
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0)  # Correct damping to zero

        builder = fea.ChBuilderCableANCF()
        spacing = 0.1  # Y-axis spacing between chains

        for i in range(n_chains):
            # Define beam start/end points with spacing
            start_y = i * spacing
            start_point = chrono.ChVectorD(0, start_y, -0.1)
            end_point = chrono.ChVectorD(0.5, start_y, -0.1)
            
            # Increase elements per chain
            num_elements = 10 + i

            # Build current beam
            builder.BuildBeam(mesh, msection_cable, num_elements, start_point, end_point)
            nodes = builder.GetLastBeamNodes()
            start_node = nodes.front()
            end_node = nodes.back()

            # Create fixed truss at beam's end point
            mtruss = chrono.ChBody()
            mtruss.SetBodyFixed(True)
            mtruss.SetPos(end_point)
            system.Add(mtruss)

            # Create hinge constraint between end node and truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(end_node, mtruss)
            system.Add(constraint_hinge)

            # Apply force to start node
            start_node.SetForce(chrono.ChVectorD(0, -0.7, 0))

            # Create box attached to start node (free end)
            box = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)
            box.SetPos(start_node.GetPos())
            system.Add(box)
            self.end_bodies.append(box)

            # Constraint between start node and box
            constraint_box = chrono.ChLinkLockSpherical()
            constraint_box.Initialize(start_node.GetFrame(), box)
            system.Add(constraint_box)

            # Add visualization for box
            box_shape = chrono.ChVisualShape(box.GetFrame(), chrono.ChVectorD(0.05, 0.05, 0.05), False)
            box.AddVisualShape(box_shape)

    def PrintBodyPositions(self):
        for idx, body in enumerate(self.end_bodies):
            print(f"Chain {idx+1} end body position: {body.GetPos()}")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

# Create model with 6 chains
model = Model1(sys, mesh, n_chains=6)

# Add FEM mesh visualization
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

# Setup visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()

# Configure solver and timestepper
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
    model.PrintBodyPositions()  # Print positions before advancing time
    sys.DoStepDynamics(0.01)