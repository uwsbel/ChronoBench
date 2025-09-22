import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.end_boxes = []
        for i in range(n_chains):
            # Create section with cable properties
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)

            # Calculate beam parameters
            num_elements = 10 + i * 2
            start_x = i * 0.5
            end_x = (i + 1) * 0.5
            start = chrono.ChVector3d(start_x, 0, -0.1)
            end = chrono.ChVector3d(end_x, 0, -0.1)

            # Build the beam
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, msection_cable, num_elements, start, end)
            
            # Get beam nodes
            nodes = builder.GetLastBeamNodes()
            front_node = nodes.front()
            back_node = nodes.back()

            # Apply force to front node
            front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Create truss at back node position
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(back_node.GetPos())
            system.Add(mtruss)

            # Hinge constraint between truss and back node
            truss_constraint = fea.ChLinkNodeFrame()
            truss_constraint.Initialize(back_node, mtruss)
            system.Add(truss_constraint)

            # Create box at front node position
            box = chrono.ChBody()
            box.SetMass(1.0)
            box.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
            box.SetPos(front_node.GetPos())
            
            # Add visualization for box
            box_vis = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
            box.AddVisualShape(box_vis)
            system.Add(box)
            self.end_boxes.append(box)

            # Constraint between front node and box
            box_constraint = fea.ChLinkNodeFrame()
            box_constraint.Initialize(front_node, box)
            system.Add(box_constraint)

    def PrintBodyPositions(self):
        """Print positions of end bodies for all chains"""
        for idx, box in enumerate(self.end_boxes):
            pos = box.GetPos()
            print(f"Chain {idx} end body: X={pos.x:.4f}, Y={pos.y:.4f}, Z={pos.z:.4f}")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Add visualization for FEM mesh
vis_beam_moments = chrono.ChVisualShapeFEA(mesh)
vis_beam_moments.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_moments.SetColorscaleMinMax(-0.4, 0.4)
vis_beam_moments.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam_moments)

# Add node visualization
vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(vis_nodes)

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Cables with Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1.5))
vis.AddTypicalLights()

# Configure solver
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)

# Set timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()