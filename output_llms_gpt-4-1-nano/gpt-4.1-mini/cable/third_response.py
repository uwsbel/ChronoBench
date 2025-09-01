import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of beams composed of ANCF cable elements,
# each chain with its own fixed truss and connected box body at the free end.
# This model demonstrates multiple chains with increasing number of elements,
# proper constraints, loads, visualization, and printing of end body positions.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        # Store end bodies of chains (the boxes) for position printing
        self.end_bodies = []

        # Parameters for positioning chains to avoid overlap
        # We'll space chains along Y axis
        y_spacing = 0.15

        for i in range(self.n_chains):
            # Create beam section properties
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)      # 15 mm diameter
            msection_cable2.SetYoungModulus(0.01e9) # 0.01 GPa
            msection_cable2.SetRayleighDamping(0.0001)

            # Builder for this chain
            builder = fea.ChBuilderCableANCF()

            # Number of elements increases with chain index (start from 3 to avoid zero elements)
            n_elements = 3 + i

            # Define start point 'A' and end point 'B' for the beam in 3D space
            # Stagger chains in Y axis to avoid overlap
            start_point = chrono.ChVector3d(0, i * y_spacing, -0.1)
            end_point = chrono.ChVector3d(0.5, i * y_spacing, -0.1)

            # Build beam and add nodes/elements to mesh
            builder.BuildBeam(
                self.mesh,
                msection_cable2,
                n_elements,
                start_point,
                end_point
            )

            # Add mesh to system (only once, outside loop, but safe here as mesh is same)
            # (Will be done once outside after model creation)
            
            # Apply force at the front node (start node)
            front_node = builder.GetLastBeamNodes().front()
            force_vector = chrono.ChVector3d(0, -0.7, 0)
            front_node.SetForce(force_vector)

            # Create fixed truss body for this chain
            mtruss = chrono.ChBody()
            mtruss.SetBodyFixed(True)
            self.system.Add(mtruss)

            # Hinge constraint between the last node (back node) and the truss body at beam start point
            constraint_hinge = fea.ChLinkNodeFrame()
            back_node = builder.GetLastBeamNodes().back()
            # Initialize hinge at the position coincident with back node
            constraint_hinge.Initialize(back_node, mtruss)
            self.system.Add(constraint_hinge)

            # Create a box body connected to beam endpoint (back node)
            box = chrono.ChBodyEasyBox(0.02, 0.04, 0.02, 1000, True, True)  # size and density, visualization and collision
            # Position box at end point, slightly translated in Z to avoid clash
            box.SetPos(end_point + chrono.ChVector3d(0, 0, 0.02))
            box.SetMass(0.1)  # Set some mass to box
            box.SetInertiaXX(chrono.ChVectorD(1e-4, 1e-4, 1e-4))  # Small inertia
            self.system.Add(box)

            # Link box to the beam endpoint via Marker and a constraint to allow motion
            # Use a ChLinkPointFrame to link node and box at node position:
            link_node_box = chrono.ChLinkPointFrame()
            link_node_box.Initialize(back_node, box)
            self.system.Add(link_node_box)

            # Save box for printing positions later
            self.end_bodies.append(box)

    def PrintBodyPositions(self):
        print("Positions of end bodies (box bodies) of each chain:")
        for idx, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {idx} end body position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")


# =====================================================================
# Main simulation setup
# =====================================================================

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Add mesh to the system once after all chains are added

# Add visualization for the FEM mesh:
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node positions:
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables - Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.9, -1))  # Adjust camera for higher overview
vis.AddTypicalLights()

# Set solver type and settings
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop with printing positions
time_step = 0.01
step_count = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    # Print positions every 50 steps (~0.5 s)
    step_count += 1
    if step_count % 50 == 0:
        print(f"Step: {step_count}, time: {sys.GetChTime():.2f} s")
        model.PrintBodyPositions()