import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# ----------------------------------------------------------------------------
# Model1:
# Multiple ANCF cable chains.  Each chain has:
#   - a fixed truss body at the root
#   - an ANCF cable beam with an increasing number of elements
#   - a dynamic box body attached to the free end of the cable
#   - node-frame constraints at both cable ends
#   - an external nodal force on the free end
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains

        self.trusses = []
        self.end_bodies = []
        self.constraints = []
        self.end_nodes = []

        # Cable section properties
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0005)

        # Density is important if gravity should affect the cable mass.
        # Some examples omit this because a default may exist, but setting it
        # explicitly is safer.
        msection_cable.SetDensity(1000.0)

        builder = fea.ChBuilderCableANCF()

        for i in range(n_chains):
            # Increasing number of ANCF cable elements for each chain
            n_elements = 6 + 2 * i

            # Offset each chain in Z to avoid overlap.
            # Slightly different lengths make the chains easier to distinguish.
            z_offset = -0.30 + i * 0.12
            y_offset = 0.0
            length = 0.35 + 0.04 * i

            start_point = chrono.ChVector3d(0.0, y_offset, z_offset)
            end_point = chrono.ChVector3d(length, y_offset, z_offset)

            # ----------------------------------------------------------------
            # Fixed truss/root body
            # ----------------------------------------------------------------
            mtruss = chrono.ChBody()
            mtruss.SetName(f"fixed_truss_{i}")
            mtruss.SetFixed(True)
            mtruss.SetPos(start_point)
            system.Add(mtruss)
            self.trusses.append(mtruss)

            # ----------------------------------------------------------------
            # Build ANCF cable beam
            # ----------------------------------------------------------------
            builder.BuildBeam(
                mesh,
                msection_cable,
                n_elements,
                start_point,
                end_point
            )

            # Avoid using .front()/.back() because these are not always exposed
            # consistently in Python bindings.
            beam_nodes = builder.GetLastBeamNodes()
            start_node = beam_nodes[0]
            end_node = beam_nodes[len(beam_nodes) - 1]

            self.end_nodes.append(end_node)

            # ----------------------------------------------------------------
            # Constraint: root cable node to fixed truss
            # This fixes the position of the first node to the truss frame.
            # ----------------------------------------------------------------
            root_constraint = fea.ChLinkNodeFrame()
            root_constraint.Initialize(start_node, mtruss)
            system.Add(root_constraint)
            self.constraints.append(root_constraint)

            # ----------------------------------------------------------------
            # Dynamic box body attached to cable endpoint
            # ----------------------------------------------------------------
            box_size = 0.045
            box_density = 1000.0

            end_body = chrono.ChBodyEasyBox(
                box_size,
                box_size,
                box_size,
                box_density,
                True,    # visualization
                False    # collision
            )
            end_body.SetName(f"end_box_{i}")
            end_body.SetPos(end_point)
            end_body.SetFixed(False)
            system.Add(end_body)
            self.end_bodies.append(end_body)

            # ----------------------------------------------------------------
            # Constraint: cable endpoint node to box body
            # This connects the beam endpoint to the dynamic box.
            # ----------------------------------------------------------------
            end_constraint = fea.ChLinkNodeFrame()
            end_constraint.Initialize(end_node, end_body)
            system.Add(end_constraint)
            self.constraints.append(end_constraint)

            # ----------------------------------------------------------------
            # External force applied to the free/end node.
            # Gravity also acts on the cable and the attached box.
            # ----------------------------------------------------------------
            force_magnitude = -0.25 * (i + 1)
            end_node.SetForce(chrono.ChVector3d(0.0, force_magnitude, 0.0))

    def PrintBodyPositions(self, time=None):
        """Print positions of all dynamic end bodies."""
        prefix = f"t = {time:8.4f} s | " if time is not None else ""

        body_strings = []
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            body_strings.append(
                f"chain {i}: ({pos.x: .5f}, {pos.y: .5f}, {pos.z: .5f})"
            )

        print(prefix + " | ".join(body_strings))


# ----------------------------------------------------------------------------
# Initialize physical system and mesh
# ----------------------------------------------------------------------------

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

mesh = fea.ChMesh()

# Create model with multiple cable chains
model = Model1(sys, mesh, n_chains=6)

# Add the mesh to the physical system
sys.Add(mesh)


# ----------------------------------------------------------------------------
# FEM visualization
# ----------------------------------------------------------------------------

# Beam moment visualization
visualizebeamA = fea.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Node position visualization
visualizebeamB = fea.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)


# ----------------------------------------------------------------------------
# Irrlicht visualization
# ----------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA multiple ANCF cable chains with end bodies")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()

# Camera adjusted to see all chains
vis.AddCamera(
    chrono.ChVector3d(0.75, 0.45, -1.35),
    chrono.ChVector3d(0.35, -0.10, 0.0)
)

vis.AddTypicalLights()


# ----------------------------------------------------------------------------
# Solver and timestepper
# ----------------------------------------------------------------------------

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


# ----------------------------------------------------------------------------
# Simulation loop
# ----------------------------------------------------------------------------

time_step = 0.005

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    # Print end-body positions at every simulation step
    model.PrintBodyPositions(sys.GetChTime())