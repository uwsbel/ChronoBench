"""Multiple ANCF cable chains with rigid endpoint boxes in a Y-up SMC system.

The model creates six parallel cable chains by default, each with its own fixed
truss reference, increasing ANCF element count, an endpoint box constrained to
the cable tip, and a downward/lateral load so the chains bend smoothly.  There
are no contact surfaces; the dynamics are driven by FEA constraints, gravity,
and nodal forces.
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Constants === define geometry and solver timing once for stable cable dynamics
TIME_STEP = 0.01
SIM_END = 3.0
RENDER_FPS = 25.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


class Model1:
    """Builds several offset ANCF cable chains and prints endpoint body states."""

    def __init__(self, sys, n_chains=6):
        self.sys = sys
        self.n_chains = n_chains
        self.mesh = fea.ChMesh()
        self.mesh.SetAutomaticGravity(True)
        self.end_bodies = []
        self.chain_nodes = []
        self.builders = []  # cache: keep SWIG builder containers alive
        self.node_sets = []  # cache: keep GetLastBeamNodes containers alive
        self._build_chains()
        self._add_visualization()
        self.sys.Add(self.mesh)

    def _make_truss(self, pos, index):
        truss = chrono.ChBody()
        truss.SetName(f"chain_{index}_truss")
        truss.SetFixed(True)
        truss.SetPos(pos)
        truss.SetMass(1.0)
        truss.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
        marker = chrono.ChVisualShapeBox(0.10, 0.10, 0.10)
        marker.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
        truss.AddVisualShape(marker)
        self.sys.AddBody(truss)
        return truss

    def _make_box(self, pos, index):
        box = chrono.ChBody()
        box.SetName(f"chain_{index}_end_box")
        box.SetMass(0.25)
        box.SetInertiaXX(chrono.ChVector3d(0.002, 0.002, 0.002))
        box.SetPos(pos)
        box_shape = chrono.ChVisualShapeBox(0.16, 0.12, 0.16)
        box_shape.SetColor(chrono.ChColor(0.8, 0.35, 0.1))
        box.AddVisualShape(box_shape)
        box.EnableCollision(False)
        self.sys.AddBody(box)
        return box

    def _build_chains(self):
        # FEA cable: no contact material needed because these chains use constraints and loads only.
        section = fea.ChBeamSectionCable()
        section.SetDiameter(0.018)
        section.SetYoungModulus(0.01e9)
        section.SetRayleighDamping(0.001)

        for i in range(self.n_chains):
            n_elements = 6 + i
            z_offset = (i - (self.n_chains - 1) / 2.0) * 0.26
            start = chrono.ChVector3d(0.0, 0.25 + 0.04 * i, z_offset)
            end = chrono.ChVector3d(1.05 + 0.08 * i, 0.25 + 0.04 * i, z_offset)

            truss = self._make_truss(start, i)
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(self.mesh, section, n_elements, start, end)
            beam_nodes = builder.GetLastBeamNodes()
            nodes = [beam_nodes[j] for j in range(beam_nodes.size())]
            self.builders.append(builder)
            self.node_sets.append(beam_nodes)
            self.chain_nodes.append(nodes)

            root_node = nodes[0]
            tip_node = nodes[-1]
            hinge_root = fea.ChLinkNodeFrame()
            hinge_root.Initialize(root_node, truss)
            self.sys.Add(hinge_root)

            box = self._make_box(end, i)
            endpoint_hinge = fea.ChLinkNodeFrame()
            endpoint_hinge.Initialize(tip_node, box)
            self.sys.Add(endpoint_hinge)

            force_y = -0.45 - 0.08 * i
            force_z = 0.04 * (i - (self.n_chains - 1) / 2.0)
            tip_node.SetForce(chrono.ChVector3d(0.0, force_y, force_z))
            self.end_bodies.append(box)

    def _add_visualization(self):
        cable_shape = chrono.ChVisualShapeFEA(self.mesh)
        cable_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        cable_shape.SetColorscaleMinMax(-0.20, 0.20)
        cable_shape.SetSmoothFaces(True)
        cable_shape.SetWireframe(False)
        self.mesh.AddVisualShapeFEA(cable_shape)

        node_shape = chrono.ChVisualShapeFEA(self.mesh)
        node_shape.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        node_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        node_shape.SetSymbolsThickness(0.006)
        node_shape.SetSymbolsScale(0.012)
        node_shape.SetZbufferHide(False)
        self.mesh.AddVisualShapeFEA(node_shape)

    def PrintBodyPositions(self):
        time = self.sys.GetChTime()  # cache: reused for one print line
        fields = [f"t={time:.3f}"]
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()  # cache: one body pose read per chain per step
            fields.append(f"chain_{i}=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f})")
        print("  ".join(fields))


# === System & solver === use the ANCF cable solver/timestepper required for flexible cables
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)
timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(timestepper)


# === Bodies, FEA mesh, and constraints === construct all chains through Model1
model = Model1(sys, n_chains=6)
end_bodies = model.end_bodies  # cache: reused in the logging loop


# === Visualization === full Irrlicht scene with camera and ground reference grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multiple ANCF Cable Chains")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.1, 1.0, 1.7), chrono.ChVector3d(0.6, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.25,
    0.25,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.6, -0.45, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.45, 0.45, 0.45),
)


# === Main loop === render at a review cadence while stepping every ANCF time step
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: reused for logging and loop status
            model.PrintBodyPositions()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # disk or frame-output failure
    print(f"Output failed: {exc}")
    raise
finally:
    pass
