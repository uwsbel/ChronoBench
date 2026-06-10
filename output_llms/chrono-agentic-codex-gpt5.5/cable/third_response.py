"""Multiple ANCF cable chains in a PyChrono SMC system.

The model builds several flexible cable chains from ANCF beam elements.  Each
chain has a fixed truss anchor, a different element count, a loaded endpoint,
and a rigid box body constrained to the cable end so the printed body positions
show each chain's dynamic response.
"""

import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Constants: compact final values for the standalone cable model ===
TIME_STEP = 0.01
SIM_END = 3.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
N_CHAINS = 6
CABLE_DIAMETER = 0.015
CABLE_YOUNG_MODULUS = 0.01e9
BOX_SIZE = 0.08
BOX_DENSITY = 900.0


class Model1:
    """Builds multiple anchored cable chains and prints their endpoint bodies."""

    def __init__(self, system, n_chains=N_CHAINS):
        self.system = system
        self.n_chains = n_chains
        self.mesh = fea.ChMesh()
        self.mesh.SetAutomaticGravity(True)
        self.end_bodies = []
        self.trusses = []
        self.node_sets = []
        self.builders = []
        self.constraints = []

        self.section = fea.ChBeamSectionCable()
        self.section.SetDiameter(CABLE_DIAMETER)
        self.section.SetYoungModulus(CABLE_YOUNG_MODULUS)
        self.section.SetRayleighDamping(0.000)

        for chain_id in range(self.n_chains):
            self._create_chain(chain_id)

        self._add_visualization()
        self.system.Add(self.mesh)

    def _create_chain(self, chain_id):
        z_offset = (chain_id - (self.n_chains - 1) * 0.5) * 0.22
        x0 = 0.0
        x1 = 0.75 + 0.10 * chain_id
        y = 1.0 - 0.03 * chain_id
        n_elements = 6 + 2 * chain_id

        truss = chrono.ChBody()
        truss.SetFixed(True)
        truss.SetPos(chrono.ChVector3d(x0, y, z_offset))
        self.system.AddBody(truss)
        self.trusses.append(truss)

        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(
            self.mesh,
            self.section,
            n_elements,
            chrono.ChVector3d(x0, y, z_offset),
            chrono.ChVector3d(x1, y, z_offset),
        )
        beam_nodes = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
        nodes = [beam_nodes[i] for i in range(beam_nodes.size())]
        start_node = nodes[0]  # cache: reused by anchor constraint
        end_node = nodes[-1]  # cache: reused by load and box constraint
        end_node.SetForce(chrono.ChVector3d(2.0 + 0.25 * chain_id, -1.0 - 0.35 * chain_id, 0.15 * math.sin(chain_id)))

        hinge = fea.ChLinkNodeFrame()
        hinge.Initialize(start_node, truss)
        self.system.Add(hinge)

        box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, False)
        box.SetPos(chrono.ChVector3d(x1, y, z_offset))
        self.system.Add(box)

        endpoint_link = fea.ChLinkNodeFrame()
        endpoint_link.Initialize(end_node, box)
        self.system.Add(endpoint_link)

        self.builders.append(builder)
        self.node_sets.append(nodes)
        self.constraints.extend([hinge, endpoint_link])
        self.end_bodies.append(box)

    def _add_visualization(self):
        vis_surface = chrono.ChVisualShapeFEA(self.mesh)
        vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
        vis_surface.SetColorscaleMinMax(0.0, 5.0)
        vis_surface.SetSmoothFaces(True)
        vis_surface.SetWireframe(False)
        self.mesh.AddVisualShapeFEA(vis_surface)

        vis_glyph = chrono.ChVisualShapeFEA(self.mesh)
        vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        vis_glyph.SetSymbolsThickness(0.006)
        vis_glyph.SetSymbolsScale(0.02)
        vis_glyph.SetZbufferHide(False)
        self.mesh.AddVisualShapeFEA(vis_glyph)

    def PrintBodyPositions(self):
        for chain_id, body in enumerate(self.end_bodies):
            pos = body.GetPos()  # cache: body position used once for formatted physics log
            print(f"chain {chain_id}: body position = ({pos.x:.5f}, {pos.y:.5f}, {pos.z:.5f})")


def build_system():
    # === System & solver: ANCF cable uses SMC, SparseQR, and implicit-linearized stepping ===
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    solver = chrono.ChSolverSparseQR()
    system.SetSolver(solver)
    solver.UseSparsityPatternLearner(True)
    solver.LockSparsityPattern(True)
    solver.SetVerbose(False)
    timestepper = chrono.ChTimestepperEulerImplicitLinearized(system)
    system.SetTimestepper(timestepper)
    # FEA cable: no contact material needed because nodes are driven by constraints and loads only.
    return system


def build_visualization(system):
    # === Visualization: Irrlicht window configured after Initialize as required by 9.0.0 ===
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Multiple ANCF Cable Chains")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.7, 1.6, -2.2), chrono.ChVector3d(0.55, 0.65, 0.0))
    vis.AddTypicalLights()
    return vis


def run():
    # === Bodies, FEA mesh, and constraints: Model1 owns the full chain topology ===
    system = build_system()
    model = Model1(system, n_chains=N_CHAINS)
    vis = build_visualization(system)


    try:
        # === Main loop: render, print endpoint-body positions, and advance dynamics ===
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            model.PrintBodyPositions()
            for _ in range(RENDER_EVERY):
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:
        print(f"simulation failure: {exc}")
        raise
    finally:
        # === Cleanup: keep a visible finally block for robust run structure ===
        pass


if __name__ == "__main__":
    run()
