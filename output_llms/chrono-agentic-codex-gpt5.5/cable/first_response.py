"""ANCF cable beam hinged to ground under gravity.

This PyChrono 9.0 SMC/FEA simulation builds a flexible beam from ANCF cable
elements, pins one end to a fixed truss, and lets gravity deform the beam.
Irrlicht renders the deformed beam and visible nodal glyphs during the loop.
"""

import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Constants: compact cable beam setup ===
TIME_STEP = 0.01
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
BEAM_LENGTH = 1.5
BEAM_DIAMETER = 0.025
BEAM_ELEMENTS = 16
Y0 = 0.0
Z0 = 0.0


def node_position(node):
    """Return a node position while keeping hot-loop attribute access explicit."""
    return node.GetPos()


def main():
    # === System & solver: SMC FEA with ANCF-compatible SparseQR ===
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    solver = chrono.ChSolverSparseQR()
    solver.UseSparsityPatternLearner(True)
    solver.LockSparsityPattern(True)
    solver.SetVerbose(False)
    sys.SetSolver(solver)
    timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
    sys.SetTimestepper(timestepper)

    # === Flexible beam: ANCF cable elements under automatic gravity ===
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)
    section = fea.ChBeamSectionCable()
    section.SetDiameter(BEAM_DIAMETER)
    section.SetYoungModulus(0.01e9)
    section.SetRayleighDamping(0.005)

    builder = fea.ChBuilderCableANCF()
    builder.BuildBeam(
        mesh,
        section,
        BEAM_ELEMENTS,
        chrono.ChVector3d(0.0, Y0, Z0),
        chrono.ChVector3d(BEAM_LENGTH, Y0, Z0),
    )
    beam_nodes_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
    beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]  # cache: reused in loop
    root_node = beam_nodes[0]  # cache: hinged node
    tip_node = beam_nodes[-1]  # cache: logged free-end node
    sys.Add(mesh)

    # === Ground hinge: pin the left cable node to a fixed truss ===
    truss = chrono.ChBody()
    truss.SetFixed(True)
    sys.Add(truss)
    hinge = fea.ChLinkNodeFrame()
    hinge.Initialize(root_node, truss)
    sys.Add(hinge)
    # FEA cable: no contact material needed because motion is driven by hinge and gravity only.

    # === FEA visualization: deformation field plus nodal positions ===
    vis_surface = chrono.ChVisualShapeFEA(mesh)
    vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
    vis_surface.SetColorscaleMinMax(0.0, 0.7)
    vis_surface.SetSmoothFaces(True)
    vis_surface.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_surface)

    vis_nodes = chrono.ChVisualShapeFEA(mesh)
    vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    vis_nodes.SetSymbolsThickness(0.012)
    vis_nodes.SetSymbolsScale(0.018)
    vis_nodes.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(vis_nodes)

    # === Irrlicht window: complete render block for deformation review ===
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ANCF cable beam under gravity")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, 1.0, 2.2), chrono.ChVector3d(0.75, -0.45, 0.0))
    vis.AddTypicalLights()
    vis.AddGrid(
        0.25,
        0.25,
        12,
        12,
        chrono.ChCoordsysd(chrono.ChVector3d(0.75, -0.75, 0.0), chrono.QUNIT),
        chrono.ChColor(0.35, 0.35, 0.35),
    )

    # === Main loop: render, log nodal positions, and advance FEA ===
    frame = 0
    try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1
            for _ in range(RENDER_EVERY):
                time_now = sys.GetChTime()  # cache: used in logging and stop check
                root_pos = node_position(root_node)  # cache: node getter once per step
                tip_pos = node_position(tip_node)  # cache: node getter once per step
                print(
                    f"t={time_now:.3f} root=({root_pos.x:.3f},{root_pos.y:.3f},{root_pos.z:.3f}) "
                    f"tip=({tip_pos.x:.3f},{tip_pos.y:.3f},{tip_pos.z:.3f})"
                )
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
    except (OSError, IOError) as exc:  # disk or permission failure while writing review data
        traceback.print_exc()
        raise
    except RuntimeError as exc:  # Chrono solver or Irrlicht runtime failure
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
