"""ANCF cable FEA simulation using an SMC system.

The model builds a flexible cable from ANCF beam elements, pins the rear node to a
fixed truss, and applies a downward load to the front node. The cable section uses
Rayleigh damping 0.0001, and the system is solved with a configured MINRES solver.
"""

import contextlib
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Constants === define cable geometry, material, solver, and recording cadence
TIME_STEP = 0.001
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CABLE_ELEMENTS = 10
CABLE_START = chrono.ChVector3d(0.0, 0.0, 0.0)
CABLE_END = chrono.ChVector3d(0.5, 0.0, 0.0)
CABLE_DIAMETER = 0.015
CABLE_DENSITY = 1000.0
CABLE_YOUNG_MODULUS = 0.01e9
CABLE_RAYLEIGH_DAMPING = 0.0001
FRONT_NODE_FORCE = chrono.ChVector3d(0, -0.7, 0)


def main():
    # === System & solver === SMC FEA system with prompt-required MINRES settings
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    solver = chrono.ChSolverMINRES()
    print("Using MINRES solver")
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)
    sys.SetSolver(solver)

    timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
    sys.SetTimestepper(timestepper)

    # === Cable mesh === ANCF cable section, beam builder, fixed rear, loaded front
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(False)

    section = fea.ChBeamSectionCable()
    section.SetDiameter(CABLE_DIAMETER)
    section.SetDensity(CABLE_DENSITY)
    section.SetYoungModulus(CABLE_YOUNG_MODULUS)
    section.SetRayleighDamping(CABLE_RAYLEIGH_DAMPING)

    builder = fea.ChBuilderCableANCF()
    builder.BuildBeam(mesh, section, CABLE_ELEMENTS, CABLE_START, CABLE_END)
    cable_nodes_ref = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
    cable_nodes = [cable_nodes_ref[i] for i in range(cable_nodes_ref.size())]  # cache: reused for constraints and logging
    rear_node = cable_nodes[0]  # cache: fixed support node
    front_node = cable_nodes[-1]  # cache: force and telemetry node
    front_node.SetForce(FRONT_NODE_FORCE)

    truss = chrono.ChBody()
    truss.SetFixed(True)
    sys.AddBody(truss)

    rear_pin = fea.ChLinkNodeFrame()
    rear_pin.Initialize(rear_node, truss)
    sys.Add(rear_pin)

    sys.Add(mesh)

    # FEA cable: no contact material needed because the cable is constrained and force-driven only.
    vis_cable = chrono.ChVisualShapeFEA(mesh)
    vis_cable.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
    vis_cable.SetColorscaleMinMax(0.0, 0.08)
    vis_cable.SetSmoothFaces(True)
    vis_cable.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_cable)

    vis_nodes = chrono.ChVisualShapeFEA(mesh)
    vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    vis_nodes.SetSymbolsThickness(0.006)
    vis_nodes.SetSymbolsScale(0.01)
    vis_nodes.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(vis_nodes)

    # === Visualization === Irrlicht window configured after Initialize for Y-up FEA view
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ANCF Cable with MINRES Solver")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, 0.25, 0.8), chrono.ChVector3d(0.25, -0.08, 0.0))
    vis.AddTypicalLights()
    vis.AddGrid(
        0.05,
        0.05,
        20,
        20,
        chrono.ChCoordsysd(chrono.ChVector3d(0.25, -0.2, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
        chrono.ChColor(0.4, 0.4, 0.4),
    )

    # === Main loop === render at fixed cadence and advance FEA dynamics
    frame = 0
    try:
        with contextlib.ExitStack() as file_stack:

            while vis.Run() and sys.GetChTime() < SIM_END:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                frame += 1
                for _ in range(RENDER_EVERY):
                    sim_time = sys.GetChTime()  # cache: reused for logging and stop condition
                    front_pos = front_node.GetPos()  # cache: queried once per step
                    front_vel = front_node.GetPosDt()  # cache: queried once per step
                    sys.DoStepDynamics(TIME_STEP)
                    if sys.GetChTime() >= SIM_END:
                        break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # output directory or file write guard
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
