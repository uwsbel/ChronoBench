"""
Multi-chain ANCF cable simulation with connected end bodies.
Each chain: fixed truss -> ANCF cable -> box body.
plan_type: mbs (FEA cable)
"""
import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Parameters ===
N_CHAINS = 6
TIME_STEP = 0.01
SIM_END = 10.0
RENDER_FPS = 50.0

# Cable properties
CABLE_DIAMETER = 0.015
CABLE_YOUNG = 0.01e9  # Pa (flexible)
CABLE_RAYLEIGH = 0.0001
CABLE_DENSITY = 1000.0  # kg/m3

# Chain layout: chains spread along Z axis, increasing length
CHAIN_START_Z = -0.1
CHAIN_END_X_BASE = 0.1


# === System setup ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# MINRES solver for ANCF cable
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

# Euler implicit linearized timestepper for ANCF
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# Shared cable section
msection_cable = fea.ChBeamSectionCable()
msection_cable.SetDiameter(CABLE_DIAMETER)
msection_cable.SetYoungModulus(CABLE_YOUNG)
msection_cable.SetRayleighDamping(CABLE_RAYLEIGH)

# Fixed truss body (shared across all chains)
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
mtruss.SetName("mtruss")
sys.Add(mtruss)

# Store end bodies for PrintBodyPositions
end_bodies = []


# === Build chains ===
for j in range(N_CHAINS):
    # --- First beam: from truss to box1 ---
    n_elements_1 = 1 + j
    beam_start = chrono.ChVector3d(0, 0, CHAIN_START_Z * j)
    beam_end_1 = chrono.ChVector3d(CHAIN_END_X_BASE + 0.1 * j, 0, CHAIN_START_Z * j)

    builder1 = fea.ChBuilderCableANCF()
    builder1.BuildBeam(mesh, msection_cable, n_elements_1, beam_start, beam_end_1)

    # Fix front node to truss via hinge
    front_node1 = builder1.GetLastBeamNodes().front()
    constraint_hinge = fea.ChLinkNodeFrame()
    constraint_hinge.Initialize(front_node1, mtruss)
    sys.Add(constraint_hinge)

    # Add visual sphere at hinge
    msphere = chrono.ChVisualShapeSphere(0.02)
    constraint_hinge.AddVisualShape(msphere)

    # Apply downward force to end node
    back_node1 = builder1.GetLastBeamNodes().back()
    back_node1.SetForce(chrono.ChVector3d(0, -0.2, 0))

    # --- Box 1 at end of first beam ---
    box1_pos = back_node1.GetPos() + chrono.ChVector3d(0.1, 0, 0)
    mbox1 = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000, True, True, None)
    mbox1.SetPos(box1_pos)
    mbox1.SetName(f"mbox_{j}_1")
    sys.Add(mbox1)

    # Connect end node to box1: position constraint
    constraint_pos1 = fea.ChLinkNodeFrame()
    constraint_pos1.Initialize(back_node1, mbox1)
    sys.Add(constraint_pos1)

    # Direction constraint (beam continues in X direction)
    constraint_dir1 = fea.ChLinkNodeSlopeFrame()
    constraint_dir1.Initialize(back_node1, mbox1)
    constraint_dir1.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
    sys.Add(constraint_dir1)

    # --- Second beam: from box1 to box2 ---
    n_elements_2 = 1 + (N_CHAINS - j)
    beam_start_2 = chrono.ChVector3d(box1_pos.x + 0.1, 0, CHAIN_START_Z * j)
    beam_end_2 = chrono.ChVector3d(box1_pos.x + 0.1 + 0.1 * (N_CHAINS - j), 0, CHAIN_START_Z * j)

    builder2 = fea.ChBuilderCableANCF()
    builder2.BuildBeam(mesh, msection_cable, n_elements_2, beam_start_2, beam_end_2)

    # Connect front of second beam to box1
    front_node2 = builder2.GetLastBeamNodes().front()
    constraint_pos2 = fea.ChLinkNodeFrame()
    constraint_pos2.Initialize(front_node2, mbox1)
    sys.Add(constraint_pos2)

    constraint_dir2 = fea.ChLinkNodeSlopeFrame()
    constraint_dir2.Initialize(front_node2, mbox1)
    constraint_dir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
    sys.Add(constraint_dir2)

    # --- Box 2 (final end body) ---
    back_node2 = builder2.GetLastBeamNodes().back()
    box2_pos = back_node2.GetPos() + chrono.ChVector3d(0.1, 0, 0)
    mbox2 = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000, True, True, None)
    mbox2.SetPos(box2_pos)
    mbox2.SetName(f"mbox_{j}_2")
    sys.Add(mbox2)
    end_bodies.append(mbox2)

    # Connect back node of second beam to box2
    constraint_pos3 = fea.ChLinkNodeFrame()
    constraint_pos3.Initialize(back_node2, mbox2)
    sys.Add(constraint_pos3)

    constraint_dir3 = fea.ChLinkNodeSlopeFrame()
    constraint_dir3.Initialize(back_node2, mbox2)
    constraint_dir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
    sys.Add(constraint_dir3)


# === FEA Visualization ===
vis_fea_A = chrono.ChVisualShapeFEA()
vis_fea_A.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_fea_A.SetColormapRange(-0.4, 0.4)
vis_fea_A.SetSmoothFaces(True)
vis_fea_A.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_fea_A)

vis_fea_B = chrono.ChVisualShapeFEA()
vis_fea_B.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_fea_B.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_fea_B.SetSymbolsThickness(0.006)
vis_fea_B.SetSymbolsScale(0.01)
vis_fea_B.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_fea_B)


# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA cables")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


# === PrintBodyPositions ===
def print_body_positions(step_time):
    """Print positions of end bodies each simulation step."""
    parts = [f"t={step_time:.4f}"]
    for i, body in enumerate(end_bodies):
        p = body.GetPos()
        parts.append(f"chain_{i}: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
    print("  ".join(parts))


# === Review-only: recording + CSV ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))


# === Main loop ===
frame = 0
while vis.Run() and sys.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        t = sys.GetChTime()
        print_body_positions(t)


        sys.DoStepDynamics(TIME_STEP)
        if sys.GetChTime() >= SIM_END:
            break


print("Simulation complete.")
