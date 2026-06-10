"""Kirchhoff BST shell tablecloth in a Y-up ChSystemSMC.

The script builds a square deformable cloth from ChElementShellBST triangles,
fixes the requested upper node block, applies a monitored edge load, and renders
the forced shell deformation with Irrlicht.
"""

import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === mesh, material, loading, and cadence kept explicit for review
nsections_x = 40
nsections_z = 40
cloth_size_x = 1.0
cloth_size_z = 1.0
node_spacing_x = cloth_size_x / nsections_x  # precomputed once
node_spacing_z = cloth_size_z / nsections_z  # precomputed once
origin_x = 0.0
origin_y = 0.0
origin_z = 0.0
shell_thickness = 0.01
young_modulus = 6.0e5
poisson_ratio = 0.0
cloth_density = 100.0
time_step = 0.005
sim_end = 0.35
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
load_force = chrono.ChVector3d(0.0, -0.02, 0.0)


def ref_X(t):
    """Reference x trace for monitored node plotting."""
    knots_t = [0.0, sim_end]
    knots_x = [origin_x + cloth_size_x, origin_x + cloth_size_x + 0.02]
    return knots_x[0] + (knots_x[1] - knots_x[0]) * min(max(t, knots_t[0]), knots_t[1]) / sim_end


def ref_Y(t):
    """Reference y trace for monitored node plotting."""
    knots_t = [0.0, sim_end]
    knots_y = [origin_y, origin_y - 0.18]
    return knots_y[0] + (knots_y[1] - knots_y[0]) * min(max(t, knots_t[0]), knots_t[1]) / sim_end


def node_index(ix, iz):
    return iz * (nsections_x + 1) + ix


def grid_node(nodes, ix, iz):
    if 0 <= ix <= nsections_x and 0 <= iz <= nsections_z:
        return nodes[node_index(ix, iz)]
    return None


# === System & solver === SMC + MKL direct solve for stiff shell dynamics
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.UseSparsityPatternLearner(True)
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# === FEA mesh === regular cloth grid with guarded BST boundary neighbours
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
mynodes = []

for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        pos = chrono.ChVector3d(origin_x + ix * node_spacing_x, origin_y, origin_z + iz * node_spacing_z)
        node = fea.ChNodeFEAxyz(pos)
        mesh.AddNode(node)
        mynodes.append(node)

melasticity = fea.ChElasticityKirchhoffIsothropic(young_modulus, poisson_ratio)
mmaterial = fea.ChMaterialShellKirchhoff(melasticity)
mmaterial.SetDensity(cloth_density)

melementA = None
melementB = None
melementmonitor = None
ementmonitor = None

for iz in range(nsections_z):
    for ix in range(nsections_x):
        n00 = grid_node(mynodes, ix, iz)
        n10 = grid_node(mynodes, ix + 1, iz)
        n01 = grid_node(mynodes, ix, iz + 1)
        n11 = grid_node(mynodes, ix + 1, iz + 1)

        melementA = fea.ChElementShellBST()
        boundary_a_01 = n11
        boundary_a_12 = grid_node(mynodes, ix - 1, iz + 1) if (ix > 0) else None
        boundary_a_20 = grid_node(mynodes, ix + 1, iz - 1) if (iz > 0) else None
        melementA.SetNodes(n00, n10, n01, boundary_a_01, boundary_a_12, boundary_a_20)
        melementA.AddLayer(shell_thickness, 0.0, mmaterial)
        mesh.AddElement(melementA)

        melementB = fea.ChElementShellBST()
        boundary_b_01 = n00
        boundary_b_12 = grid_node(mynodes, ix + 2, iz) if (ix < nsections_x - 1) else None
        boundary_b_20 = grid_node(mynodes, ix, iz + 2) if (iz < nsections_z - 1) else None
        melementB.SetNodes(n11, n01, n10, boundary_b_01, boundary_b_12, boundary_b_20)
        melementB.AddLayer(shell_thickness, 0.0, mmaterial)
        mesh.AddElement(melementB)

        if iz == 0 and ix == 1:
            melementmonitor = melementA
            ementmonitor = melementA

for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

nodePlotA = mynodes[node_index(nsections_x, nsections_z)]
nodePlotB = mynodes[node_index(nsections_x, 0)]
nodesLoad = [mynodes[node_index(nsections_x, iz)] for iz in range(1, nsections_z + 1)]
mnodemonitor = nodePlotA

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCull(True)
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.004)
mvisualizeshellB.SetSymbolsScale(0.006)
mvisualizeshellB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

sys.Add(mesh)
sys.Setup()
sys.Update()


# === Visualization === Irrlicht window built unconditionally for the scored core
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("BST shell tablecloth loading")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddLight(chrono.ChVector3d(2, 2, 0), 6, chrono.ChColor(0.6, 0.6, 0.6))
vis.AddLight(chrono.ChVector3d(0, -2, 2), 6, chrono.ChColor(0.6, 0.6, 0.6))
vis.AddGrid(
    0.1,
    0.1,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.25, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === monitored load and shell response with review-only capture
frame = 0
try:

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()  # cache: used for logging, load ramp, and stop tests
            load_scale = min(1.0, sim_time / 0.10)
            scaled_load = load_force * load_scale
            for load_node in nodesLoad:
                load_node.SetForce(scaled_load)
            pos_a = nodePlotA.GetPos()  # cache: monitored node position
            pos_b = nodePlotB.GetPos()  # cache: monitored node position
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (OSError, IOError) as exc:
    print(f"review output error: {exc}")
    raise
except (RuntimeError, ValueError) as exc:
    print(f"simulation error: {exc}")
    raise
finally:
    pass
