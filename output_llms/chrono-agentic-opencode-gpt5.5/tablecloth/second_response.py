"""Kirchhoff shell tablecloth simulation in a Y-up ChSystemSMC.

The model builds a square cloth from BST shell elements, fixes a corner block of
nodes, applies a constant load to the opposite edge, and monitors selected nodes
and elements while the cloth deforms under load and gravity.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === physics and mesh settings kept explicit for reproducibility
time_step = 0.005
sim_end = 0.75
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

nsections_x = 30
nsections_z = 30
cloth_size_x = 1.2
cloth_size_z = 1.2
dx = cloth_size_x / nsections_x  # precomputed once
dz = cloth_size_z / nsections_z  # precomputed once
node_mass = 0.004
shell_thickness = 0.002
young_modulus = 2.0e6
poisson_ratio = 0.30
shell_density = 450.0
load_force = chrono.ChVector3d(0.0, -0.12, 0.0)


# === Helpers === compact interpolation and indexing utilities used by the mesh
def ref_X(u):
    return -0.5 * cloth_size_x + cloth_size_x * u


def ref_Y(v):
    return -0.5 * cloth_size_z + cloth_size_z * v


def node_index(ix, iz):
    return iz * (nsections_x + 1) + ix


def safe_node(mynodes, ix, iz):
    if 0 <= ix <= nsections_x and 0 <= iz <= nsections_z:
        return mynodes[node_index(ix, iz)]
    return None


# === System & solver === SMC FEA shell with direct MKL solver and HHT timestepper
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.UseSparsityPatternLearner(True)
mkl_solver.LockSparsityPattern(True)
mkl_solver.SetVerbose(False)
sys.SetSolver(mkl_solver)

timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Cloth mesh === BST shell grid with conditional boundary-neighbour checks
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

melasticity = fea.ChElasticityKirchhoffIsothropic(young_modulus, poisson_ratio)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(shell_density)

mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        x = ref_X(ix / nsections_x)
        z = ref_Y(iz / nsections_z)
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, 0.0, z))
        node.SetMass(node_mass)
        mesh.AddNode(node)
        mynodes.append(node)

nodesLoad = []
for iz in range(nsections_z + 1):
    nodesLoad.append(mynodes[node_index(nsections_x, iz)])

nodePlotA = mynodes[node_index(nsections_x, nsections_z)]
nodePlotB = mynodes[node_index(nsections_x, nsections_z // 2)]
mnodemonitor = nodePlotA
melementmonitor = None
ementmonitor = None

for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

for iz in range(nsections_z):
    for ix in range(nsections_x):
        n00 = safe_node(mynodes, ix, iz)
        n10 = safe_node(mynodes, ix + 1, iz)
        n01 = safe_node(mynodes, ix, iz + 1)
        n11 = safe_node(mynodes, ix + 1, iz + 1)

        # Boundary neighbours are guarded by ix/iz checks to avoid invalid indices.
        b_left = safe_node(mynodes, ix - 1, iz) if ix > 0 else None
        b_down = safe_node(mynodes, ix, iz - 1) if iz > 0 else None
        b_diag = safe_node(mynodes, ix + 1, iz + 1)
        melementA = fea.ChElementShellBST()
        melementA.SetNodes(n00, n10, n01, b_diag, b_left, b_down)
        melementA.AddLayer(shell_thickness, 0.0, material)
        mesh.AddElement(melementA)

        b_right = safe_node(mynodes, ix + 2, iz + 1) if ix < nsections_x - 1 else None
        b_up = safe_node(mynodes, ix + 1, iz + 2) if iz < nsections_z - 1 else None
        b_diag2 = safe_node(mynodes, ix, iz)
        melementB = fea.ChElementShellBST()
        melementB.SetNodes(n11, n01, n10, b_diag2, b_right, b_up)
        melementB.AddLayer(shell_thickness, 0.0, material)
        mesh.AddElement(melementB)

        if iz == 0 and ix == 1:
            melementmonitor = melementA
            ementmonitor = melementA

for node in nodesLoad:
    node.SetForce(load_force)

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mvisualizeshellA.SetColorscaleMinMax(0.0, 1.5)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCull(False)
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.004)
mvisualizeshellB.SetSymbolsScale(0.005)
mvisualizeshellB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

sys.Add(mesh)
sys.Setup()
sys.Update()

mesh_ref = mesh  # cache: keep strong reference to the FEA mesh for the run
node_a = nodePlotA  # cache: monitored plotting node reused every step
node_b = nodePlotB  # cache: monitored plotting node reused every step
monitor_node = mnodemonitor  # cache: named monitor node reused every step


# === Visualization === Irrlicht window initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("BST shell tablecloth")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 1.0, 1.8), chrono.ChVector3d(0.0, -0.1, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.15, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render-cadence review run with per-step FEA integration
try:

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === assemble review artifacts only when recording is enabled
