"""Kirchhoff shell tablecloth folding simulation.

This PyChrono FEA model uses an SMC system with a PardisoMKL direct solver,
an isotropic Kirchhoff shell material, and a triangular BST shell mesh.  The
cloth is a thin rectangular grid with one edge fixed and a mild initial crease;
gravity folds the free shell panels while Irrlicht renders the deformation.
"""

import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants: grid, material, and runtime are fixed up front ===
TIME_STEP = 0.0005
SIM_END = 0.6
RENDER_FPS = 30.0

CLOTH_WIDTH = 1.20
CLOTH_LENGTH = 1.00
NX = 18
NZ = 16
HALF_WIDTH = 0.5 * CLOTH_WIDTH
HALF_LENGTH = 0.5 * CLOTH_LENGTH
DX = CLOTH_WIDTH / (NX - 1)
DZ = CLOTH_LENGTH / (NZ - 1)

SHELL_THICKNESS = 0.005
YOUNG_MODULUS = 2.0e7
POISSON_RATIO = 0.30
DENSITY = 185.0
CREASE_AMPLITUDE = 0.06
PINNED_COLUMNS = 2


# === System & solver: SMC FEA with direct linear solves ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
timestepper.SetAlpha(-0.05)
timestepper.SetMaxIters(20)
timestepper.SetAbsTolerances(1e-8)
sys.SetTimestepper(timestepper)


# === Shell mesh: generated grid of BST Kirchhoff triangles ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
elasticity = fea.ChElasticityKirchhoffIsothropic(YOUNG_MODULUS, POISSON_RATIO)
material = fea.ChMaterialShellKirchhoff(elasticity)
material.SetDensity(DENSITY)

nodes = []
for ix in range(NX):
    col = []
    x = -HALF_WIDTH + ix * DX
    for iz in range(NZ):
        z = -HALF_LENGTH + iz * DZ
        crease = CREASE_AMPLITUDE * math.sin(math.pi * iz / (NZ - 1))
        y = 0.55 - crease * (1.0 - abs(x) / HALF_WIDTH)
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        if ix < PINNED_COLUMNS:
            node.SetFixed(True)
        mesh.AddNode(node)
        col.append(node)
    nodes.append(col)


def node_at(ix, iz):
    """Return a node or None when a BST neighbour lies outside the cloth."""
    if 0 <= ix < NX and 0 <= iz < NZ:
        return nodes[ix][iz]
    return None


elements = []  # cache: strong references keep SWIG-owned shell elements alive


def add_bst(n0, n1, n2, nb0, nb1, nb2):
    """Create one BST triangle with its opposite-edge neighbour stencil."""
    element = fea.ChElementShellBST()
    element.SetNodes(n0, n1, n2, nb0, nb1, nb2)
    element.AddLayer(SHELL_THICKNESS, 0.0, material)
    mesh.AddElement(element)
    elements.append(element)


for ix in range(NX - 1):
    for iz in range(NZ - 1):
        n00 = nodes[ix][iz]
        n10 = nodes[ix + 1][iz]
        n01 = nodes[ix][iz + 1]
        n11 = nodes[ix + 1][iz + 1]
        add_bst(n00, n10, n11, node_at(ix + 2, iz + 1), n01, node_at(ix, iz - 1))
        add_bst(n00, n11, n01, node_at(ix + 1, iz + 2), node_at(ix - 1, iz), n10)

visual = chrono.ChVisualShapeFEA(mesh)
visual.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
visual.SetShellResolution(2)
visual.SetSmoothFaces(True)
visual.SetBackfaceCull(False)
visual.SetDefaultMeshColor(chrono.ChColor(0.85, 0.18, 0.12))
mesh.AddVisualShapeFEA(visual)

sys.Add(mesh)
sys.Setup()
sys.Update()

# FEA shell: no contact material is needed because folding is driven by gravity
# and fixed edge constraints rather than collision against a rigid table.


# === Visualization: Irrlicht window created before scene nodes are added ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Kirchhoff Shell Tablecloth Folding")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(1.35, 1.25, 1.45),
    chrono.ChVector3d(0.10, 0.34, 0.00),
)
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(1.5, 3.5, 2.0), 8.0, chrono.ChColor(0.9, 0.9, 0.85))
grid_frame = chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0.0, 0),
    chrono.QuatFromAngleX(chrono.CH_PI_2),
)
vis.AddGrid(0.1, 0.1, 16, 14, grid_frame, chrono.ChColor(0.38, 0.38, 0.38))


# === Main loop: render, log review data, and advance shell dynamics ===
tip_node = nodes[-1][NZ // 2]  # cache: free-edge midpoint sampled every step
fold_node = nodes[NX // 2][NZ // 2]  # cache: center node sampled every step


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
