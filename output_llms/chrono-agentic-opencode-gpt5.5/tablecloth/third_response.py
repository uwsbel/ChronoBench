"""Kirchhoff BST tablecloth simulation using ChSystemSMC.

The model builds a square finite-element shell cloth suspended from one edge and
falling under Y-up gravity. The primary objective is to show the shell surface
with smooth faces, wireframe mode, and backface culling enabled on the shell
visualization object named mvisualizeshellA.
"""

# === Imports === standard PyChrono modules for FEA and Irrlicht
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === cloth dimensions and precomputed loop cadence
CLOTH_SIZE_X = 1.2
CLOTH_SIZE_Z = 1.2
CLOTH_Y = 1.0
NODE_COUNT_X = 9
NODE_COUNT_Z = 9
THICKNESS = 0.006
DENSITY = 650.0
YOUNG_MODULUS = 1.2e6
POISSON_RATIO = 0.30
TIME_STEP = 0.001
SIM_END = 2.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


def node_at(nodes, ix, iz):
    """Return a cached grid node by integer cloth coordinates."""
    return nodes[ix][iz]


def triangle_neighbor(nodes, tri, edge):
    """Find the opposite BST neighbour node for one triangle edge."""
    edge_nodes = {tri[edge], tri[(edge + 1) % 3]}
    for other in TRIANGLES:
        if other == tri:
            continue
        if edge_nodes.issubset(set(other)):
            for candidate in other:
                if candidate not in edge_nodes:
                    ix, iz = candidate
                    return node_at(nodes, ix, iz)
    return None


# === System & Solver === FEA shells use SMC with a direct Pardiso solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA Mesh === rectangular tablecloth grid with one fixed edge
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

melasticity = fea.ChElasticityKirchhoffIsothropic(YOUNG_MODULUS, POISSON_RATIO)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(DENSITY)

dx = CLOTH_SIZE_X / (NODE_COUNT_X - 1)
dz = CLOTH_SIZE_Z / (NODE_COUNT_Z - 1)
nodes = []
for ix in range(NODE_COUNT_X):
    column = []
    for iz in range(NODE_COUNT_Z):
        x = ix * dx - CLOTH_SIZE_X / 2.0
        z = iz * dz - CLOTH_SIZE_Z / 2.0
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, CLOTH_Y, z))
        node.SetMass(0.01)
        if iz == NODE_COUNT_Z - 1:
            node.SetFixed(True)
        mesh.AddNode(node)
        column.append(node)
    nodes.append(column)

TRIANGLES = []
for ix in range(NODE_COUNT_X - 1):
    for iz in range(NODE_COUNT_Z - 1):
        TRIANGLES.append(((ix, iz), (ix + 1, iz), (ix + 1, iz + 1)))
        TRIANGLES.append(((ix, iz), (ix + 1, iz + 1), (ix, iz + 1)))

for tri in TRIANGLES:
    n0 = node_at(nodes, *tri[0])
    n1 = node_at(nodes, *tri[1])
    n2 = node_at(nodes, *tri[2])
    nb0 = triangle_neighbor(nodes, tri, 0)
    nb1 = triangle_neighbor(nodes, tri, 1)
    nb2 = triangle_neighbor(nodes, tri, 2)
    element = fea.ChElementShellBST()
    element.SetNodes(n0, n1, n2, nb0, nb1, nb2)
    element.AddLayer(THICKNESS, 0.0, material)
    mesh.AddElement(element)

sys.Add(mesh)
mesh_nodes = mesh.GetNumNodes()  # cache: mesh size reused in console/report output
mesh_elements = mesh.GetNumElements()  # cache: mesh size reused in console/report output
print("Tablecloth mesh nodes:", mesh_nodes, "elements:", mesh_elements)


# === Visualization Assets === shell surface with requested smooth/wire/backface settings
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
mvisualizeshellA.SetColorscaleMinMax(0.0, 0.35)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.005)
mvisualizeshellB.SetSymbolsScale(0.006)
mvisualizeshellB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

sys.Setup()
sys.Update()


# === Irrlicht Window === Y-up camera, sky, lights, and a reference grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("BST tablecloth shell visualization")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.7, 1.1, 1.7), chrono.ChVector3d(0.0, 0.55, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.2,
    0.2,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)

# === Main Loop === render once per frame and advance shell dynamics in batches
frame = 0
free_corner = node_at(nodes, 0, 0)  # cache: tracked free cloth corner
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    raise RuntimeError("tablecloth FEA solve failed") from exc
finally:
    pass
