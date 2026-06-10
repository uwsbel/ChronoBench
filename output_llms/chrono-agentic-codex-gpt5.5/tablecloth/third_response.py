"""FEA tablecloth shell simulation using a ChSystemSMC system.

The model builds a rectangular Kirchhoff BST shell mesh with one edge fixed so
gravity drapes the cloth.  The primary FEA visualization shape is named
mvisualizeshellA and enables smooth faces, wireframe drawing, and backface
culling as the requested visualization behavior.
"""

import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === mesh, material, and timing values kept explicit for review
cloth_length = 1.20
cloth_width = 0.80
num_x = 10
num_z = 8
node_mass = 0.010
shell_thickness = 0.002
young_modulus = 1.0e6
poisson_ratio = 0.30
cloth_density = 500.0
time_step = 0.001
sim_end = 2.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


def node_at(nodes, ix, iz):
    """Return a cached grid node by integer cloth-grid coordinates."""
    return nodes[iz][ix]


def make_shell(nodes, i0, j0, i1, j1, i2, j2, material):
    """Create one BST triangular shell element with boundary neighbors omitted."""
    ele = fea.ChElementShellBST()
    ele.SetNodes(
        node_at(nodes, i0, j0),
        node_at(nodes, i1, j1),
        node_at(nodes, i2, j2),
        None,
        None,
        None,
    )
    ele.AddLayer(shell_thickness, 0.0, material)
    return ele


# === System & solver === SMC and Pardiso/HHT match stiff shell FEA usage
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)
# Shell FEA only: no collision surface or contact material is needed.


# === FEA cloth mesh === rectangular BST shell with the left edge clamped
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

elasticity = fea.ChElasticityKirchhoffIsothropic(young_modulus, poisson_ratio)
shell_material = fea.ChMaterialShellKirchhoff(elasticity)
shell_material.SetDensity(cloth_density)

nodes = []
dx = cloth_length / (num_x - 1)
dz = cloth_width / (num_z - 1)
x0 = -0.5 * cloth_length
z0 = -0.5 * cloth_width
for iz in range(num_z):
    row = []
    for ix in range(num_x):
        x = x0 + ix * dx
        z = z0 + iz * dz
        sag = -0.04 * math.sin(math.pi * ix / (num_x - 1))
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, 0.55 + sag, z))
        node.SetMass(node_mass)
        if ix == 0:
            node.SetFixed(True)
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)
cloth_nodes = [node for row in nodes for node in row]  # cache: flat node list reused for logging

for iz in range(num_z - 1):
    for ix in range(num_x - 1):
        mesh.AddElement(make_shell(nodes, ix, iz, ix + 1, iz, ix + 1, iz + 1, shell_material))
        mesh.AddElement(make_shell(nodes, ix, iz, ix + 1, iz + 1, ix, iz + 1, shell_material))

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
mvisualizeshellA.SetColorscaleMinMax(0.0, 0.20)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(True)
mvisualizeshellA.SetBackfaceCull(True)
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


# === Visualization === Irrlicht window initialized before camera, lights, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth FEA Shell Visualization")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.4, 1.1, 1.3), chrono.ChVector3d(0.1, 0.35, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    24,
    18,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render the shell and advance the FEA dynamics

frame = 0
free_corner = node_at(nodes, num_x - 1, num_z - 1)  # cache: tracked cloth corner
center_node = node_at(nodes, num_x // 2, num_z // 2)  # cache: tracked cloth center
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
