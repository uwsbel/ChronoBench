"""Flexible tablecloth folding with Kirchhoff BST shell elements.

This self-contained PyChrono FEA simulation uses a Y-up ChSystemSMC, an
isotropic Kirchhoff shell material, a rectangular node grid, and triangular
BST shell elements. One cloth edge is fixed while gravity bends the remaining
shell surface downward, producing a folding tablecloth motion visualized with
Irrlicht and solved with PardisoMKL.
"""

import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === named parameters define the cloth and time integration
time_step = 0.001
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

cloth_length = 1.2
cloth_width = 0.8
num_x = 18
num_z = 12
node_dx = cloth_length / (num_x - 1)  # precomputed once
node_dz = cloth_width / (num_z - 1)  # precomputed once
cloth_y = 0.35
thickness = 0.002
density = 700.0
young_modulus = 0.8e6
poisson_ratio = 0.3
rayleigh_damping = 0.03


# === System & solver === ChSystemSMC with Y-up gravity and direct FEA solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA mesh === rectangular tablecloth grid made from Kirchhoff BST shells
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

melasticity = fea.ChElasticityKirchhoffIsothropic(young_modulus, poisson_ratio)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

nodes = []
for ix in range(num_x):
    row = []
    x = -0.5 * cloth_length + ix * node_dx
    for iz in range(num_z):
        z = -0.5 * cloth_width + iz * node_dz
        y = cloth_y + 0.06 * max(0.0, x) * max(0.0, x)
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))
        if ix == 0:
            node.SetFixed(True)
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)

triangles = []
for ix in range(num_x - 1):
    for iz in range(num_z - 1):
        n00 = nodes[ix][iz]
        n10 = nodes[ix + 1][iz]
        n01 = nodes[ix][iz + 1]
        n11 = nodes[ix + 1][iz + 1]
        triangles.append((n00, n10, n11))
        triangles.append((n00, n11, n01))

edge_to_opposites = {}
for tri in triangles:
    a, b, c = tri
    for edge, opposite in (((b, c), a), ((c, a), b), ((a, b), c)):
        key = tuple(sorted((id(edge[0]), id(edge[1]))))
        edge_to_opposites.setdefault(key, []).append(opposite)


def neighbor_for(edge, own_opposite):
    """Return the adjacent triangle node across an edge, or None at boundaries."""
    key = tuple(sorted((id(edge[0]), id(edge[1]))))
    for candidate in edge_to_opposites[key]:
        if candidate is not own_opposite:
            return candidate
    return None


for n0, n1, n2 in triangles:
    boundary_1 = neighbor_for((n1, n2), n0)
    boundary_2 = neighbor_for((n2, n0), n1)
    boundary_3 = neighbor_for((n0, n1), n2)
    element = fea.ChElementShellBST()
    element.SetNodes(n0, n1, n2, boundary_1, boundary_2, boundary_3)
    element.AddLayer(thickness, 0.0, material)
    mesh.AddElement(element)

vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
vis_surface.SetShellResolution(2)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.004)
vis_glyph.SetSymbolsScale(0.006)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)
sys.Setup()
sys.Update()

cloth_nodes = nodes  # cache: node grid reused for diagnostics in the loop
free_corner = nodes[-1][-1]  # cache: tracked cloth corner for review diagnostics
center_node = nodes[num_x // 2][num_z // 2]  # cache: tracked center node


# === Visualization === Irrlicht window with camera and lights added after Initialize
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Kirchhoff shell tablecloth folding")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.1, 0.9, 1.35), chrono.ChVector3d(0.05, 0.1, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    18,
    14,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.02, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at frame rate while stepping shell dynamics at 1 ms
frame = 0
try:
    try:
        while vis.Run() and sys.GetChTime() < sim_end:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1
            for _ in range(render_every):
                sim_time = sys.GetChTime()  # cache: used by logging and stop checks
                sys.DoStepDynamics(time_step)
                if sys.GetChTime() >= sim_end:
                    break
    except (RuntimeError, ValueError) as exc:
        traceback.print_exc()
        raise
finally:
    pass
