import os
import pychrono as chrono                                            # core
import pychrono.fea as fea                                           # finite elements
import pychrono.pardisomkl as mkl                                    # direct sparse solver
import pychrono.irrlicht as chronoirr                                # visualization

out_dir = "tablecloth_out"                                          # FEA-shell output dir
try:
    os.mkdir(out_dir)                                               # create once
except OSError:
    print("Directory already exists or cannot be created: " + out_dir)

sys = chrono.ChSystemSMC()                                          # SMC system for stiff shells
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # Y-up world, g down

mesh = fea.ChMesh()                                                 # FEA mesh container
mesh.SetAutomaticGravity(True)                                      # apply gravity to the cloth nodes

E = 0.01e9                                                          # Young's modulus (Pa), soft cloth
nu = 0.3                                                            # Poisson ratio
thickness = 0.01                                                    # shell thickness (m)
density = 200                                                       # areal density driver (kg/m^3)

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)           # isotropic Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)               # shell material from elasticity
material.SetDensity(density)                                        # cloth density

nx = 20                                                             # nodes along X
ny = 20                                                             # nodes along Z
L = 1.0                                                             # tablecloth side length (m)
dx = L / (nx - 1)                                                   # node spacing X
dy = L / (ny - 1)                                                   # node spacing Z
y0 = 1.0                                                            # initial drop height (m)

nodes = [[None] * ny for _ in range(nx)]                           # grid of node handles
for ix in range(nx):                                               # build node grid
    for iy in range(ny):
        x = ix * dx - L / 2                                         # center the cloth on origin
        z = iy * dy - L / 2
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y0, z))       # planar cloth at height y0
        mesh.AddNode(node)                                         # register node (mass from shell density)
        nodes[ix][iy] = node                                       # keep handle

def nb(ix, iy):                                                   # neighbour node (None at the cloth border -> free edge)
    if ix < 0 or iy < 0 or ix >= nx or iy >= ny:                 # off the grid
        return None
    return nodes[ix][iy]

for ix in range(nx - 1):                                           # build two BST triangles per grid cell
    for iy in range(ny - 1):
        n00 = nodes[ix][iy]                                        # cell corner nodes
        n10 = nodes[ix + 1][iy]
        n01 = nodes[ix][iy + 1]
        n11 = nodes[ix + 1][iy + 1]

        ele_a = fea.ChElementShellBST()                           # triangle A: (n00, n10, n11), diagonal n00-n11
        ele_a.SetNodes(n00, n10, n11,                             # 3 main nodes
                       nb(ix + 2, iy + 1),                        # boundary opposite edge n10-n11 (right cell apex)
                       n01,                                       # boundary opposite edge n11-n00 (this cell's other apex)
                       nb(ix + 1, iy - 1))                        # boundary opposite edge n00-n10 (cell below)
        ele_a.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)   # single isotropic Kirchhoff layer
        mesh.AddElement(ele_a)

        ele_b = fea.ChElementShellBST()                           # triangle B: (n00, n11, n01)
        ele_b.SetNodes(n00, n11, n01,                             # 3 main nodes
                       nb(ix, iy + 2),                            # boundary opposite edge n11-n01 (cell above)
                       nb(ix - 1, iy + 1),                        # boundary opposite edge n01-n00 (left cell)
                       n10)                                       # boundary opposite edge n00-n11 (this cell's other apex)
        ele_b.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)
        mesh.AddElement(ele_b)

sys.Add(mesh)                                                      # register the mesh (mandatory)

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact present -> Bullet collision

contact_mat = chrono.ChContactMaterialSMC()                       # cloth-table contact material
contact_mat.SetYoungModulus(2e5)
contact_mat.SetFriction(0.3)

table = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.3, 0.1, 1000, True, True, contact_mat)  # round table the cloth folds over
table.SetPos(chrono.ChVector3d(0, 0.45, 0))                       # top just below the cloth
table.SetFixed(True)                                              # rigid support
sys.Add(table)

contact_surf = fea.ChContactSurfaceNodeCloud(contact_mat, mesh)   # node-cloud contact for the cloth
contact_surf.AddAllNodes(0.01)                                    # collision radius per node
mesh.AddContactSurface(contact_surf)

vis_surface = chrono.ChVisualShapeFEA(mesh)                       # coloured deformed surface
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)  # speed field
vis_surface.SetColorscaleMinMax(0.0, 1.0)                         # speed colour range
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
vis_surface.SetShellResolution(2)                                 # subdivide shells for display
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                         # node markers
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.004)
mesh.AddVisualShapeFEA(vis_glyph)

sys.SetSolver(mkl.ChSolverPardisoMKL())                           # Pardiso MKL direct solver
sys.Setup()                                                       # finalize DOFs (shells require it)
sys.Update()                                                      # propagate initial state

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht window
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                 # Y-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth folding (Kirchhoff shells)")
vis.Initialize()                                                  # device first, scene after
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 1.4, -1.8), chrono.ChVector3d(0, 0.4, 0))
vis.AddTypicalLights()

time_step = 2e-4                                                  # small step for stable stiff-shell integration
sim_end = 2.0                                                     # simulate the drop + drape
render_fps = 50.0                                                 # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant
center = nodes[nx // 2][ny // 2]                                 # mid node tracked for the plot mirror
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
