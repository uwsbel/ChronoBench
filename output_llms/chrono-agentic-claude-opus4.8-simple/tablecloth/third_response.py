import errno
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"            # frame/data output dir for the shell demo
try:
    os.mkdir(out_dir)                                                 # may already exist on re-run
except OSError as exc:                                               # the one allowed try/except (tablecloth dir guard)
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

sys = chrono.ChSystemSMC()                                           # SMC system (all FEA truths use SMC)

mesh = fea.ChMesh()                                                  # container for FEA nodes + elements
sys.Add(mesh)                                                        # remember to add the mesh to the system

# Kirchhoff isotropic shell material for the cloth sheet
density = 100                                                        # kg/m^3
E = 6e4                                                              # Young's modulus
nu = 0.0                                                             # Poisson ratio
thickness = 0.01                                                     # shell thickness
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)            # note spelling: Isothropic
material = fea.ChMaterialShellKirchhoff(melasticity)                # wrap elasticity in a shell material
material.SetDensity(density)                                        # set cloth density

L_x = 1                                                             # sheet extent in x
nsections_x = 40                                                    # subdivisions in x
L_z = 1                                                             # sheet extent in z
nsections_z = 40                                                    # subdivisions in z

# Build the 41x41 node grid in the XZ plane (y = 0)
mynodes = []                                                        # keep strong refs (used when adding elements)
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  # grid point
        mnode = fea.ChNodeFEAxyz(p)                                 # position-only shell node
        mesh.AddNode(mnode)                                        # register the node
        mynodes.append(mnode)                                     # store for stencil lookups

# Two BST triangles per cell, each with its 3 main + 3 boundary (neighbour) nodes
for iz in range(nsections_z):
    for ix in range(nsections_x):
        melementA = fea.ChElementShellBST()                        # lower triangle of the cell
        mesh.AddElement(melementA)

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]  # opposite-edge neighbour
        if ix > 0:                                                 # left neighbour exists only off the left edge
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]
        else:
            boundary_2 = None
        if iz > 0:                                                 # below neighbour exists only off the bottom edge
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1]
        else:
            boundary_3 = None

        melementA.SetNodes(                                        # 3 main + 3 boundary nodes
            mynodes[(iz) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single shell layer

        melementB = fea.ChElementShellBST()                        # upper triangle of the cell
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]        # opposite-edge neighbour
        if ix < nsections_x - 1:                                   # right neighbour off the right edge
            boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2]
        else:
            boundary_2 = None
        if iz < nsections_z - 1:                                   # above neighbour off the top edge
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix]
        else:
            boundary_3 = None

        melementB.SetNodes(                                        # 3 main + 3 boundary nodes
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single shell layer

# Fix a 30x30 corner block of nodes so the cloth hangs/drapes from there
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# Shape 1 — coloured shell surface field (with the requested visual enhancements)
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                   # mesh is a ctor arg
mvisualizeshellA.SetSmoothFaces(True)                             # smooth shaded faces
mvisualizeshellA.SetWireframe(True)                               # draw wireframe overlay
mvisualizeshellA.SetShellResolution(2)                            # subdivide each shell for display
mvisualizeshellA.SetBackfaceCull(True)                            # cull back faces
mesh.AddVisualShapeFEA(mvisualizeshellA)                          # register the surface shape

# Shape 2 — node-dot glyphs
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)                  # glyph shape on the same mesh
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)        # no scalar field on glyphs
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # dots at node positions
mvisualizeshellB.SetSymbolsThickness(0.006)                      # glyph dot size
mesh.AddVisualShapeFEA(mvisualizeshellB)                         # register the glyph shape

# Irrlicht visualization (Y-up FEA scene; Initialize first, then scene elements, NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))   # eye, look-at
vis.AddTypicalLights()

# Pardiso MKL direct solver (stiff shell matrices diverge on iterative solvers)
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(False)                            # sparsity changes as cloth deforms
sys.SetSolver(mkl_solver)

timestep = 0.001                                                # shell timestep
sys.Setup()                                                     # finalize the system before stepping
sys.Update()                                                    # update derived quantities

render_fps = 50.0                                               # untagged render cadence target
render_every = max(1, round(1.0 / (render_fps * timestep)))     # untagged render-cadence constant
mid_node = mynodes[(nsections_z // 2) * (nsections_x + 1) + nsections_x]   # a free corner node to track

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(timestep)
