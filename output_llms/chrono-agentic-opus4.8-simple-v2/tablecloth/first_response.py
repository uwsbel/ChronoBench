import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"           # FEA shell output folder
try:
    os.mkdir(out_dir)                                              # create output dir if missing
except OSError as exc:
    if exc.errno != errno.EEXIST:                                 # ignore "already exists"
        print("Error creating output directory ")

sys = chrono.ChSystemSMC()                                         # SMC system for stiff shell FEA

mesh = fea.ChMesh()                                                # container for nodes + elements
sys.Add(mesh)                                                      # register mesh with the system

# Isotropic Kirchhoff shell material for the tablecloth
density = 100                                                       # kg/m^3 areal density driver
E = 6e4                                                             # Young's modulus (Pa)
nu = 0.0                                                            # Poisson ratio
thickness = 0.01                                                    # shell thickness (m)

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)           # isotropic Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)               # wrap into a shell material
material.SetDensity(density)                                        # set cloth density

# Rectangular mesh grid for the tablecloth
L_x = 1                                                             # cloth extent in X (m)
nsections_x = 40                                                    # grid divisions in X
L_z = 1                                                             # cloth extent in Z (m)
nsections_z = 40                                                    # grid divisions in Z

mynodes = []                                                        # row-major node list for stencil lookup
for iz in range(nsections_z + 1):                                  # walk the grid in Z
    for ix in range(nsections_x + 1):                              # walk the grid in X
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  # node position (Y-up flat sheet)
        mnode = fea.ChNodeFEAxyz(p)                                # xyz position node (no rotation DOF for BST)
        mesh.AddNode(mnode)                                        # add node to mesh
        mynodes.append(mnode)                                     # keep strong ref + index

# Build two BST triangles per grid cell, each with its 3-node neighbour stencil
for iz in range(nsections_z):                                      # cell row
    for ix in range(nsections_x):                                  # cell column
        melementA = fea.ChElementShellBST()                       # first triangle of the cell
        mesh.AddElement(melementA)                                # register element

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]   # opposite-corner neighbour
        if ix > 0:                                                # left neighbour exists?
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]
        else:
            boundary_2 = None                                     # edge: no neighbour
        if iz > 0:                                                # below neighbour exists?
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1]
        else:
            boundary_3 = None                                     # edge: no neighbour

        melementA.SetNodes(                                       # 3 main nodes + 3 boundary neighbours
            mynodes[(iz) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single Kirchhoff layer

        melementB = fea.ChElementShellBST()                      # second triangle of the cell
        mesh.AddElement(melementB)                               # register element

        boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]      # opposite-corner neighbour
        if ix < nsections_x - 1:                                 # right neighbour exists?
            boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2]
        else:
            boundary_2 = None                                    # edge: no neighbour
        if iz < nsections_z - 1:                                 # above neighbour exists?
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix]
        else:
            boundary_3 = None                                    # edge: no neighbour

        melementB.SetNodes(                                      # 3 main nodes + 3 boundary neighbours
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[(iz) * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single Kirchhoff layer

# Pin a 30x30 corner block so the rest of the cloth folds under gravity
for j in range(30):                                               # fixed-node block rows
    for k in range(30):                                           # fixed-node block columns
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)         # clamp this node

# FEA visualization — shaded shell surface + node dot glyphs
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                  # surface shell shape (mesh is ctor arg)
mvisualizeshellA.SetShellResolution(2)                           # subdivide each shell face for smooth render
mesh.AddVisualShapeFEA(mvisualizeshellA)                         # register surface shape

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)                 # glyph shape for node markers
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)         # no scalar field on glyphs
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # render nodes as dots
mvisualizeshellB.SetSymbolsThickness(0.006)                     # dot size
mesh.AddVisualShapeFEA(mvisualizeshellB)                         # register glyph shape

# Irrlicht visualization window (Initialize first, then scene elements, no grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)                                            # bind the physics system
vis.SetWindowSize(1024, 768)                                    # window resolution
vis.SetWindowTitle('Shells FEA test: triangle BST elements')   # window title
vis.Initialize()                                               # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # logo overlay
vis.AddSkyBox()                                                 # sky box backdrop
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))  # eye, target
vis.AddTypicalLights()                                         # standard two-light rig

# PardisoMKL direct solver for the stiff shell stiffness matrix
mkl_solver = mkl.ChSolverPardisoMKL()                          # MKL Pardiso direct solver
mkl_solver.LockSparsityPattern(True)                           # reuse the sparsity pattern each step
sys.SetSolver(mkl_solver)                                      # install the solver

timestep = 0.005                                               # shell integration step (s)
sys.Setup()                                                    # allocate state for the mesh
sys.Update()                                                   # assemble system before stepping

sim_end = 5.0                                                  # stop time (s)
render_fps = 50.0                                             # review frame rate
render_every = max(1, round(1.0 / (render_fps * timestep)))  # physics steps per rendered frame

# Simulation loop — the live render is the output (pure-Irrlicht FEA: no stdout)
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(timestep)
        if sys.GetChTime() >= sim_end:
            break
