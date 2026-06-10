import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory for FEA shell results
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create Chrono physical system (SMC for FEA stiff shells)
sys = chrono.ChSystemSMC()

# Create and add FEA mesh to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# Material properties for the tablecloth
density = 100                   # kg/m³ — lightweight cloth
E = 6e4                         # Young's modulus (Pa) — soft cloth
nu = 0.0                        # Poisson's ratio — isotropic
thickness = 0.01                # shell thickness (m)

# Create isotropic Kirchhoff material
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)   # isotropic Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)        # Kirchhoff shell material
material.SetDensity(density)                                # set cloth density

# Mesh dimensions — 1m × 1m tablecloth subdivided into 40×40 sections
L_x, L_z = 1, 1                    # tablecloth size (m)
nsections_x, nsections_z = 40, 40  # number of mesh subdivisions per axis

# Node monitoring variables and load tracking
nodePlotA = None    # first plot node (lower-left corner region)
nodePlotB = None    # second plot node (central region)
nodesLoad = []      # list of nodes for applying loads
load_force = chrono.ChVector3d(0, -2, 0)   # load force vector (downward N)
mnodemonitor = None     # monitored node
melementmonitor = None  # monitored element

# Interpolation reference functions (linear time ramp, used for reference tracking)
ref_X = lambda t: t * 0.1   # reference X position ramp
ref_Y = lambda t: t * 0.1   # reference Y position ramp

# Build node grid
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  # node position
        mnode = fea.ChNodeFEAxyz(p)   # xyz-only FEA node
        mesh.AddNode(mnode)
        mynodes.append(mnode)
        if iz == 0 and ix == 0:
            nodePlotA = mnode           # assign first plot node (corner)
        if iz == nsections_z // 2 and ix == nsections_x // 2:
            nodePlotB = mnode           # assign second plot node (center)
        nodesLoad.append(mnode)         # collect all nodes for load application
        if iz == 0 and ix == 1:
            mnodemonitor = mnode        # assign monitoring node

# Build BST (Belytschko-Bathe Shell Triangle) elements, two triangles per quad cell
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # --- Triangle A (lower-left, lower-right, upper-left) ---
        melementA = fea.ChElementShellBST()
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]                                         # upper-right neighbour
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None                  # upper-left neighbour
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None                  # lower-right neighbour
        melementA.SetNodes(
            mynodes[iz * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0, material)   # single isotropic layer
        mesh.AddElement(melementA)
        if iz == 0 and ix == 1:
            melementmonitor = melementA   # assign monitor element at iz==0, ix==1

        # --- Triangle B (upper-right, upper-left, lower-right) ---
        melementB = fea.ChElementShellBST()
        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]                                                          # lower-left neighbour
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None                   # lower-right+1 neighbour
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None                 # upper-left+1 neighbour
        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix],
            mynodes[iz * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0, material)   # single isotropic layer
        mesh.AddElement(melementB)

# Fix upper nodes — two nested loops to pin top rows of the mesh
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)   # fix upper node at [j,k]

# FEA visualization — surface shape for deformed shell (smooth faces, wireframe mode)
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)    # pass mesh to ctor (9.0.0)
mvisualizeshellA.SetSmoothFaces(True)               # enable smooth face shading
mvisualizeshellA.SetWireframe(True)                 # wireframe mode for the mesh
mvisualizeshellA.SetShellResolution(2)              # smoother shell surface
mesh.AddVisualShapeFEA(mvisualizeshellA)

# FEA visualization — node dot glyphs with DataType_NONE
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)    # pass mesh to ctor (9.0.0)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)    # show node dots
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)              # no scalar field
mvisualizeshellB.SetSymbolsThickness(0.006)         # dot size
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization system — Initialize FIRST, then add scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()                                                           # FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))           # AFTER Initialize
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()

# Pardiso MKL solver — required for stiff shell stiffness matrices
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)                # lock sparsity pattern for efficiency
sys.SetSolver(mkl_solver)

# Simulation parameters
timestep = 0.005    # time step (s) — adjusted from 0.001 to 0.005
sim_end = 5.0       # simulation duration (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * timestep)))   # render cadence

# Final setup — required before loop for shell elements
sys.Setup()
sys.Update()


# Simulation loop — pure Irrlicht FEA scene, no stdout output (render is the output)
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(timestep)
        if sys.GetChTime() >= sim_end:
            break
