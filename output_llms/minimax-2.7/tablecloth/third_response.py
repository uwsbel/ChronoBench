import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# ----------------------------------------------------------------------
# Output directory
# ----------------------------------------------------------------------
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

# ----------------------------------------------------------------------
# Create a Chrono Engine physical system
# ----------------------------------------------------------------------
sys = chrono.ChSystemSMC()

# ----------------------------------------------------------------------
# Create a mesh (container for FEA elements and nodes)
# ----------------------------------------------------------------------
mesh = fea.ChMesh()
sys.Add(mesh)

# Disable gravity for the FEA mesh
mesh.SetAutomaticGravity(False)

# ----------------------------------------------------------------------
# Optional – nodes used only for plotting / load tracking
# ----------------------------------------------------------------------
nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = []

# ----------------------------------------------------------------------
# Interpolation functions (not used but kept for reference)
# ----------------------------------------------------------------------
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# ----------------------------------------------------------------------
# Load‑force vector (not used but kept for reference)
# ----------------------------------------------------------------------
load_force = chrono.ChVector3d()

# ----------------------------------------------------------------------
# Monitoring nodes / elements
# ----------------------------------------------------------------------
mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = fea.ChElementShellBST()   # will be overwritten later

# ----------------------------------------------------------------------
# Main build‑section (always executed because of `if True`)
# ----------------------------------------------------------------------
if True:
    # ---- Material properties ------------------------------------------------
    density = 100.0                # kg/m³
    E       = 6e4                  # Pa
    nu      = 0.0
    thickness = 0.01               # m

    # Isotropic Kirchhoff elasticity for shells
    melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)

    # Wrapper material for a single shell layer
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)

    # ---- Mesh geometry -----------------------------------------------------
    L_x = 1.0
    nsections_x = 40
    L_z = 1.0
    nsections_z = 40

    mynodes = []

    # Create the node grid
    for iz in range(nsections_z + 1):
        for ix in range(nsections_x + 1):
            p = chrono.ChVector3d(ix * (L_x / nsections_x),
                                 0.0,
                                 iz * (L_z / nsections_z))
            mnode = fea.ChNodeFEAxyz(p)
            mesh.AddNode(mnode)
            mynodes.append(mnode)

    # ---- Elements -----------------------------------------------------------
    for iz in range(nsections_z):
        for ix in range(nsections_x):
            # ----- First triangle (A) -----
            melementA = fea.ChElementShellBST()
            mesh.AddElement(melementA)

            # Keep a reference to the element we want to monitor
            if iz == 0 and ix == 1:
                melementmonitor = melementA   # fixed the earlier typo

            # Boundary nodes for element A
            bA1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
            bA2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
            bA3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None

            melementA.SetNodes(
                mynodes[iz * (nsections_x + 1) + ix],
                mynodes[iz * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                bA1, bA2, bA3)

            melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

            # ----- Second triangle (B) -----
            melementB = fea.ChElementShellBST()
            mesh.AddElement(melementB)

            # Boundary nodes for element B
            bB1 = mynodes[iz * (nsections_x + 1) + ix]
            bB2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
            bB3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None

            melementB.SetNodes(
                mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                mynodes[iz * (nsections_x + 1) + ix + 1],
                bB1, bB2, bB3)

            melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

    # ---- Fix a portion of the nodes (here: lower‑left 30×30 block) -------
    for j in range(30):
        for k in range(30):
            mynodes[j * (nsections_x + 1) + k].SetFixed(True)

    # ----------------------------------------------------------------------
    # Visualisation – shell A (smooth, wireframe, back‑face culling)
    # ----------------------------------------------------------------------
    mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellA.SetSmoothFaces(True)          # <-- enabled
    mvisualizeshellA.SetWireframe(True)             # <-- enabled
    mvisualizeshellA.SetShellResolution(2)        # set visualisation resolution
    mvisualizeshellA.SetBackfaceCull(True)         # <-- enabled
    mesh.AddVisualShapeFEA(mvisualizeshellA)

    # ----------------------------------------------------------------------
    # Visualisation – FEM data (node dots)
    # ----------------------------------------------------------------------
    mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    mvisualizeshellB.SetSymbolsThickness(0.006)
    mesh.AddVisualShapeFEA(mvisualizeshellB)

# ----------------------------------------------------------------------
# Irrlicht visualisation setup
# ----------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3),
              chrono.ChVector3d(.5, -.3, .5))
vis.AddTypicalLights()

# ----------------------------------------------------------------------
# Linear solver – PardisoMKL (sparse‑matrix solver)
# ----------------------------------------------------------------------
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# ----------------------------------------------------------------------
# Time step
# ----------------------------------------------------------------------
timestep = 0.005

# ----------------------------------------------------------------------
# System preparation
# ----------------------------------------------------------------------
sys.Setup()
sys.Update()

# ----------------------------------------------------------------------
# Optional – interpolation records (kept for reference)
# ----------------------------------------------------------------------
rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()

# ----------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(timestep)