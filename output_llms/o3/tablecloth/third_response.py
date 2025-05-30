#
#  SHELLS –  Bias Strip Triangular element (BST) –  PYTHON EXAMPLE
#
#  This script has been cleaned-up, a few bugs were fixed and, as requested,
#  the visualisation of the first FEA shape has been enhanced by
#  • enabling smooth faces
#  • drawing the wire-frame
#  • activating back-face culling
#
#  It should run with a recent official build of Project Chrono that
#  contains the FEA, Irrlicht and Pardiso-MKL modules.
#

import os
import errno

import pychrono.core      as chrono
import pychrono.irrlicht  as chronoirr
import pychrono.fea       as fea
import pychrono.pardisomkl as mkl


# -------------------------------------------------------------------------
# CREATE/VERIFY OUTPUT DIRECTORY
# -------------------------------------------------------------------------
out_dir = os.path.join(chrono.GetChronoOutputPath(), "FEA_SHELLS_BST")
if not os.path.isdir(out_dir):
    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            print("Error creating output directory:", out_dir)


# -------------------------------------------------------------------------
# CREATE PHYSICAL SYSTEM AND MESH
# -------------------------------------------------------------------------
sys  = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

# Switch–off the automatic gravity load that the mesh can optionally add
mesh.SetAutomaticGravity(False)


# -------------------------------------------------------------------------
# MATERIAL DEFINITION
# -------------------------------------------------------------------------
density   = 100.0           # [kg/m³]
E         = 6.0e4           # [Pa]
nu        = 0.0             # Poisson ratio
thickness = 0.01            # [m]

# --- corrected class-name (‘Isotropic’ is the proper spelling) -------------
melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu)
material    = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)


# -------------------------------------------------------------------------
# GEOMETRY (NODES & ELEMENTS)
# -------------------------------------------------------------------------
L_x          = 1.0
L_z          = 1.0
nsections_x  = 40
nsections_z  = 40

mynodes = []

# --- helper that returns a valid node even if the requested index is out of range
def safe_node(idx):
    idx = max(0, min(idx, len(mynodes) - 1))
    return mynodes[idx]

# ---- create nodes on a regular grid -------------------------------------------------
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        pos = chrono.ChVector3d(float(ix) * L_x / nsections_x,
                                0.0,
                                float(iz) * L_z / nsections_z)
        node = fea.ChNodeFEAxyz(pos)
        mesh.AddNode(node)
        mynodes.append(node)

# ---- create BST shell elements ------------------------------------------------------
melementmonitor = None  # will store one element just to show that we can access it later

for iz in range(nsections_z):
    for ix in range(nsections_x):

        # --- first triangle in the quad ----------------------------------------------
        elemA = fea.ChElementShellBST()
        mesh.AddElement(elemA)

        if iz == 0 and ix == 1:
            melementmonitor = elemA          # store a reference to this element

        n1 = safe_node( iz      * (nsections_x + 1) + ix     )
        n2 = safe_node( iz      * (nsections_x + 1) + ix + 1 )
        n3 = safe_node((iz + 1) * (nsections_x + 1) + ix     )

        # mid-side nodes – duplicates are acceptable at borders
        n4 = safe_node((iz + 1) * (nsections_x + 1) + ix + 1)
        n5 = safe_node((iz + 1) * (nsections_x + 1) + max(ix - 1, 0))
        n6 = safe_node(max(iz - 1, 0) * (nsections_x + 1) + ix + 1)

        elemA.SetNodes(n1, n2, n3, n4, n5, n6)
        elemA.AddLayer(thickness, 0.0, material)

        # --- second triangle in the quad --------------------------------------------
        elemB = fea.ChElementShellBST()
        mesh.AddElement(elemB)

        n1b = safe_node((iz + 1) * (nsections_x + 1) + ix + 1)
        n2b = safe_node((iz + 1) * (nsections_x + 1) + ix    )
        n3b = safe_node( iz      * (nsections_x + 1) + ix + 1)

        n4b = safe_node( iz      * (nsections_x + 1) + ix    )
        n5b = safe_node( iz      * (nsections_x + 1) + min(ix + 2, nsections_x))
        n6b = safe_node(min(iz + 2, nsections_z) * (nsections_x + 1) + ix)

        elemB.SetNodes(n1b, n2b, n3b, n4b, n5b, n6b)
        elemB.AddLayer(thickness, 0.0, material)

# ---- fix nodes (upper-left 30 × 30 patch) -------------------------------------------
for j in range(30):
    for k in range(30):
        idx = j * (nsections_x + 1) + k
        if idx < len(mynodes):
            mynodes[idx].SetFixed(True)


# -------------------------------------------------------------------------
# VISUALISATION ASSETS
# -------------------------------------------------------------------------
# NOTE: the visual shape class belongs to module ‘fea’
mvisualizeshellA = fea.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetSmoothFaces(True)       # 1.  smooth shading
mvisualizeshellA.SetWireframe(True)         # 2.  wire-frame overlay
mvisualizeshellA.SetBackfaceCull(True)      # 3.  back-face culling
mvisualizeshellA.SetShellResolution(2)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = fea.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)


# -------------------------------------------------------------------------
# IRRLICHT VISUAL SYSTEM
# -------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Shells FEA test: triangle BST elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()


# -------------------------------------------------------------------------
# SOLVER (PARDISO-MKL)
# -------------------------------------------------------------------------
solver = mkl.ChSolverPardisoMKL()
solver.LockSparsityPattern(True)
sys.SetSolver(solver)


# -------------------------------------------------------------------------
# FINALISE AND SIMULATE
# -------------------------------------------------------------------------
timestep = 0.005

sys.Setup()
sys.Update()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(timestep)