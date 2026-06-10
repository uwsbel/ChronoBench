"""
Tablecloth simulation using PyChrono FEA shell elements (ChElementShellBST).

Models the folding/draping of a tablecloth as a rectangular grid of
Kirchhoff BST shell elements with an isotropic Kirchhoff material.
A block of nodes in one corner is fixed; the rest drape under gravity.

System: ChSystemSMC
Solver: Pardiso MKL (stiff shells, sparsity pattern locked)
Timestepper: default (implicit — stiff shell scenes use the system default)
World: Y-up (FEA convention), gravity = (0, -9.81, 0)
Expected behavior: cloth folds downward from the fixed corner block.
"""

# === Imports ===
import os
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Named constants ===
# Cloth geometry
L_X = 1.0            # cloth span along X (m)
L_Z = 1.0            # cloth span along Z (m)
NSECTIONS_X = 40     # divisions along X (matches reference demo)
NSECTIONS_Z = 40     # divisions along Z

# Material properties (isotropic Kirchhoff — reference values)
THICKNESS = 0.01     # shell thickness (m)
DENSITY = 100.0      # kg/m³
E_MOD = 6e4          # Young's modulus (Pa)
NU = 0.0             # Poisson's ratio (reference uses 0)

# Simulation timing
TIME_STEP = 0.005    # physics timestep (s) — reference value
SIM_END = 5.0        # simulation end time (s)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Camera (Y-up scene)
CAM_EYE = chrono.ChVector3d(1.0, 0.3, 1.3)
CAM_TARGET = chrono.ChVector3d(0.5, -0.3, 0.5)

# === System & solver ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up FEA convention

# Pardiso MKL with locked sparsity pattern for efficiency
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# === FEA mesh — tablecloth shell grid ===
mesh = fea.ChMesh()
sys.Add(mesh)  # add mesh before building nodes/elements

# Isotropic Kirchhoff material — note spelling: Isothropic
melasticity = fea.ChElasticityKirchhoffIsothropic(E_MOD, NU)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(DENSITY)

# Build flat node array: (NSECTIONS_Z+1) * (NSECTIONS_X+1) nodes
# Layout: nodes[iz * (NSECTIONS_X+1) + ix], in XZ-plane at Y=0
mynodes = []  # cache: keep Python references to prevent SWIG GC
for iz in range(NSECTIONS_Z + 1):
    for ix in range(NSECTIONS_X + 1):
        p = chrono.ChVector3d(ix * (L_X / NSECTIONS_X), 0.0, iz * (L_Z / NSECTIONS_Z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Build BST shell elements — 2 triangles per quad
# BST stencil for element A (lower-left triangle: iz,ix | iz,ix+1 | iz+1,ix):
#   boundary_1 = (iz+1)*(nx+1) + ix+1   (always present for interior)
#   boundary_2 = (iz+1)*(nx+1) + ix-1   (None at left boundary ix==0)
#   boundary_3 = (iz-1)*(nx+1) + ix+1   (None at bottom boundary iz==0)
# BST stencil for element B (upper-right triangle: iz+1,ix+1 | iz+1,ix | iz,ix+1):
#   boundary_1 = iz*(nx+1) + ix          (always present)
#   boundary_2 = iz*(nx+1) + ix+2        (None at right boundary ix==nx-1)
#   boundary_3 = (iz+2)*(nx+1) + ix      (None at top boundary iz==nz-1)
NX1 = NSECTIONS_X + 1  # precomputed stride, used in element stencil below

elements = []  # cache: keep references to prevent premature GC
for iz in range(NSECTIONS_Z):
    for ix in range(NSECTIONS_X):
        # --- Element A: lower-left triangle ---
        melementA = fea.ChElementShellBST()
        mesh.AddElement(melementA)

        b1 = mynodes[(iz + 1) * NX1 + ix + 1]
        b2 = mynodes[(iz + 1) * NX1 + ix - 1] if ix > 0 else None
        b3 = mynodes[(iz - 1) * NX1 + ix + 1] if iz > 0 else None

        melementA.SetNodes(
            mynodes[iz * NX1 + ix],
            mynodes[iz * NX1 + ix + 1],
            mynodes[(iz + 1) * NX1 + ix],
            b1, b2, b3
        )
        melementA.AddLayer(THICKNESS, 0.0, material)
        elements.append(melementA)

        # --- Element B: upper-right triangle ---
        melementB = fea.ChElementShellBST()
        mesh.AddElement(melementB)

        b4 = mynodes[iz * NX1 + ix]
        b5 = mynodes[iz * NX1 + ix + 2] if ix < NSECTIONS_X - 1 else None
        b6 = mynodes[(iz + 2) * NX1 + ix] if iz < NSECTIONS_Z - 1 else None

        melementB.SetNodes(
            mynodes[(iz + 1) * NX1 + ix + 1],
            mynodes[(iz + 1) * NX1 + ix],
            mynodes[iz * NX1 + ix + 1],
            b4, b5, b6
        )
        melementB.AddLayer(THICKNESS, 0.0, material)
        elements.append(melementB)

# Fix a 30x30 corner block of nodes (reference tablecloth pinning)
for j in range(30):
    for k in range(30):
        mynodes[j * NX1 + k].SetFixed(True)

# FEA shell: no contact surface needed — cloth interacts only through FEA stiffness
# (no rigid-body collision in this scene; SetCollisionSystemType is omitted)

# Setup + update required before visualization and loop for shell meshes
sys.Setup()
sys.Update()

# === FEA Visualization ===
# Surface shape: deformed mesh coloured by node displacement norm
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Glyph shape: node position dots
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization — Irrlicht window ===
# Y-up scene (FEA convention): CameraVerticalDir_Y
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Tablecloth — PyChrono FEA Shell BST")
vis.Initialize()                                           # Initialize FIRST (Irrlicht rule)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAM_EYE, CAM_TARGET)                         # AFTER Initialize
vis.AddTypicalLights()

# === Review-only recording setup ===

# Cache tip node — free corner diagonally opposite the fixed block
tip_node = mynodes[NSECTIONS_Z * NX1 + NSECTIONS_X]  # cache: fetched once, reused every step

# === Main simulation loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence or invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
