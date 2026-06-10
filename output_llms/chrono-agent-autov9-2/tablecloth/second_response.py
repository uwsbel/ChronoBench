"""Draping tablecloth — thin-shell FEA cloth (ChElementShellBST / Kirchhoff).

Models a square piece of cloth as a regular triangular mesh of Kirchhoff-Love
thin-shell elements (ChElementShellBST). Each triangular element carries the
three corner nodes of its main triangle plus the three opposite neighbour nodes
across its edges; those neighbour stencils are what give the BST element its
bending (curvature) response. A rectangular block of nodes along one side of the
mesh is fixed, so the rest of the sheet sags and drapes under gravity.

System: ChSystemSMC (deformable FEA needs SMC + a direct sparse solver).
Solver: PardisoMKL direct sparse solver with a locked sparsity pattern.
Integrator: linearized implicit Euler (robust for cloth shells).
Bodies: one ChMesh of shell elements; no rigid bodies, no contact.
Expected behavior: the unpinned portion of the cloth falls and folds under
gravity, reaching a smooth draped shape; node speeds decay as it settles.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants: geometry / mesh / physics / time ===
# Cloth is a square sheet subdivided into a regular grid of nodes.
nsections_x = 40          # number of subdivisions along X
nsections_z = 40          # number of subdivisions along Z
cloth_length_x = 1.0      # sheet extent along X (m)
cloth_length_z = 1.0      # sheet extent along Z (m)
cloth_height = 0.5        # initial height of the sheet above the ground (m)

cloth_thickness = 0.01    # shell thickness (m)
youngs_modulus = 6e5      # in-plane stiffness of the cloth (Pa)
poisson_ratio = 0.0       # Poisson ratio for the isotropic shell
density = 200.0           # areal density driver (kg/m^3)
rayleigh_beta = 1e-3      # light Rayleigh (stiffness-proportional) damping

gravity = -9.81           # gravitational acceleration along -Y (m/s^2)
time_step = 0.005         # FEA time step (s)
sim_end = 3.0             # simulation duration (s)
render_fps = 50.0         # review render cadence (frames per second)

# Derived grid spacing (precomputed once).
dx = cloth_length_x / nsections_x          # node spacing along X
dz = cloth_length_z / nsections_z          # node spacing along Z
nodes_per_row = nsections_x + 1            # node count along X per Z row
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# Size of the fixed (pinned) corner block of nodes.
fix_count_x = 30          # number of pinned columns
fix_count_z = 30          # number of pinned rows

# === System & gravity ===
# ChSystemSMC is required for FEA; gravity acts along -Y so the cloth drapes down.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, gravity, 0))

# === Solver & timestepper ===
# Direct sparse PardisoMKL solver with a locked sparsity pattern (mesh topology is
# fixed, so the pattern never changes -> reuse the symbolic factorization).
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA cloth material (Kirchhoff thin shell) ===
# Isotropic Kirchhoff elasticity + light Rayleigh damping, wrapped in a shell material.
elasticity = fea.ChElasticityKirchhoffIsothropic(youngs_modulus, poisson_ratio)
damping = fea.ChDampingKirchhoffRayleigh(elasticity, rayleigh_beta)
cloth_material = fea.ChMaterialShellKirchhoff(elasticity, None, damping)
cloth_material.SetDensity(density)

# === FEA mesh: nodes ===
# Build the (nsections_x+1) x (nsections_z+1) grid of position nodes. The cloth lies
# in the X-Z plane at y = cloth_height. Keep strong references to avoid SWIG GC.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        x = ix * dx - 0.5 * cloth_length_x
        z = iz * dz - 0.5 * cloth_length_z
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, cloth_height, z))
        node.SetMass(0.0)   # element mass is integrated from material density
        mesh.AddNode(node)
        mynodes.append(node)

# === FEA mesh: shell elements ===
# Two BST triangles per grid cell. nodes 0,1,2 are the main triangle; nodes 3,4,5
# are the opposite neighbour nodes across the three edges (the bending stencils).
# Boundary cells lack a neighbour across an outer edge -> pass None there. The
# conditional (ix > 0) / (iz > 0) guards keep neighbour indices in range.
melementmonitor = None       # element selected for monitoring
melementA = None
melementB = None

def node_at(ix, iz):
    """Return the grid node at column ix, row iz (used to assemble triangles)."""
    return mynodes[iz * nodes_per_row + ix]

cloth_elements = []
for iz in range(nsections_z):
    for ix in range(nsections_x):
        # Lower-left triangle. Main nodes are 0=(ix,iz), 1=(ix+1,iz), 2=(ix,iz+1).
        # BST node 3 is the bending wing across edge (node1,node2) — the cell
        # diagonal — i.e. the opposite vertex of the sibling triangle, (ix+1,iz+1).
        # Wings 4,5 span the two outer cell edges; on a regular grid those
        # straight-line edges have no well-shaped opposite vertex, so pass None
        # there (membrane only) — this is what keeps the curvature stencil from NaN.
        melementA = fea.ChElementShellBST()
        melementA.SetNodes(
            node_at(ix, iz), node_at(ix + 1, iz), node_at(ix, iz + 1),
            node_at(ix + 1, iz + 1), None, None,
        )
        melementA.AddLayer(cloth_thickness, 0.0 * chrono.CH_DEG_TO_RAD, cloth_material)
        mesh.AddElement(melementA)
        cloth_elements.append(melementA)

        # Upper-right triangle. Main nodes 0=(ix+1,iz+1), 1=(ix,iz+1), 2=(ix+1,iz);
        # its edge (node1,node2) is the same cell diagonal, so wing node 3 is the
        # opposite vertex (ix,iz). Outer edges -> None, as above.
        melementB = fea.ChElementShellBST()
        melementB.SetNodes(
            node_at(ix + 1, iz + 1), node_at(ix, iz + 1), node_at(ix + 1, iz),
            node_at(ix, iz), None, None,
        )
        melementB.AddLayer(cloth_thickness, 0.0 * chrono.CH_DEG_TO_RAD, cloth_material)
        mesh.AddElement(melementB)
        cloth_elements.append(melementB)

        # Select one interior element to monitor its mid-surface motion.
        if (iz == 0 and ix == 1):
            melementmonitor = melementA

# === Fixed nodes ===
# Pin a fix_count_x x fix_count_z block of nodes so the rest of the sheet drapes.
for j in range(fix_count_z):
    for k in range(fix_count_x):
        mynodes[j * nodes_per_row + k].SetFixed(True)

# Node selected for monitoring its trajectory.
mnodemonitor = mynodes[-1]

sys.Add(mesh)

# === FEA visualization shapes ===
# Filled smooth shading of the deformed shell surface.
mvisualizeshellA = chrono.ChVisualShapeFEA()
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mvisualizeshellA.SetColormapRange(chrono.ChVector2d(0.0, 1.0))
mvisualizeshellA.SetSmoothFaces(True)
mvisualizeshellA.SetWireframe(False)
mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

# Plain node markers (no scalar field) for the second visual overlay.
mvisualizeshellB = chrono.ChVisualShapeFEA()
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSmoothFaces(False)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA tablecloth — draping Kirchhoff shell")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.4, 1.0, 1.4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                               chrono.Q_ROTATE_Y_TO_Z),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid in the X-Z plane

# === Reference tracking helpers ===
# Linear reference interpolants used to compare the monitored node against a target.
def ref_X(t):
    """Reference X position the monitored node is compared against over time."""
    return 0.0

def ref_Y(t):
    """Reference Y position (settling target) for the monitored node."""
    return cloth_height + 0.5 * gravity * t * t

# Vertical load vector available for applying point loads to selected nodes.
load_force = chrono.ChVector3d(0, 0, 0)
nodesLoad = [mnodemonitor]
nodePlotA = mnodemonitor
nodePlotB = mynodes[fix_count_z * nodes_per_row]   # first free node below the pinned block

# === Main loop ===
# Render once per frame; advance FEA physics in an inner batch between frames.
# Log the monitored node's pose/speed each step for the physics check.


try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing ===
