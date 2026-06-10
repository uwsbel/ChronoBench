import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # SMC system for the stiff Kirchhoff shell
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity, g = 9.81 down

try:
    os.mkdir("tablecloth_out")                                       # output dir for shell results
except OSError:
    print("Error creating directory")                               # tolerate an existing directory

mesh = fea.ChMesh()                                                  # FEA mesh holding the cloth nodes/elements
mesh.SetAutomaticGravity(False)                                      # shell weight applied via explicit node forces

E = 0.01e9                                                           # Young's modulus of the cloth (Pa)
nu = 0.0                                                             # Poisson ratio
thickness = 0.01                                                     # shell layer thickness (m)
density = 100.0                                                      # cloth density (kg/m^3)

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)            # isotropic Kirchhoff elasticity
mdamping = fea.ChDampingKirchhoffRayleigh(melasticity)             # Rayleigh damping for the shell
mdamping.SetBeta(0.01)                                            # stiffness-proportional damping
material = fea.ChMaterialShellKirchhoff(melasticity, None, mdamping)  # shell material with damping
material.SetDensity(density)                                        # cloth surface density

L_x = 1.0                                                            # cloth side length in X (m)
L_z = 1.0                                                            # cloth side length in Z (m)
nsections_x = 40                                                     # grid divisions along X
nsections_z = 40                                                     # grid divisions along Z

mynodes = []                                                         # row-major list of all grid nodes

for iz in range(nsections_z + 1):                                   # build the (nx+1) x (nz+1) node grid
    for ix in range(nsections_x + 1):
        x = ix * (L_x / nsections_x)                               # node X coordinate
        z = iz * (L_z / nsections_z)                               # node Z coordinate
        y = 0.0                                                     # flat cloth starts in the Y = 0 plane
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))        # position-only shell node
        node.SetMass(0.0)                                           # mass comes from the shell layer
        mesh.AddNode(node)                                         # register the node with the mesh
        mynodes.append(node)                                       # keep a strong reference (SWIG GC)

load_force = chrono.ChVector3d(0, -0.005, 0)                        # per-node load force vector (N)
nodesLoad = []                                                      # nodes that receive the applied load
for iz in range(nsections_z + 1):                                  # apply the load over every interior node
    for ix in range(nsections_x + 1):
        n = mynodes[iz * (nsections_x + 1) + ix]                   # node at (ix, iz)
        n.SetForce(load_force)                                     # downward load on the cloth node
        nodesLoad.append(n)                                       # track the loaded nodes

def ref_X(t):                                                      # reference X trajectory for tracking
    return 0.5 + 0.1 * math.sin(2.0 * math.pi * t)                # gentle horizontal oscillation

def ref_Y(t):                                                      # reference Y trajectory for tracking
    return -0.2 * (1.0 - math.cos(2.0 * math.pi * t))             # vertical reference profile

nodePlotA = mynodes[0]                                              # first node monitored for plotting
nodePlotB = mynodes[nsections_x]                                    # opposite-corner node monitored for plotting

mnodemonitor = mynodes[(nsections_z // 2) * (nsections_x + 1) + (nsections_x // 2)]  # central monitored node
melementmonitor = None                                              # element selected for monitoring below

for iz in range(nsections_z):                                      # build BST triangle pairs per grid cell
    for ix in range(nsections_x):
        n0 = mynodes[iz * (nsections_x + 1) + ix]                  # lower-left node of the cell
        n1 = mynodes[iz * (nsections_x + 1) + ix + 1]             # lower-right node
        n2 = mynodes[(iz + 1) * (nsections_x + 1) + ix]          # upper-left node
        n3 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]     # upper-right node

        # neighbour nodes for the BST bending stencil; None at the grid boundary
        nb_a = mynodes[(iz - 1) * (nsections_x + 1) + ix] if (iz > 0) else None         # below n0/n1
        nb_b = mynodes[iz * (nsections_x + 1) + ix - 1] if (ix > 0) else None           # left of n0/n2
        nb_c = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]                            # diagonal neighbour

        melementA = fea.ChElementShellBST()                       # lower triangle of the cell
        melementA.SetNodes(n0, n1, n2, nb_a, nb_b, nb_c)         # 3 main + 3 boundary-aware neighbours
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single isotropic layer
        mesh.AddElement(melementA)                                # register the element

        if (iz == 0 and ix == 1):                                 # pick one specific element to monitor
            melementmonitor = melementA

        nb_d = mynodes[(iz + 2) * (nsections_x + 1) + ix + 1] if (iz < nsections_z - 1) else None  # above n3
        nb_e = mynodes[(iz + 1) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None  # right of n3
        nb_f = mynodes[iz * (nsections_x + 1) + ix]                                       # diagonal neighbour

        melementB = fea.ChElementShellBST()                       # upper triangle of the cell
        melementB.SetNodes(n3, n2, n1, nb_d, nb_e, nb_f)         # mirrored stencil for the second triangle
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single isotropic layer
        mesh.AddElement(melementB)                                # register the element

for j in range(30):                                                # fix a 30x30 block of upper nodes (clamp edge)
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)         # pin the node so the cloth hangs from it

sys.Add(mesh)                                                      # add the FEA mesh to the system

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                   # surface field visualization of the cloth
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)  # color by node speed
mvisualizeshellA.SetColorscaleMinMax(0.0, 5.0)                     # speed color range (lo, hi)
mvisualizeshellA.SetSmoothFaces(True)                              # smooth the rendered shell faces
mvisualizeshellA.SetWireframe(False)                              # solid (non-wireframe) shell surface
mvisualizeshellA.SetShellResolution(2)                            # tessellation for shell visualization
mesh.AddVisualShapeFEA(mvisualizeshellA)                           # register the surface visual shape

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)                   # node glyph visualization
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # draw node dots
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)            # no scalar field on glyphs
mvisualizeshellB.SetSymbolsThickness(0.006)                       # glyph dot size
mesh.AddVisualShapeFEA(mvisualizeshellB)                           # register the glyph visual shape

solver = mkl.ChSolverPardisoMKL()                                 # direct sparse solver for the stiff shell
solver.LockSparsityPattern(True)                                  # lock sparsity pattern to speed factorization
sys.SetSolver(solver)                                             # use Pardiso MKL

sys.Setup()                                                       # finalize the shell mesh assembly
sys.Update()                                                      # update derived quantities before stepping

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht render window
vis.AttachSystem(sys)                                            # attach the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)               # Y-up camera convention
vis.SetWindowSize(1280, 720)                                    # window resolution
vis.SetWindowTitle("FEA shell tablecloth")                     # window title
vis.Initialize()                                                # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                 # standard sky box
vis.AddCamera(chrono.ChVector3d(0.5, 1.2, -1.5), chrono.ChVector3d(0.5, -0.2, 0.5))  # eye, target
vis.AddTypicalLights()                                          # standard lighting

time_step = 0.005                                               # solver time step
sim_end = 5.0                                                   # total simulated time (s)
render_fps = 50.0                                              # target review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = sys.GetChTime()                                  # current simulated time
        sys.DoStepDynamics(time_step)                        # advance the shell dynamics one step
        if sys.GetChTime() >= sim_end:
            break
