import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"             # FEA-shell output dir

try:                                                                  # tablecloth os.mkdir guard
    os.mkdir(out_dir)
except OSError as exc:                                                # tolerate existing dir
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

sys = chrono.ChSystemSMC()                                            # FEA scenes use SMC

mesh = fea.ChMesh()                                                   # container for nodes/elements
sys.Add(mesh)                                                         # register the mesh

nodePlotA = fea.ChNodeFEAxyz()                                        # plotting node A
nodePlotB = fea.ChNodeFEAxyz()                                        # plotting node B
nodesLoad = []                                                        # nodes that receive the load

ref_X = chrono.ChFunctionInterp()                                     # reference tracking X
ref_Y = chrono.ChFunctionInterp()                                    # reference tracking Y

load_force = chrono.ChVector3d()                                      # applied load force vector

mnodemonitor = fea.ChNodeFEAxyz()                                     # monitored node
melementmonitor = fea.ChElementShellBST()                            # monitored element

density = 100                                                         # cloth density kg/m^3
E = 6e4                                                               # Young's modulus
nu = 0.0                                                              # Poisson ratio
thickness = 0.01                                                      # shell thickness

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)             # isotropic Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)                 # shell material
material.SetDensity(density)                                          # set cloth density

L_x = 1                                                               # cloth size in X
nsections_x = 40                                                      # subdivisions in X
L_z = 1                                                               # cloth size in Z
nsections_z = 40                                                      # subdivisions in Z

mynodes = []                                                         # grid nodes, kept for element loop
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  # node position on XZ
        mnode = fea.ChNodeFEAxyz(p)                                  # create the xyz node
        mesh.AddNode(mnode)                                          # add node to mesh
        mynodes.append(mnode)                                        # store strong ref (SWIG GC)

for iz in range(nsections_z):
    for ix in range(nsections_x):
        melementA = fea.ChElementShellBST()                         # first BST triangle of the cell
        mesh.AddElement(melementA)

        if (iz == 0 and ix == 1):                                   # tag one element for monitoring
            ementmonitor = melementA

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1] # opposite-corner neighbour
        if (ix > 0):                                                # guard against left edge
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1]
        else:
            boundary_2 = None
        if (iz > 0):                                                # guard against bottom edge
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1]
        else:
            boundary_3 = None

        melementA.SetNodes(                                         # 3 main nodes + 3 BST neighbours
            mynodes[(iz    ) * (nsections_x + 1) + ix    ],
            mynodes[(iz    ) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix    ],
            boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single shell layer

        melementB = fea.ChElementShellBST()                         # second BST triangle of the cell
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz    ) * (nsections_x + 1) + ix    ] # opposite-corner neighbour
        if (ix < nsections_x - 1):                                  # guard against right edge
            boundary_2 = mynodes[(iz    ) * (nsections_x + 1) + ix + 2]
        else:
            boundary_2 = None
        if (iz < nsections_z - 1):                                  # guard against top edge
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix    ]
        else:
            boundary_3 = None

        melementB.SetNodes(                                         # mirrored triangle node set
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
            mynodes[(iz + 1) * (nsections_x + 1) + ix    ],
            mynodes[(iz    ) * (nsections_x + 1) + ix + 1],
            boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single shell layer

for j in range(30):                                                 # fix the upper block of nodes
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)           # pin node so cloth hangs

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                    # surface shape (deformed cloth)
mvisualizeshellA.SetSmoothFaces(True)                              # smooth shaded faces
mvisualizeshellA.SetWireframe(True)                               # show wireframe overlay
mvisualizeshellA.SetShellResolution(2)                             # subdivide each shell for render
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)                    # glyph shape (node dots)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # node position dots
mvisualizeshellB.SetSymbolsThickness(0.006)                        # glyph dot size
mesh.AddVisualShapeFEA(mvisualizeshellB)

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht render window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()                                                   # Initialize first, then scene nodes
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))  # eye, target
vis.AddTypicalLights()

mkl_solver = mkl.ChSolverPardisoMKL()                              # direct solver for stiff shells
mkl_solver.LockSparsityPattern(True)                              # lock sparsity to speed up factorization
sys.SetSolver(mkl_solver)

timestep = 0.005                                                   # shell timestep
sys.Setup()                                                       # finalize DOFs after building mesh
sys.Update()                                                      # initial state update

rec_X = chrono.ChFunctionInterp()                                 # recorded response X
rec_Y = chrono.ChFunctionInterp()                                # recorded response Y

mtime = 0                                                         # elapsed simulation time

sim_end = 5.0                                                                  # review-only run horizon
render_fps = 50.0                                                              # review-only target fps
render_every = max(1, round(1.0 / (render_fps * timestep)))                   # untagged cadence constant

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(timestep)
