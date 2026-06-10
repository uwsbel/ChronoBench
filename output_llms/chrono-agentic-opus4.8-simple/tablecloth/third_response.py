import pychrono.core as chrono                                        # core PyChrono
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer
import pychrono.fea as fea                                            # FEA shells
import pychrono.pardisomkl as mkl                                     # direct sparse solver
import errno                                                          # for the mkdir guard
import os

out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"            # demo output dir
try:
    os.mkdir(out_dir)                                                # create if needed
except OSError as exc:                                               # tolerate pre-existing dir
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

sys = chrono.ChSystemSMC()                                           # SMC system for FEA

mesh = fea.ChMesh()                                                  # container for nodes/elements
sys.Add(mesh)                                                        # register the mesh

density = 100                                                        # shell density kg/m^3
E = 6e4                                                              # Young's modulus
nu = 0.0                                                             # Poisson ratio
thickness = 0.01                                                     # shell thickness

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)            # isotropic Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)                # wrap into a shell material
material.SetDensity(density)                                         # set material density

L_x = 1                                                              # cloth extent in x
nsections_x = 40                                                     # subdivisions in x
L_z = 1                                                              # cloth extent in z
nsections_z = 40                                                     # subdivisions in z

mynodes = []                                                        # keep refs for element wiring
for iz in range(nsections_z + 1):                                   # build the node grid
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))  # node position
        mnode = fea.ChNodeFEAxyz(p)                                 # xyz node
        mesh.AddNode(mnode)                                         # add to mesh
        mynodes.append(mnode)                                       # keep reference

for iz in range(nsections_z):                                       # build two BST triangles per cell
    for ix in range(nsections_x):
        melementA = fea.ChElementShellBST()                        # first triangle
        mesh.AddElement(melementA)

        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]  # neighbour across hypotenuse
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None   # left neighbour
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None   # bottom neighbour

        melementA.SetNodes(
            mynodes[(iz) * (nsections_x + 1) + ix],                # main node 0
            mynodes[(iz) * (nsections_x + 1) + ix + 1],            # main node 1
            mynodes[(iz + 1) * (nsections_x + 1) + ix],            # main node 2
            boundary_1, boundary_2, boundary_3)                    # 3 boundary neighbours
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single layer

        melementB = fea.ChElementShellBST()                        # second triangle
        mesh.AddElement(melementB)

        boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]        # neighbour across hypotenuse
        boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None  # right neighbour
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None   # top neighbour

        melementB.SetNodes(
            mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],        # main node 0
            mynodes[(iz + 1) * (nsections_x + 1) + ix],            # main node 1
            mynodes[(iz) * (nsections_x + 1) + ix + 1],            # main node 2
            boundary_1, boundary_2, boundary_3)                    # 3 boundary neighbours
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single layer

for j in range(30):                                                 # pin one corner region
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)          # fix node to hang the cloth

mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                    # surface visualization of the shells
mvisualizeshellA.SetSmoothFaces(True)                              # smooth shaded faces
mvisualizeshellA.SetWireframe(True)                               # overlay wireframe
mvisualizeshellA.SetShellResolution(2)                            # tessellate each shell
mvisualizeshellA.SetBackfaceCull(True)                            # cull back-facing triangles
mesh.AddVisualShapeFEA(mvisualizeshellA)                          # register surface shape

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)                  # node glyphs
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)        # no scalar field
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # dots at nodes
mvisualizeshellB.SetSymbolsThickness(0.006)                      # glyph size
mesh.AddVisualShapeFEA(mvisualizeshellB)                          # register glyph shape

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht visualization
vis.AttachSystem(sys)                                             # bind the system
vis.SetWindowSize(1024, 768)                                     # window size
vis.SetWindowTitle('Shells FEA test: triangle BST elements')    # window title
vis.Initialize()                                                # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # logo
vis.AddSkyBox()                                                 # sky box
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))  # camera eye/target
vis.AddTypicalLights()                                          # lights

mkl_solver = mkl.ChSolverPardisoMKL()                           # Pardiso MKL direct solver
mkl_solver.LockSparsityPattern(True)                           # reuse sparsity pattern
sys.SetSolver(mkl_solver)                                       # use it for the stiff shells

timestep = 0.005                                                # integration step
sys.Setup()                                                    # set up DOF mapping
sys.Update()                                                   # update derived quantities

monitor = mynodes[len(mynodes) - 1]                            # free corner node to track

render_fps = 50.0                                              # target render cadence
render_every = max(1, round(1.0 / (render_fps * timestep)))   # physics steps per rendered frame
sim_end = 5.0                                                  # stop time


while vis.Run() and sys.GetChTime() < sim_end:                # real-time loop
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):                            # advance a batch of steps
        sys.DoStepDynamics(timestep)                        # step the dynamics
        if sys.GetChTime() >= sim_end:
            break
