import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

out_dir = "FEA_TABLECLOTH"                                            # output dir for the shell demo
try:
    os.mkdir(out_dir)                                                 # create the working output folder
except OSError:
    print("Error creating output directory")                         # tablecloth-style guard

sys = chrono.ChSystemSMC()                                           # SMC system for stiff Kirchhoff shells
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up world, g down

mesh = fea.ChMesh()                                                  # FEA mesh holding the tablecloth shell
mesh.SetAutomaticGravity(False)                                      # apply nodal weight by hand, not auto FEA gravity

# Cloth material — isotropic Kirchhoff thin shell
E = 0.01e9                                                            # Young's modulus (soft cloth) [Pa]
nu = 0.0                                                              # Poisson ratio
thickness = 0.01                                                      # shell thickness [m]
density = 200                                                        # surface density driver [kg/m^3]
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)             # iso Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)                  # shell material wrapper
material.SetDensity(density)                                         # cloth density

# Build a regular grid of xyz nodes for the cloth
nsections_x = 20                                                     # grid divisions along X
nsections_y = 20                                                     # grid divisions along Y
L_x = 1.0                                                            # cloth side length X [m]
L_y = 1.0                                                            # cloth side length Y [m]
node_mass = density * thickness * (L_x / nsections_x) * (L_y / nsections_y)  # lumped nodal mass

nodes = []                                                          # row-major grid of nodes (keep ref: SWIG GC)
for iy in range(nsections_y + 1):
    row = []                                                         # one grid row
    for ix in range(nsections_x + 1):
        x = ix * (L_x / nsections_x) - L_x * 0.5                    # center the cloth at origin in X
        y = 0.5                                                     # start the cloth at height 0.5 m
        z = iy * (L_y / nsections_y) - L_y * 0.5                    # center the cloth in Z
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, z))         # xyz node (BST shells use position-only nodes)
        node.SetMass(node_mass)                                     # lumped mass at the node
        mesh.AddNode(node)                                          # register node with the mesh
        row.append(node)                                            # store in the grid
    nodes.append(row)                                              # append the row

def get_node(ix, iy):                                               # safe grid lookup; None outside the boundary
    if ix < 0 or iy < 0 or ix > nsections_x or iy > nsections_y:
        return None
    return nodes[iy][ix]

# Build BST triangle elements with their 3 boundary-neighbour nodes (Kirchhoff BST stencil)
for iy in range(nsections_y):
    for ix in range(nsections_x):
        # split each quad into two triangles; each ChElementShellBST takes 3 main + 3 neighbour nodes
        # --- lower-left triangle: (ix,iy) (ix+1,iy) (ix,iy+1) ---
        nA0 = get_node(ix,     iy)                                  # main node 0
        nA1 = get_node(ix + 1, iy)                                 # main node 1
        nA2 = get_node(ix,     iy + 1)                             # main node 2
        bA0 = get_node(ix + 1, iy + 1)                            # neighbour opposite edge 0
        bA1 = get_node(ix,     iy - 1)                            # neighbour opposite edge 1 (None at boundary)
        bA2 = get_node(ix - 1, iy)                                # neighbour opposite edge 2 (None at boundary)
        eleA = fea.ChElementShellBST()                            # Kirchhoff BST triangle
        eleA.SetNodes(nA0, nA1, nA2, bA0, bA1, bA2)               # 3 main + 3 neighbour nodes
        eleA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single cloth layer
        mesh.AddElement(eleA)                                     # add triangle to the mesh
        # --- upper-right triangle: (ix+1,iy) (ix+1,iy+1) (ix,iy+1) ---
        nB0 = get_node(ix + 1, iy)                                # main node 0
        nB1 = get_node(ix + 1, iy + 1)                            # main node 1
        nB2 = get_node(ix,     iy + 1)                            # main node 2
        bB0 = get_node(ix,     iy)                                # neighbour opposite edge 0
        bB1 = get_node(ix + 2, iy + 1)                            # neighbour opposite edge 1 (None at boundary)
        bB2 = get_node(ix + 1, iy + 2)                            # neighbour opposite edge 2 (None at boundary)
        eleB = fea.ChElementShellBST()                            # Kirchhoff BST triangle
        eleB.SetNodes(nB0, nB1, nB2, bB0, bB1, bB2)               # 3 main + 3 neighbour nodes
        eleB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single cloth layer
        mesh.AddElement(eleB)                                     # add triangle to the mesh

sys.Add(mesh)                                                      # register the FEA mesh with the system

# A rigid table/box under the cloth for it to drape over
contact_mat = chrono.ChContactMaterialSMC()                       # SMC contact material
contact_mat.SetYoungModulus(6e4)                                  # contact stiffness
contact_mat.SetFriction(0.3)                                      # friction coefficient
contact_mat.SetRestitution(0.0)                                   # no bounce
contact_mat.SetAdhesion(0)                                        # no adhesion

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required: cloth contacts the table

# FEA contact surface so the cloth nodes can collide with the table
cloth_contact = fea.ChContactSurfaceNodeCloud(contact_mat)        # node-cloud contact for the shell
mesh.AddContactSurface(cloth_contact)                            # attach to the mesh (sets owning mesh)
cloth_contact.AddAllNodes(0.01)                                 # sphere radius around each cloth node

# Rigid table the cloth falls onto
table = chrono.ChBodyEasyBox(0.6, 0.4, 0.6, 1000, True, True, contact_mat)  # box: size, density, vis, coll, mat
table.SetPos(chrono.ChVector3d(0, 0.0, 0))                       # table top near y=0.2
table.SetFixed(True)                                            # the table does not move
sys.Add(table)                                                  # add the table to the system

# === Cloth visualization shapes (two-shape FEA pattern) ===
# Shape A — coloured deformed surface with the prompt's visualization enhancements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                 # surface visual shape (mesh is ctor arg)
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)  # colour by node speed
mvisualizeshellA.SetColorscaleMinMax(0.0, 5.0)                  # speed colour range (lo, hi scalars)
mvisualizeshellA.SetSmoothFaces(True)                          # smooth shaded faces (prompt)
mvisualizeshellA.SetWireframe(True)                            # overlay wireframe (prompt)
mvisualizeshellA.SetBackfaceCull(True)                         # cull back faces (prompt)
mvisualizeshellA.SetShellResolution(2)                         # tessellate each shell for smoother render
mesh.AddVisualShapeFEA(mvisualizeshellA)                       # register surface shape

# Shape B — node glyphs (dots at the cloth nodes)
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)               # glyph visual shape
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # show node dots
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
mvisualizeshellB.SetSymbolsThickness(0.006)                   # glyph dot thickness
mesh.AddVisualShapeFEA(mvisualizeshellB)                      # register glyph shape

# Solver + timestepper for the stiff shell stiffness matrix
solver = mkl.ChSolverPardisoMKL()                             # direct sparse solver (stiff shells)
sys.SetSolver(solver)                                         # use Pardiso MKL
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)  # stable implicit stepper

sys.Setup()                                                  # finalize the system topology (shells require it)
sys.Update()                                                # update state before stepping

# === Irrlicht visualization (Initialize first, then scene elements; NO grid) ===
vis = chronoirr.ChVisualSystemIrrlicht()                    # Irrlicht render window
vis.AttachSystem(sys)                                       # bind to the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)          # Y-up vertical for this FEA scene
vis.SetWindowSize(1280, 720)                               # window resolution
vis.SetWindowTitle("FEA tablecloth (Kirchhoff BST shell)")  # window title
vis.Initialize()                                           # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                            # standard sky box
vis.AddCamera(chrono.ChVector3d(1.0, 1.4, 1.6), chrono.ChVector3d(0, 0.2, 0))  # eye, look-at
vis.AddTypicalLights()                                     # standard two-light setup

time_step = 1e-3                                           # stiff-shell timestep [s]
sim_end = 5.0                                              # simulation duration [s]
render_fps = 50.0                                          # target review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged render-cadence constant
center_node = get_node(nsections_x // 2, nsections_y // 2)   # central cloth node (for the review plot mirror)
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                      # begin a render frame
    vis.Render()                                          # draw the scene
    vis.EndScene()                                        # finish the frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                     # advance one physics step
        if sys.GetChTime() >= sim_end:
            break
