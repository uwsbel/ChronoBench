import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                             # SMC system for stiff FEA shell

mesh = fea.ChMesh()                                                    # the deformable tablecloth mesh
mesh.SetAutomaticGravity(True)                                         # distribute gravity body force over shell nodes

E = 6e4                                                                # Young's modulus (Pa)
nu = 0.0                                                               # Poisson ratio (isotropic Kirchhoff)
density = 100                                                          # surface density (kg/m^3)
thickness = 0.01                                                       # shell thickness (m)

melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)              # isotropic Kirchhoff elasticity
material = fea.ChMaterialShellKirchhoff(melasticity)                  # shell material wrapping the elasticity
material.SetDensity(density)                                           # material density

L_x = 1.0                                                              # tablecloth size along X
L_z = 1.0                                                              # tablecloth size along Z
nsections_x = 40                                                       # cells along X -> 41 nodes
nsections_z = 40                                                       # cells along Z -> 41 nodes

gravity = chrono.ChVector3d(0, -9.81, 0)                              # Y-up gravity
sys.SetGravitationalAcceleration(gravity)                             # g acting downward in Y

# build a (nsections_x+1) x (nsections_z+1) grid of nodes in the XZ plane (y = 0)
nodes = []                                                             # 2D grid of ChNodeFEAxyz, indexed [ix][iz]
for ix in range(nsections_x + 1):
    row = []
    for iz in range(nsections_z + 1):
        x = ix * (L_x / nsections_x)                                   # node X coordinate
        z = iz * (L_z / nsections_z)                                   # node Z coordinate
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, 0, z))            # flat sheet at y = 0
        node.SetMass(0)                                                # mass comes from shell density
        mesh.AddNode(node)                                             # register node in the mesh
        row.append(node)
    nodes.append(row)


def node_at(ix, iz):                                                   # safe neighbour lookup (None outside grid)
    if 0 <= ix <= nsections_x and 0 <= iz <= nsections_z:
        return nodes[ix][iz]
    return None


# two ChElementShellBST triangles per quad cell, each with 3 boundary neighbour nodes
for ix in range(nsections_x):
    for iz in range(nsections_z):
        n00 = nodes[ix][iz]                                            # lower-left corner
        n10 = nodes[ix + 1][iz]                                        # lower-right corner
        n01 = nodes[ix][iz + 1]                                        # upper-left corner
        n11 = nodes[ix + 1][iz + 1]                                    # upper-right corner

        # first triangle: n00, n10, n01 ; neighbours opposite each main node
        tri1 = fea.ChElementShellBST()                                 # BST Kirchhoff shell triangle
        tri1.SetNodes(n00, n10, n01,
                      node_at(ix + 1, iz + 1),                         # neighbour opposite n00
                      node_at(ix - 1, iz + 1),                         # neighbour opposite n10
                      node_at(ix + 1, iz - 1))                         # neighbour opposite n01
        tri1.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)   # single isotropic layer
        mesh.AddElement(tri1)                                          # add triangle to the mesh

        # second triangle: n11, n01, n10 ; neighbours opposite each main node
        tri2 = fea.ChElementShellBST()                                 # second BST triangle of the quad
        tri2.SetNodes(n11, n01, n10,
                      node_at(ix, iz),                                 # neighbour opposite n11
                      node_at(ix + 2, iz),                             # neighbour opposite n01
                      node_at(ix, iz + 2))                             # neighbour opposite n10
        tri2.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)   # single isotropic layer
        mesh.AddElement(tri2)                                          # add triangle to the mesh

sys.Add(mesh)                                                          # add the FEA mesh to the system

# surface visual shape — smooth shaded deformed shell, coloured by node displacement
vis_surface = chrono.ChVisualShapeFEA(mesh)                            # mesh passed to the constructor
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)  # colour by node speed
vis_surface.SetColorscaleMinMax(0.0, 5.5)                              # speed colour range (lo, hi)
vis_surface.SetShellResolution(2)                                     # subdivide shells for smooth rendering
vis_surface.SetSmoothFaces(True)                                       # smooth shading across faces
vis_surface.SetWireframe(False)                                        # solid surface, not wireframe
mesh.AddVisualShapeFEA(vis_surface)                                    # register the surface shape

# node-glyph visual shape — dots at every node position
vis_glyph = chrono.ChVisualShapeFEA(mesh)                              # second shape, mesh ctor arg
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # draw node positions as dots
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)        # glyph shape carries no scalar field
vis_glyph.SetSymbolsThickness(0.006)                                   # dot thickness
mesh.AddVisualShapeFEA(vis_glyph)                                      # register the glyph shape

# pin the two far corners so the cloth folds/drapes under gravity instead of free-falling
nodes[0][0].SetFixed(True)                                             # fix one corner node
nodes[nsections_x][0].SetFixed(True)                                   # fix the adjacent far corner node

sys.SetSolver(mkl.ChSolverPardisoMKL())                               # direct Pardiso MKL solver for stiff shells

sys.Setup()                                                            # finalize the system DOF layout
sys.Update()                                                          # update states before stepping

vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht visualization window
vis.AttachSystem(sys)                                                  # attach the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                     # Y-up world convention
vis.SetWindowSize(1280, 720)                                           # window resolution
vis.SetWindowTitle("Tablecloth folding — Kirchhoff shell")           # window title
vis.Initialize()                                                       # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # PyChrono logo
vis.AddSkyBox()                                                        # standard sky box
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))  # eye, look-at
vis.AddTypicalLights()                                                 # standard two-light setup

time_step = 0.001                                                      # integration step (s)
render_fps = 50.0                                                      # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))          # physics steps per rendered frame
mid = nodes[nsections_x // 2][nsections_z // 2]                       # mid node for the review plot
while vis.Run():                                                       # real-time render loop
    vis.BeginScene()                                                   # begin frame
    vis.Render()                                                       # draw the scene
    vis.EndScene()                                                     # end frame
    for _ in range(render_every):                                      # advance physics between frames
        sys.DoStepDynamics(time_step)                                  # advance one physics step
