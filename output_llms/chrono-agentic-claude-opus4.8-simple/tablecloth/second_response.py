import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                            # SMC system for stiff Kirchhoff shells

out_dir = "tablecloth_output"                                        # frame/data output directory
try:
    os.mkdir(out_dir)                                                # may already exist
except OSError:
    pass                                                            # ignore pre-existing dir

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(False)                                      # forced/quasi-static cloth response
sys.Add(mesh)                                                       # register mesh with system

density = 100.0                                                      # kg/m^3
E = 6e4                                                              # Young modulus [Pa]
nu = 0.0                                                             # Poisson ratio
thickness = 0.01                                                    # shell thickness [m]
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)            # isotropic Kirchhoff elasticity (note spelling)
material = fea.ChMaterialShellKirchhoff(melasticity)                 # shell material wrapper
material.SetDensity(density)                                        # cloth density

L_x = 1.0                                                            # sheet size in X
L_z = 1.0                                                            # sheet size in Z
nsections_x = 40                                                     # cells along X -> 41 nodes
nsections_z = 40                                                     # cells along Z -> 41 nodes

load_force = chrono.ChVector3d(0, -1.2, 0)                           # downward load force on tracked nodes
nodesLoad = []                                                       # nodes that receive load_force

# reference tracking interpolation functions (parametric u in [0,1])
def ref_X(u):
    return u * L_x                                                  # reference X coordinate along the sheet

def ref_Y(u):
    return 0.0                                                      # reference Y (flat sheet at y=0)

nodePlotA = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))            # monitored plot node A (reassigned below)
nodePlotB = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))            # monitored plot node B (reassigned below)
mnodemonitor = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))         # generic monitored node (reassigned below)
melementmonitor = None                                              # monitored element (assigned below)

# build the 41x41 grid of shell nodes in the XZ plane (y = 0)
mynodes = []                                                        # flat list, index = ix*(nsections_z+1)+iz
for ix in range(nsections_x + 1):
    for iz in range(nsections_z + 1):
        px = ix * (L_x / nsections_x)                              # node X coordinate
        pz = iz * (L_z / nsections_z)                              # node Z coordinate
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(px, 0.0, pz))   # cloth node at y=0
        mesh.AddNode(node)                                        # add to mesh
        mynodes.append(node)                                      # keep strong reference

# two ChElementShellBST triangles per quad cell, with the BST neighbour stencil
melementA = None                                                   # first element (used for monitor)
for ix in range(nsections_x):
    for iz in range(nsections_z):
        # main-cell node indices
        inode0 = (ix) * (nsections_z + 1) + (iz)                  # (ix,   iz)
        inode1 = (ix + 1) * (nsections_z + 1) + (iz)             # (ix+1, iz)
        inode2 = (ix) * (nsections_z + 1) + (iz + 1)             # (ix,   iz+1)
        inode3 = (ix + 1) * (nsections_z + 1) + (iz + 1)        # (ix+1, iz+1)

        # boundary neighbour nodes with conditional checks to avoid out-of-bound (ix>0)/(iz>0)
        boundaryAx = mynodes[(ix - 1) * (nsections_z + 1) + (iz + 1)] if (ix > 0) else None  # neighbour across -X edge
        boundaryAz = mynodes[(ix + 1) * (nsections_z + 1) + (iz - 1)] if (iz > 0) else None  # neighbour across -Z edge

        # triangle A: nodes (0,1,2) with two neighbours + a free diagonal neighbour
        melementA = fea.ChElementShellBST()                       # first BST triangle
        melementA.SetNodes(mynodes[inode0], mynodes[inode1], mynodes[inode2],
                           boundaryAx, boundaryAz, mynodes[inode3])   # 3 main + 3 neighbours (None at edges)
        melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single shell layer
        mesh.AddElement(melementA)                                # add triangle A

        # element monitor: pick a specific element when (iz == 0 and ix == 1)
        if iz == 0 and ix == 1:
            melementmonitor = melementA                           # ementmonitor = melementA

        # boundary neighbour nodes for the second triangle
        boundaryBx = mynodes[(ix + 1) * (nsections_z + 1) + (iz + 2)] if (iz < nsections_z - 1) else None  # +Z edge
        boundaryBz = mynodes[(ix + 2) * (nsections_z + 1) + (iz + 1)] if (ix < nsections_x - 1) else None  # +X edge

        # triangle B: nodes (1,3,2) closing the quad
        melementB = fea.ChElementShellBST()                       # second BST triangle
        melementB.SetNodes(mynodes[inode1], mynodes[inode3], mynodes[inode2],
                           boundaryBx, boundaryBz, mynodes[inode0])   # 3 main + 3 neighbours
        melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)  # single shell layer
        mesh.AddElement(melementB)                                # add triangle B

# fix the upper block of nodes (30x30 corner clamps the sheet)
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)        # pin node to ground

# assign monitor / plot / load nodes from the grid
nodePlotA = mynodes[(nsections_x) * (nsections_z + 1) + (nsections_z)]   # far corner node A
nodePlotB = mynodes[(nsections_x) * (nsections_z + 1) + 0]              # far edge node B
mnodemonitor = nodePlotA                                          # generic monitored node
nodesLoad.append(nodePlotA)                                       # load the free corner
nodesLoad.append(nodePlotB)                                       # load the free edge
for nl in nodesLoad:
    nl.SetForce(load_force)                                       # apply load force to tracked nodes

# visual shape A — coloured deformed surface (shell resolution 2)
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)                  # mesh is a ctor arg
mvisualizeshellA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # deformed surface field
mvisualizeshellA.SetSmoothFaces(True)                            # smooth shaded faces
mvisualizeshellA.SetWireframe(False)                             # solid (wireframe + backface cull are optional)
# mvisualizeshellA.SetWireframe(True)                            # optional wireframe view
# mvisualizeshellA.SetBackfaceCull(True)                         # optional backface culling
mvisualizeshellA.SetShellResolution(2)                           # subdivide each shell for display
mesh.AddVisualShapeFEA(mvisualizeshellA)                          # register surface shape

# visual shape B — node glyphs (dots), no scalar field
mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)                 # second visual shape
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # node dot glyphs
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)            # no scalar field on glyphs
mvisualizeshellB.SetSymbolsThickness(0.006)                      # glyph dot thickness
mesh.AddVisualShapeFEA(mvisualizeshellB)                          # register glyph shape

mkl_solver = mkl.ChSolverPardisoMKL()                            # direct sparse solver for stiff shells
mkl_solver.LockSparsityPattern(True)                             # lock sparsity pattern for speed
sys.SetSolver(mkl_solver)                                        # install solver

ts = chrono.ChTimestepperHHT(sys)                                # implicit HHT integrator
ts.SetStepControl(False)                                         # fixed-step (canonical-minimal HHT)
sys.SetTimestepper(ts)                                           # install timestepper

sys.Setup()                                                      # finalize system DOFs (shells require it)
sys.Update()                                                     # update state before stepping

vis = chronoirr.ChVisualSystemIrrlicht()                         # Irrlicht visualization
vis.AttachSystem(sys)                                            # attach the physical system
vis.SetWindowSize(1024, 768)                                     # window resolution
vis.SetWindowTitle("Kirchhoff shell tablecloth")                # window title
vis.Initialize()                                                # Irrlicht: Initialize FIRST
vis.AddLogo()                                                   # default logo
vis.AddSkyBox()                                                 # sky backdrop
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))  # eye, target
vis.AddTypicalLights()                                          # default lighting

timestep = 0.005                                                # integration step (raised from 0.001)
render_fps = 50.0                                               # target render frame rate
render_every = max(1, round(1.0 / (render_fps * timestep)))     # physics steps per rendered frame

while vis.Run():                                               # real-time render loop
    vis.BeginScene()                                           # begin frame
    vis.Render()                                               # draw scene
    vis.EndScene()                                             # end frame
    for _ in range(render_every):                              # advance physics between frames
        sys.DoStepDynamics(timestep)                           # one HHT step
