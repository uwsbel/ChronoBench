"""
Tablecloth folding simulation using PyChrono FEA shell elements.

System:    ChSystemSMC (required for FEA)
Solver:    PardisoMKL (direct, for stiff Kirchhoff shells)
Elements:  ChElementShellBST with isotropic Kirchhoff material
Mesh:      NxN grid of triangular BST shell elements
Objective: Model a rectangular tablecloth that folds / droops under gravity using
           shell finite elements. Corner nodes along one edge are fixed; gravity
           causes the cloth to fold and drape.
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
TIME_STEP   = 0.001        # s — stiff shell timestep
SIM_END     = 5.0          # s
RENDER_FPS  = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Tablecloth geometry
CLOTH_LX    = 1.0          # m — width  (X)
CLOTH_LY    = 1.0          # m — length (Y)
DIV_X       = 10           # mesh divisions along X
DIV_Y       = 10           # mesh divisions along Y

# Shell material (isotropic Kirchhoff cloth)
THICKNESS   = 0.01         # m
E_MOD       = 5e4          # Pa — compliant cloth
NU          = 0.3          # Poisson ratio
DENSITY     = 100.0        # kg/m³

# === System & gravity (Y-up, FEA convention) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Pardiso MKL direct solver for stiff Kirchhoff shells
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper (canonical minimal form for FEA shell scenes)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh — tablecloth BST shell grid ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Kirchhoff isotropic material (note: spelling is "Isothropic" in 9.0.0 API)
melasticity = fea.ChElasticityKirchhoffIsothropic(E_MOD, NU)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(DENSITY)

# Build a (DIV_X+1) x (DIV_Y+1) grid of ChNodeFEAxyz nodes
#   X direction: 0 .. CLOTH_LX
#   Y direction: 0 .. CLOTH_LY (vertical in Y-up world)
#   Cloth laid out in the XY plane, hanging from the top edge (Y = CLOTH_LY)

nodes = []
dx = CLOTH_LX / DIV_X
dy = CLOTH_LY / DIV_Y

for iy in range(DIV_Y + 1):
    row = []
    for ix in range(DIV_X + 1):
        x = ix * dx
        y = iy * dy
        node = fea.ChNodeFEAxyz(chrono.ChVector3d(x, y, 0.0))
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)

# Fix the top edge (iy == DIV_Y) so the cloth hangs from there
for ix in range(DIV_X + 1):
    nodes[DIV_Y][ix].SetFixed(True)

# Build BST shell elements — each quad cell is split into 2 triangles
# BST element topology: SetNodes(n0, n1, n2, nb0, nb1, nb2)
#   (n0,n1,n2) = corner nodes; nb0/nb1/nb2 = neighbour nodes (None at boundary edges)
#
# For a quad (ix, iy)→(ix+1,iy)→(ix+1,iy+1)→(ix,iy+1):
#   Triangle A: n00, n10, n01  (lower-left, lower-right, upper-left)
#   Triangle B: n10, n11, n01  (lower-right, upper-right, upper-left)

for iy in range(DIV_Y):
    for ix in range(DIV_X):
        n00 = nodes[iy    ][ix    ]
        n10 = nodes[iy    ][ix + 1]
        n01 = nodes[iy + 1][ix    ]
        n11 = nodes[iy + 1][ix + 1]

        # Neighbour lookups — None at boundary
        def get_node(row, col):
            if 0 <= row <= DIV_Y and 0 <= col <= DIV_X:
                return nodes[row][col]
            return None

        # Triangle A: (n00, n10, n01)
        # Neighbours opposite each vertex (the node across the shared edge)
        nb_A0 = get_node(iy,     ix + 2)   # opposite n00 in edge n10-n01 → across from n00: use n11 approximation; cross-elem = n10-col+1 if available
        nb_A1 = get_node(iy - 1, ix    )   # opposite n10 across edge n00-n01 → below row
        nb_A2 = get_node(iy,     ix + 1) if ix + 1 <= DIV_X else None  # kept consistent
        # Use the correct BST stencil: neighbour is the node that completes the
        # triangle on the other side of each edge. For a uniform grid:
        # Edge n00-n10 (bottom): neighbour is node below → get_node(iy-1, ix) is n00-row-1
        # Edge n10-n01 (diagonal): neighbour is n11
        # Edge n01-n00 (left):  neighbour is get_node(iy, ix-1)
        nb_a0 = get_node(iy - 1, ix    )   # across edge n00–n10
        nb_a1 = n11                         # across diagonal n10–n01
        nb_a2 = get_node(iy,     ix - 1)   # across edge n01–n00

        ele_a = fea.ChElementShellBST()
        ele_a.SetNodes(n00, n10, n01, nb_a0, nb_a1, nb_a2)
        ele_a.AddLayer(THICKNESS, 0.0, material)
        mesh.AddElement(ele_a)

        # Triangle B: (n10, n11, n01)
        nb_b0 = get_node(iy,     ix + 2)   # across edge n10–n11 (right side)
        nb_b1 = get_node(iy + 2, ix    )   # across edge n11–n01 (top)
        nb_b2 = n00                         # across diagonal n01–n10

        ele_b = fea.ChElementShellBST()
        ele_b.SetNodes(n10, n11, n01, nb_b0, nb_b1, nb_b2)
        ele_b.AddLayer(THICKNESS, 0.0, material)
        mesh.AddElement(ele_b)

sys.Add(mesh)

# Required for shell elements before the loop
sys.Setup()
sys.Update()

# === FEA Visualization — two shapes (surface + node glyphs) ===
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
vis_surface.SetColorscaleMinMax(0.0, 0.3)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window (Y-up FEA scene) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Tablecloth Folding — FEA Shell Elements")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 0.8, 2.0), chrono.ChVector3d(0.5, 0.5, 0.0))
vis.AddTypicalLights()

# === Review-only setup ===


# Cache the free corner node (bottom-left: ix=0, iy=0) for logging
tip_node = nodes[0][0]  # cache: bottom-left free corner, fetched once

# === Main loop ===
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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad mesh state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
