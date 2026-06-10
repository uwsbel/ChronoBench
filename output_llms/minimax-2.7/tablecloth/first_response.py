#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PyChrono simulation: folding of a table‑cloth using shell elements.

* Isotropic Kirchhoff material (thin‑plate theory)
* Grid of quadrilateral shell elements (20 × 20 elements)
* Four corners of the cloth are fixed; a small downward force is applied at the centre
* Gravity is active (‑9.81 m s⁻²)
* PardisoMKL sparse solver is used for the linear system
* Irrlicht provides a real‑time 3‑D view
"""

# ----------------------------------------------------------------------
# 1️⃣  Core imports
# ----------------------------------------------------------------------
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# ----------------------------------------------------------------------
# 2️⃣  Create a physical system and set global parameters
# ----------------------------------------------------------------------
sys = chrono.ChSystemSMC()                     # SMC = smooth‑mesh contact (default for shells)
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))   # gravity

# Time step and integration scheme
dt = 0.005                                      # seconds
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)

# ----------------------------------------------------------------------
# 3️⃣  Solver selection – PardisoMKL (Intel MKL) for fast sparse factorisation
# ----------------------------------------------------------------------
sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)

# ----------------------------------------------------------------------
# 4️⃣  Create a finite‑element mesh
# ----------------------------------------------------------------------
mesh = fea.ChMesh()
sys.Add(mesh)

# ----------------------------------------------------------------------
# 5️⃣  Material – isotropic Kirchhoff (thin plate)  E = 1e7 Pa, ν = 0.3, thickness = 1 mm
# ----------------------------------------------------------------------
mat = fea.ChMaterialShellIsotropic(1e7, 0.3, 0.001)

# ----------------------------------------------------------------------
# 6️⃣  Generate a rectangular grid of nodes and shell elements
# ----------------------------------------------------------------------
width  = 2.0    # size along X (metres)
length = 2.0    # size along Y (metres)
nx     = 20     # divisions along X
ny     = 20     # divisions along Y

# ---- Nodes ------------------------------------------------------------
nodes = []
for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * width / nx
        y = j * length / ny
        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, 0.0))
        node.SetFixed(False)          # all nodes start free
        mesh.AddNode(node)
        nodes.append(node)

# ---- Elements (Kirchhoff shells) --------------------------------------
for i in range(nx):
    for j in range(ny):
        # Four corner nodes of the quadrilateral element
        n0 = nodes[i * (ny + 1) + j]
        n1 = nodes[(i + 1) * (ny + 1) + j]
        n2 = nodes[(i + 1) * (ny + 1) + (j + 1)]
        n3 = nodes[i * (ny + 1) + (j + 1)]

        elem = fea.ChElementShellKirchhoff()
        elem.SetMaterial(mat)
        elem.SetNodes(n0, n1, n2, n3)
        mesh.AddElement(elem)

# ----------------------------------------------------------------------
# 7️⃣  Boundary conditions – fix the four corners of the table‑cloth
# ----------------------------------------------------------------------
tol = 1e-4   # tolerance for corner detection
for node in nodes:
    p = node.GetPos()
    if (abs(p.x)          < tol and abs(p.y)          < tol) or \
       (abs(p.x - width)  < tol and abs(p.y)          < tol) or \
       (abs(p.x)          < tol and abs(p.y - length) < tol) or \
       (abs(p.x - width)  < tol and abs(p.y - length) < tol):
        node.SetFixed(True)

# ----------------------------------------------------------------------
# 8️⃣  Optional: apply a small downward force at the centre to start a fold
# ----------------------------------------------------------------------
centre_node = None
for node in nodes:
    p = node.GetPos()
    if abs(p.x - width / 2) < tol and abs(p.y - length / 2) < tol:
        centre_node = node
        break

if centre_node:
    force = chrono.ChForce()
    force.SetMod(5.0)                                 # magnitude 5 N
    force.SetDir(chrono.ChVectorD(0, 0, -1))         # downward
    centre_node.AddForce(force)

# ----------------------------------------------------------------------
# 9️⃣  Irrlicht visualisation
# ----------------------------------------------------------------------
app = irr.ChIrrApp(sys,
                    "Table‑cloth folding (Kirchhoff shells)",
                    irr.dimension2d_u32(1280, 720))

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(irr.vector3df(2.0, 2.0, 3.0),
                     irr.vector3df(1.0, 1.0, 0.0))
app.AddTypicalLights()

# Tell Irrlicht how to render the mesh (surface shading)
app.AddMesh(irr.ChIrrNodeMeshAdapter(mesh))

# ----------------------------------------------------------------------
# 🔟  Main simulation loop
# ----------------------------------------------------------------------
app.SetStep(dt)          # timestep
app.SetFPS(30)            # desired frame rate (for visualisation)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    app.DoStep()         # advance the physics

# ----------------------------------------------------------------------
# End of script
# ----------------------------------------------------------------------