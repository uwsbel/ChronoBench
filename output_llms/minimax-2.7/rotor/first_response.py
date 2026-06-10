#!/usr/bin/env python3
"""
Jeffcott‑rotor simulation with an isogeometric (IGA) beam.

Features:
* IGA beam (cubic B‑spline) representing the flexible shaft.
* Flywheel (rigid disc) attached at the beam centre.
* Rotational motor driving the left end of the beam.
* FEM visualisation via Irrlicht.
"""

# ------------------------------------------------------------
# 1)  Imports
# ------------------------------------------------------------
import pychrono as chrono
from chrono.core import ChVectorD, ChQuaternionD, ChFrameD, ChFunction_Const
import pychrono.fem as fem
import pychrono.irrlicht as irr

# ------------------------------------------------------------
# 2)  Create the physical system and set basic parameters
# ------------------------------------------------------------
system = chrono.ChSystemSMC()

# No gravity – a pure rotor (gravity can be added if desired)
system.Set_G_acc(ChVectorD(0.0, 0.0, 0.0))

# Solver settings (required for the flexible element)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(200)

# Time step – must be small for a flexible beam
dt = 1.0e-5
system.SetStepSize(dt)

# ------------------------------------------------------------
# 3)  Build the IGA beam (shaft)
# ------------------------------------------------------------
# Beam parameters
L_beam = 1.0                # length of the shaft (m)
n_control_points = 11       # cubic B‑spline → 4 control points per element
dx = L_beam / (n_control_points - 1)

# --- 3.1) Isogeometric mesh container ------------------------
iga_mesh = fem.ChIsogeometricMesh()

# --- 3.2) Knot vector (cubic B‑spline, open knot) -----------
kv = fem.ChKnotVector()
kv.SetKnots([0, 0, 0, 0,
             0.25, 0.5, 0.75,
             1, 1, 1, 1])
iga_mesh.SetKnotVector(kv)

# --- 3.3) Control‑point nodes (they will be the beam nodes) --
nodes = []
for i in range(n_control_points):
    pos = ChVectorD(i * dx, 0.0, 0.0)
    node = fem.ChNodeFEMbody(ChFrameD(pos, ChQuaternionD(1, 0, 0, 0)))  # identity orientation
    # small nodal mass – the element uses the section density
    node.SetMass(0.01)
    iga_mesh.AddControlPoint(node)   # register as a control point
    nodes.append(node)

# --- 3.4) Beam element (IGA) ---------------------------------
beam_elem = fem.ChElementBeamIGA()
beam_elem.SetMesh(iga_mesh)

# --- 3.5) Beam section (circular) ----------------------------
section = fem.ChBeamSectionCircular()
section.SetDiameter(0.05)            # 5 cm shaft
section.SetYoungModulus(2.1e11)     # steel
section.SetShearModulus(8.0e10)
section.SetDensity(7850.0)           # kg/m³
beam_elem.SetSection(section)

# --- 3.6) Add the whole mesh to the system --------------------
system.Add(iga_mesh)

# ------------------------------------------------------------
# 4)  Flywheel (rigid disc) at the centre of the beam
# ------------------------------------------------------------
flywheel = chrono.ChBody()
flywheel.SetMass(5.0)                              # kg
flywheel.SetInertiaXX(ChVectorD(0.04, 0.04, 0.02))  # principal moments
flywheel.SetPos(ChVectorD(L_beam / 2.0, 0.0, 0.0))

# Visual shape (sphere – cheap to render)
sph = chrono.ChSphereShape(0.15)
flywheel.AddShape(sph, ChFrameD())
system.AddBody(flywheel)

# --- 4.1) Attach the flywheel to the middle control point ----
mid_node = nodes[len(nodes) // 2]          # centre node
flywheel_joint = chrono.ChLinkLock()
flywheel_joint.Initialize(flywheel, mid_node,
                          ChFrameD(), ChFrameD())   # default frames → lock relative motion
system.AddLink(flywheel_joint)

# ------------------------------------------------------------
# 5)  Motor at the left end (ground → beam)
# ------------------------------------------------------------
# --- 5.1) Fixed ground body ----------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(ChVectorD(0.0, 0.0, 0.0))
system.AddBody(ground)

# --- 5.2) Revolute joint between ground and left node -------
motor_joint = chrono.ChLinkRevolute()
motor_joint.Initialize(ground, nodes[0],
                       ChFrameD(ChVectorD(0.0, 0.0, 0.0), ChQuaternionD(1, 0, 0, 0)))
system.AddLink(motor_joint)

# --- 5.3) Enable motor and set a constant rotational speed ----
motor_joint.SetMotorEnable(True)
motor_joint.SetMotorSpeed(ChFunction_Const(100.0))   # rad/s (≈ 955 rev/min)

# ------------------------------------------------------------
# 6)  Visualisation (Irrlicht)
# ------------------------------------------------------------
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor – IGA Beam")
vis.AddLight(irr.ChVectorf(0, 10, 0), 5.0)
vis.AddCamera(irr.ChVectorf(-2, -1, -2), irr.ChVectorf(0.5, 0, 0))
vis.Initialize()

# ------------------------------------------------------------
# 7)  Simulation loop
# ------------------------------------------------------------
time = 0.0
end_time = 2.0                     # simulate 2 s (change as needed)

print("Starting simulation …")
while vis.Run():
    if time >= end_time:
        break

    system.DoStepDynamics(dt)
    time += dt

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

print(f"Simulation finished at t = {time:.3f} s")