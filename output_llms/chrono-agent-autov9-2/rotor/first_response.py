"""Jeffcott rotor on a flexible IGA Cosserat beam (PyChrono 9.0.1, ChSystemSMC).

Models the lateral whirl dynamics of a Jeffcott rotor: a slender circular shaft
discretized as an Isogeometric-Analysis (IGA) Cosserat beam, with a rigid
flywheel rigidly clamped to the beam's mid-span node and a rotational-speed motor
that spins one end of the shaft about its axis. A small mass-eccentric (unbalance)
body is offset from the disk center so that the spin excites a synchronous whirl
of the shaft mid-point.

System type: ChSystemSMC (FEA requires SMC + a stable implicit timestepper).
Main bodies:
  * IGA Cosserat beam shaft (fea.ChMesh, ChNodeFEAxyzrot nodes) along world X.
  * rigid flywheel disk clamped to the mid-span node.
  * rigid eccentric ("unbalance") mass offset from the disk, clamped to the disk.
  * fixed ground/truss body that the spin motor reacts against.
Expected behavior: the motor ramps the shaft to a constant spin rate; the
unbalance drives a growing-then-bounded lateral deflection (whirl) of the
mid-span node, visible as the beam bowing while it rotates. There is no contact
in this scene, so no collision system / contact material is configured.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics (no bare position literals downstream)
time_step = 2.5e-4               # FEA-stable implicit step for the stiff steel shaft
sim_end = 4.0                    # seconds of simulated rotor spin-up + whirl
render_fps = 30.0

beam_length = 1.0                # shaft span between bearings [m]
beam_diameter = 0.012            # slender circular shaft [m]
beam_E = 2.0e11                  # Young's modulus, steel [Pa]
beam_G = 7.7e10                  # shear modulus, steel [Pa]
beam_density = 7800.0            # steel density [kg/m^3]
beam_elements = 10               # IGA spans
beam_rayleigh_beta = 0.02        # Rayleigh (stiffness-proportional) damping

flywheel_mass = 3.0              # disk mass [kg]
flywheel_radius = 0.10           # disk radius [m]
flywheel_thick = 0.02            # disk axial thickness [m]

unbalance_mass = 0.05            # eccentric point mass [kg]
unbalance_ecc = 0.10             # radial eccentricity from shaft axis (disk rim) [m]
unbalance_size = 0.05            # visible marker edge for the unbalance body [m]

spin_speed = 20.0                # commanded constant spin rate [rad/s]

# Derived positions (shaft along +X, gravity along -Z, mid-span at the origin)
x_start = -0.5 * beam_length
x_end = 0.5 * beam_length
beam_dir_y = chrono.ChVector3d(0, 1, 0)   # cross-section reference direction
mid_x = 0.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# Strong references that must outlive the build to defeat SWIG garbage collection.
keepalive = []

# === System, solver & gravity === SMC + MKL direct solver; implicit timestepper; no contact
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# FEA stiffness matrices require a direct solver; iterative solvers diverge here.
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
keepalive.append(solver)
# HHT is the canonical implicit integrator for slender FEA beams, but on this stiff
# Jeffcott rotor it stalls at the minimum step; EULER_IMPLICIT_LINEARIZED is the
# robust fallback and integrates the whirl cleanly with the MKL direct solve.
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA mesh & IGA Cosserat beam === slender shaft built with ChBuilderBeamIGA
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
keepalive.append(mesh)

# Circular Cosserat section (Easy* sets elasticity + inertia from diameter/E/G/rho).
# FEA beam: no contact material needed — driven by constraints + gravity + motor only.
sec = fea.ChBeamSectionCosseratEasyCircular(beam_diameter, beam_E, beam_G, beam_density)
# Rayleigh damping bleeds off numerical ringing; built from the section's elasticity.
damping = fea.ChDampingCosseratRayleigh(sec.GetElasticity(), beam_rayleigh_beta)
sec.SetDamping(damping)
sec.SetDrawCircularRadius(0.5 * beam_diameter)
keepalive.extend([sec, damping])

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, sec, beam_elements,
    chrono.ChVector3d(x_start, 0, 0),    # shaft start (driven end)
    chrono.ChVector3d(x_end, 0, 0),      # shaft end (free bearing)
    beam_dir_y,                          # cross-section Y reference
    3,                                   # cubic B-spline order
)
keepalive.append(builder)

# Cache the node container once (SWIG temp would dangle if indexed inline).
beam_nodes_container = builder.GetLastBeamNodes()           # cache: SWIG GC guard
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
keepalive.append(beam_nodes_container)
node_drive = beam_nodes[0]                  # driven end node (ChNodeFEAxyzrot)
node_free = beam_nodes[-1]                  # opposite bearing node
# Mid-span node = node closest to x=mid_x; carries the flywheel.
node_mid = min(beam_nodes, key=lambda n: abs(n.GetPos().x - mid_x))

sys.Add(mesh)

# === Ground / bearings === fixed truss the motor and end bearings react against
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.Add(ground)

# Pin the free end laterally (a simple bearing): constrain its translation to the
# ground point but leave the axial spin free, so the shaft can whirl but not fly off.
bearing_free = chrono.ChLinkMateGeneric()
bearing_free.Initialize(node_free, ground, False, node_free.Frame(), node_free.Frame())
bearing_free.SetConstrainedCoords(True, True, True, False, False, False)  # lock XYZ, free rot
bearing_free.SetName("bearing_free")
sys.Add(bearing_free)

# === Flywheel disk === rigid disk clamped to the mid-span node (Jeffcott mass)
flywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_X, flywheel_radius, flywheel_thick, flywheel_mass / (
        math.pi * flywheel_radius * flywheel_radius * flywheel_thick),
    True, False)
flywheel.SetPos(node_mid.GetPos())
flywheel.SetName("flywheel")
# Tint the disk so its spin (carried by the rim unbalance marker) reads on screen.
flywheel.GetVisualShape(0).SetColor(chrono.ChColor(0.30, 0.45, 0.70))
sys.Add(flywheel)

# Rigidly weld the disk to the mid-span node (all 6 DOF) — the disk rides the beam.
weld_disk = chrono.ChLinkMateGeneric()
weld_disk.Initialize(flywheel, node_mid, False, node_mid.Frame(), node_mid.Frame())
weld_disk.SetConstrainedCoords(True, True, True, True, True, True)
weld_disk.SetName("weld_disk")
sys.Add(weld_disk)

# === Unbalance === eccentric mass offset from the disk realizes rotor imbalance
# (9.0.1 has no ChBody COM-frame setter, so the eccentricity is a real offset body).
unbalance = chrono.ChBodyEasyBox(unbalance_size, unbalance_size, unbalance_size,
                                 unbalance_mass / (unbalance_size ** 3), True, False)
unbalance.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, unbalance_ecc, 0))
unbalance.SetName("unbalance")
# Bright marker on the disk rim — its orbit makes the shaft spin unmistakable.
unbalance.GetVisualShape(0).SetColor(chrono.ChColor(0.90, 0.15, 0.10))
sys.Add(unbalance)

weld_unbalance = chrono.ChLinkMateGeneric()
weld_unbalance.Initialize(unbalance, flywheel, True,
                          chrono.ChFramed(chrono.ChVector3d(0, unbalance_ecc, 0)),
                          chrono.ChFramed(chrono.ChVector3d(0, unbalance_ecc, 0)))
weld_unbalance.SetConstrainedCoords(True, True, True, True, True, True)
weld_unbalance.SetName("weld_unbalance")
sys.Add(weld_unbalance)

# === Motor === constant-speed rotational drive spinning the driven end about X
motor = chrono.ChLinkMotorRotationSpeed()
# ChNodeFEAxyzrot IS a ChBodyFrame, so the motor attaches to the beam node directly.
motor.Initialize(node_drive, ground,
                 chrono.ChFramed(node_drive.GetPos(),
                                 chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z-> world X
motor.SetSpeedFunction(chrono.ChFunctionConst(spin_speed))
motor.SetName("spin_motor")
sys.Add(motor)

# === FEM visualization === colored deformation + undeformed wireframe reference
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 4.0))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)
keepalive.extend([vis_beam, vis_wire])

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott rotor - IGA Cosserat beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.55, -0.65, 0.30), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.3), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics advanced in inner batches


frame = 0
try:
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
