"""Jeffcott-style flexible rotor simulation (PyChrono 9.0.1, FEA + Irrlicht).

Models a slender flexible shaft as an IGA Cosserat beam (ChBuilderBeamIGA +
ChBeamSectionCosseratEasyCircular), discretized into ChNodeFEAxyzrot nodes. A
heavy rigid flywheel is rigidly clamped to the shaft mid-span; because the
flywheel center of mass is offset from the elastic axis, the spinning system
develops a self-excited whirl (Jeffcott rotor). One shaft end is driven about
its longitudinal (Y) axis by a rotational-speed motor whose target speed is a
sine function of time; the opposite end is supported by a revolute-style fixed
node so the shaft can bend laterally while spinning.

System type: ChSystemSMC (FEA stiffness/inertia require the SMC system + a
direct/structural-friendly timestepper). There is NO mechanical contact: the
beam is driven only by the motor, gravity, and internal elastic/damping forces,
so no contact material or collision surface is defined.

Expected behavior: the shaft spins up following the sinusoidal speed target,
bends under gravity and the offset-flywheel unbalance, and the mid-span node
traces a growing/decaying whirl orbit in the X-Z plane transverse to the spin
axis.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) ===
# Shaft (slender flexible beam) — modeled as a solid circular Cosserat section
# sized to the outer radius of the physical hollow tube.
beam_L = 10.0                 # shaft length (m), aligned with the Y (spin) axis
beam_ro = 0.060               # shaft outer radius (m)
beam_ri = 0.055               # shaft inner radius (m) — physical hollow tube
beam_diameter = 2.0 * beam_ro # Cosserat easy-circular section uses a diameter
n_beam_elements = 6           # IGA beam elements along the shaft
beam_order = 3                # IGA (B-spline) order — cubic

beam_E = 2.1e11               # Young's modulus (Pa), steel shaft
beam_nu = 0.3                 # Poisson ratio
beam_G = beam_E / (2.0 * (1.0 + beam_nu))   # shear modulus (Pa)
beam_density = 7800.0         # shaft density (kg/m^3), steel
rayleigh_beta = 0.002         # stiffness-proportional Rayleigh damping

# Flywheel (rigid unbalanced disk clamped at shaft mid-span).
flywheel_radius = 0.30        # flywheel radius (m)
flywheel_height = 0.10        # flywheel axial thickness (m)
flywheel_density = 7800.0     # flywheel density (kg/m^3)
flywheel_offset = 0.0015      # COM offset from elastic axis (m) -> unbalance

# Motor: target spin speed is a sine of time (rad/s).
motor_amplitude = 60.0        # sine amplitude (rad/s)
motor_frequency = 0.1         # sine frequency (Hz)

gravity_y = -3.71             # reduced-gravity environment (m/s^2), along -Y

# Shaft endpoint and mid-span coordinates (derived once; no bare literals later).
shaft_y0 = 0.0                          # driven end (motor) Y
shaft_y1 = beam_L                       # supported end Y
shaft_mid_y = 0.5 * (shaft_y0 + shaft_y1)  # flywheel attachment Y
beam_start = chrono.ChVector3d(0.0, shaft_y0, 0.0)
beam_end = chrono.ChVector3d(0.0, shaft_y1, 0.0)
beam_ydir = chrono.ChVector3d(0, 0, 1)  # section reference (transverse) direction

# Time stepping / rendering.
time_step = 2e-4              # small step for FEA stability of a driven rotor
sim_end = 6.0                 # simulated duration (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
cam_eye = chrono.ChVector3d(0, 2, 8)    # camera position (better view of long beam)
cam_target = chrono.ChVector3d(0, shaft_mid_y, 0)  # look at shaft mid-span

# === System & gravity === SMC system for FEA; reduced gravity along -Y.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, gravity_y, 0))
# Collision system intentionally omitted: this is a pure FEA + jointed rotor with
# NO contact (beam driven by motor + gravity + internal elastic forces only).

# === FEA mesh & shaft (IGA Cosserat beam) ===
# Keep strong references to mesh/builder/section so SWIG does not GC the nodes.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionCosseratEasyCircular(beam_diameter, beam_E, beam_G, beam_density)
# Cosserat Rayleigh damping built from the section's own elasticity model.
damping = fea.ChDampingCosseratRayleigh(section.GetElasticity(), rayleigh_beta)
section.SetDamping(damping)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, section, n_beam_elements, beam_start, beam_end, beam_ydir, beam_order)

# Store the node container BEFORE indexing (SWIG GC pitfall: a temporary
# container is collected and the node shared_ptrs dangle -> segfault).
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
node_driven = beam_nodes[0]                # motor-driven (bearing) end
# Mid-span node = flywheel clamp; pick the node closest to shaft_mid_y.
node_mid = min(beam_nodes, key=lambda nd: abs(nd.GetPos().y - shaft_mid_y))

sys.Add(mesh)

# === Flywheel (rigid unbalanced disk) ===
# ChBodyEasyCylinder with its axis along Y matches the shaft spin axis.
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, flywheel_radius, flywheel_height,
                                     flywheel_density)
flywheel.SetPos(chrono.ChVector3d(flywheel_offset, shaft_mid_y, 0.0))  # COM offset -> unbalance
sys.Add(flywheel)

# Rigidly clamp the flywheel to the mid-span beam node (all 6 DOF).
clamp = chrono.ChLinkMateFix()
clamp.Initialize(flywheel, node_mid)
sys.Add(clamp)

# === Support & motor (driven cantilever rotor topology, no contact) ===
# Ground/truss body: a fixed reference the motor reacts against.
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Driven end: a rotational-speed motor about Y with a sinusoidal target speed.
# The motor locks the driven node to the truss in all DOF except the actuated
# rotation, so it is the rotor's single bearing; the far end is left free to
# whirl (a classic flexible Jeffcott rotor). The motor frame is rotated so its
# actuated Z axis points along world +Y (the shaft/spin axis).
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_driven, truss,
                 chrono.ChFramed(node_driven.GetPos(),
                                 chrono.QuatFromAngleX(-chrono.CH_PI_2)))  # Z -> +Y
speed_func = chrono.ChFunctionSine(motor_amplitude, motor_frequency)
motor.SetSpeedFunction(speed_func)
sys.Add(motor)

# === FEA visualization === color the shaft by node speed; wireframe reference.
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 8.0))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)

# === Solver & timestepper === direct solver + linearized stepper for the rotor.
# FEA stiffness matrices require a direct (Pardiso/MKL) solver; the default
# iterative solver diverges on the stiff beam. A driven FEA rotor also stalls
# HHT iterations, so use the linearized Euler-implicit stepper, which integrates
# the stiff beam + motor constraint robustly.
sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y is up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Flexible Jeffcott Rotor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(cam_eye, cam_target)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid

# === Main loop === drive the rotor, render at fixed cadence, log whirl + speed.

# cache: fetch the mid-span node + motor once; reused every logged step.
mid_node = node_mid                       # cache: flywheel/whirl probe node
motor_link = motor                        # cache: motor handle reused each step

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
