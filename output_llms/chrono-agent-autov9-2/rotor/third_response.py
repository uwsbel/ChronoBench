"""Flexible spinning rotor (Jeffcott-style) driven by a time-varying motor speed.

Model
-----
A slender flexible shaft is modeled as a single IGA Cosserat beam (FEA) laid out
horizontally along the global Y axis. Both ends are pinned to fixed ground frames
(bearings) and the shaft is spun about its own axis by a rotational-speed motor
whose angular velocity follows a custom piecewise schedule defined by the
parameters A1, A2, T1, T2, T3 and w. Gravity acts along -Z, so the spinning,
gravity-loaded flexible shaft bends and whirls — the classic rotordynamics demo.

System
------
- ChSystemSMC (required for FEA stiffness matrices) with the Pardiso MKL direct
  solver and a linearized implicit Euler timestepper (robust for the stiff,
  high-spin beam dynamics where an adaptive integrator would stall).
- No contact / collision anywhere: the shaft is driven only by FEA elasticity,
  gravity, the bearing constraints and the motor, so SetCollisionSystemType is
  intentionally omitted (a pure FEA + jointed scene has nothing to collide).

Main bodies
-----------
- One fea.ChMesh holding the IGA Cosserat beam (ChNodeFEAxyzrot nodes).
- Two fixed ground bodies acting as the left/right bearings.
- One ChLinkMotorRotationSpeed driving the left end about the shaft axis, plus a
  revolute-style bearing link on the right end.

Expected behavior
------------------
The shaft accelerates and decelerates following the piecewise speed schedule; the
flexible mid-span deflects under gravity and whirls, with deflection growing as
the spin rate approaches the bending critical speed.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) ===
shaft_length = 2.0          # m, total span between bearings
shaft_diameter = 0.05       # m, circular cross-section diameter
shaft_density = 7800.0      # kg/m^3, steel
youngs_modulus = 2.0e11     # Pa, steel
poisson_ratio = 0.3
shear_modulus = youngs_modulus / (2.0 * (1.0 + poisson_ratio))
rayleigh_damping = 0.01     # structural damping (stabilizes HHT at high spin)
n_elements = 12             # IGA beam elements along the span
beam_order = 3              # cubic IGA (Cosserat) beam

# Motor speed-schedule parameters (rad/s amplitudes, s breakpoints, rad/s carrier).
A1 = 80.0
A2 = 160.0
T1 = 1.0
T2 = 3.0
T3 = 5.0
w = 6.0

time_step = 2.0e-4          # small step: HHT stays stable at high spin rates
sim_end = 6.0               # s, covers all schedule phases (through T3 and beyond)
render_fps = 50.0

# Derived layout: beam runs from A (-Y end) to B (+Y end), centered at origin.
beam_start = chrono.ChVector3d(0.0, -0.5 * shaft_length, 0.0)
beam_end = chrono.ChVector3d(0.0, 0.5 * shaft_length, 0.0)
y_dir = chrono.ChVector3d(0.0, 0.0, 1.0)   # cross-section reference direction


# === Custom motor speed schedule (ChFunction subclass) ===
class ChFunctionMyFun(chrono.ChFunction):
    """Piecewise angular-speed schedule for the rotor motor.

    GetVal(x) returns the commanded spin rate (rad/s) at time x:
      - ramp up to A1 during [0, T1)
      - hold/oscillate around A1 with an A-sized sinusoid during [T1, T2)
      - climb toward A2 during [T2, T3)
      - oscillate around A2 thereafter.
    """

    def __init__(self, a1, a2, t1, t2, t3, omega):
        super().__init__()
        self.a1 = a1
        self.a2 = a2
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.omega = omega

    def GetVal(self, x):
        if x < self.t1:
            return self.a1 * (x / self.t1)
        if x < self.t2:
            return self.a1 + 0.15 * self.a1 * math.sin(self.omega * (x - self.t1))
        if x < self.t3:
            frac = (x - self.t2) / (self.t3 - self.t2)
            return self.a1 + (self.a2 - self.a1) * frac
        return self.a2 + 0.10 * self.a2 * math.sin(self.omega * (x - self.t3))

    def Clone(self):
        return ChFunctionMyFun(self.a1, self.a2, self.t1, self.t2, self.t3, self.omega)


# === System & gravity === SMC + MKL direct solver + HHT (required for FEA beams)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
# FEA rotor: no contact material / collision system needed — the shaft is driven
# only by elasticity, gravity, bearings and the motor, so no bodies ever collide.
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# Linearized implicit Euler: one solve per step, no adaptive step shrinking, so
# the stiff spinning beam stays stable at the highest commanded spin rates where
# an adaptive integrator's step controller would collapse and abort.
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA mesh & beam section ===
# Strong references retained (mesh/builder/section) to avoid SWIG GC of nodes.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

inertia = fea.ChInertiaCosseratSimple()
inertia.SetAsCircularSection(shaft_diameter, shaft_density)

elasticity = fea.ChElasticityCosseratSimple()
elasticity.SetYoungModulus(youngs_modulus)
elasticity.SetShearModulus(shear_modulus)
elasticity.SetAsCircularSection(shaft_diameter)

damping = fea.ChDampingCosseratLinear()

section = fea.ChBeamSectionCosserat(inertia, elasticity, None, damping)
section.SetDrawCircularRadius(0.5 * shaft_diameter)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, section, n_elements, beam_start, beam_end, y_dir, beam_order)

# Keep strong references to the node container before indexing (SWIG GC guard).
beam_nodes = builder.GetLastBeamNodes()
node_left = beam_nodes.front()    # -Y bearing / motor end
node_right = beam_nodes.back()    # +Y bearing end
mid_index = beam_nodes.size() // 2
node_mid = beam_nodes[mid_index]  # mid-span: largest gravity deflection / whirl

sys.Add(mesh)

# === Bearings & motor (constraints + actuator) ===
# Left bearing ground + right bearing ground are fixed reference frames.
bearing_left = chrono.ChBody()
bearing_left.SetFixed(True)
bearing_left.SetPos(beam_start)
sys.Add(bearing_left)

bearing_right = chrono.ChBody()
bearing_right.SetFixed(True)
bearing_right.SetPos(beam_end)
sys.Add(bearing_right)

# Right end: a revolute that pins translation but frees the spin about Y.
# Motor rotates about its frame Z by convention, so align the link frame Z with Y.
spin_axis_quat = chrono.QuatFromAngleX(-chrono.CH_PI_2)  # map link-Z onto global +Y

bearing_link = chrono.ChLinkMateGeneric()
bearing_link.Initialize(node_right, bearing_right, False,
                        chrono.ChFramed(beam_end, spin_axis_quat),
                        chrono.ChFramed(beam_end, spin_axis_quat))
bearing_link.SetConstrainedCoords(True, True, True, True, True, False)  # free spin only
sys.Add(bearing_link)

# Left end: rotational-speed motor spinning the shaft about its Y axis.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_left, bearing_left,
                 chrono.ChFramed(beam_start, spin_axis_quat))
speed_fun = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
motor.SetSpeedFunction(speed_fun)
sys.Add(motor)

# === FEA visualization (surface + undeformed wireframe overlay) ===
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 0.02))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Flexible spinning rotor (FEA Cosserat beam)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.4, -2.4, 1.2), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.2, 0.2, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, -0.6), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === throttled render; physics advanced in an inner batch
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
get_time = sys.GetChTime          # cache: getter fetched once, reused every step
mid_pos = node_mid.GetPos         # cache: mid-span pose getter, reused every step


try:

    frame = 0
    while vis.Run() and get_time() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if get_time() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush CSV, assemble review video + plot, drop frame PNGs
