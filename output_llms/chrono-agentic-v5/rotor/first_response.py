"""Jeffcott rotor modelled with an Isogeometric Analysis (IGA) Cosserat beam.

Model: a flexible circular shaft is meshed as a single IGA beam (cubic NURBS).
A rigid flywheel is welded to the beam mid-node, and a rotational-speed motor
drives one beam end about the shaft axis while the opposite end is held in a
radial bearing. The system is ChSystemSMC (the FEA truth system type), solved
with a direct Pardiso-MKL solver and integrated with the HHT timestepper.

World convention: Y-up, gravity (0, -9.81, 0). This is a pure jointed FEA scene
(no rigid-body contact), so no collision system is configured.

Expected behaviour: the motor spins the shaft about its axis (X); the flywheel
mass off the elastic shaft produces the characteristic Jeffcott whirl/bending of
the beam, visualised with the FEM colour field and node glyphs in an Irrlicht
window.
"""

import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants === geometry / material / run parameters
time_step = 0.002          # IGA rotor timestep (stiff beam, direct solver)
sim_end = 6.0              # seconds of simulated rotor dynamics
render_fps = 50.0

beam_L = 1.0               # shaft length (m), spans X = 0 .. beam_L
beam_ro = 0.025           # shaft outer radius (m)
beam_density = 7800.0     # steel (kg/m^3)
beam_E = 210e9            # Young's modulus (Pa)
beam_nu = 0.3             # Poisson ratio
n_spans = 6               # IGA beam spans
iga_order = 3             # cubic NURBS

flywheel_mass = 2.0       # kg
flywheel_radius = 0.12    # m
flywheel_thick = 0.03     # m
motor_speed = 30.0        # rad/s drive speed about the shaft axis (X)

# Derived section properties for a solid circular shaft (precomputed once)
area = math.pi * beam_ro ** 2
Iyy = math.pi * beam_ro ** 4 / 4.0
Izz = Iyy
Jpolar = Iyy + Izz
beam_mid_x = beam_L * 0.5

# === System & gravity === SMC system, Y-up gravity, direct solver + HHT
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

sys.SetSolver(mkl.ChSolverPardisoMKL())            # direct solver for stiff IGA beam
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# Strong-reference holder to defend SWIG shared_ptrs from premature GC
keep = []

# === FEA mesh & IGA beam === Cosserat section built into a cubic IGA beam
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
keep.append(mesh)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(beam_density)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(beam_E)
melasticity.SetShearModulusFromPoisson(beam_nu)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(Jpolar)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)   # do NOT use SetAsCircularSection (overwrites Iyy/Izz/J)
keep.append(msection)

# FEA beam: no contact material needed — driven by constraints + gravity + motor only
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection,
                  n_spans,
                  chrono.ChVector3d(0, 0, 0),          # end A (driven)
                  chrono.ChVector3d(beam_L, 0, 0),     # end B (bearing)
                  chrono.VECT_Y,                       # suggested section Y direction
                  iga_order)
keep.append(builder)

# Keep a strong reference to the node container before indexing (SWIG GC guard)
beam_nodes = builder.GetLastBeamNodes()
n_nodes = beam_nodes.size()
node_A = beam_nodes.front()                         # driven end
node_B = beam_nodes.back()                           # bearing end
# Mid node: closest beam node to x = beam_L/2 carries the flywheel
node_mid = min((beam_nodes[i] for i in range(n_nodes)),
               key=lambda nd: abs(nd.GetPos().x - beam_mid_x))
keep.extend([node_A, node_B, node_mid])

sys.Add(mesh)

# === Bodies === rigid flywheel welded to the shaft centre
flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
fw_Ixx = 0.5 * flywheel_mass * flywheel_radius ** 2          # spin axis (X)
fw_Iyy = (1.0 / 12.0) * flywheel_mass * (3 * flywheel_radius ** 2 + flywheel_thick ** 2)
flywheel.SetInertiaXX(chrono.ChVector3d(fw_Ixx, fw_Iyy, fw_Iyy))
flywheel.SetPos(chrono.ChVector3d(beam_mid_x, 0, 0))
fw_shape = chrono.ChVisualShapeCylinder(flywheel_radius, flywheel_thick)
fw_shape.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
# Cylinder axis is local Z; rotate so its axis aligns with the shaft axis X
flywheel.AddVisualShape(fw_shape,
                        chrono.ChFramed(chrono.VNULL,
                                        chrono.QuatFromAngleY(math.pi / 2.0)))
sys.Add(flywheel)
keep.append(flywheel)

# === Joints / constraints === weld flywheel to mid-node; motor + bearing at the ends
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, flywheel)
sys.Add(weld)
keep.append(weld)

# Truss (fixed reference frame) for the motor stator and the far bearing
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)
keep.append(truss)

# Rotational-speed motor drives end A about the shaft axis (X)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_A, truss,
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                 chrono.QuatFromAngleY(math.pi / 2.0)))  # motor Z -> world X
motor.SetMotorFunction(chrono.ChFunctionConst(motor_speed))
sys.Add(motor)
keep.append(motor)

# Radial bearing at end B: constrain translations, leave rotations free
bearing = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
bearing.Initialize(node_B, truss, False, node_B.Frame(), node_B.Frame())
sys.Add(bearing)
keep.append(bearing)

# Pre-solve static step to settle the structure under gravity + motor load
sys.DoStaticLinear()

# === FEA visualisation === surface colour field + node-coordinate glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott rotor — IGA beam + flywheel")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.6, -1.2), chrono.ChVector3d(0.5, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(beam_mid_x, -0.3, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render-cadence loop; physics in inner batches
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

os.makedirs("cam", exist_ok=True)   # guard against missing output dir

node_mid_ref = node_mid          # cache: mid node fetched once, read every step
flywheel_ref = flywheel          # cache: flywheel body fetched once, read every step
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
except (RuntimeError, ValueError) as exc:     # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
