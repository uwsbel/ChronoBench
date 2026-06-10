"""Flexible slider-crank "buckling" mechanism with FEA Euler beams (PyChrono 9.0.1).

Model: a closed-loop flexible slider-crank. A rigid crank, hinged to a fixed
truss, is spun by a rotational-speed motor. The crank end carries a flexible
crank beam; that beam connects through a flexible vertical beam to a flexible
horizontal beam whose far end slides against the truss. As the crank rotates,
the slender vertical/horizontal beams are driven into compression and buckle.

System: ChSystemSMC (FEA requires SMC + a direct MKL solver). The mechanism is a
pure jointed/FEA loop with NO contact between bodies, so no collision system or
contact material is set up (none of the bodies collide; motion comes from the
motor, the FEA stiffness, and the joints).

Main bodies: a fixed truss (ChBody), a rigid crank (ChBody), and a ChMesh holding
three Euler-Bernoulli beams (crank beam, vertical beam, horizontal beam) joined by
ChLinkMateGeneric constraints. Expected behavior: the crank rotates steadily and
the flexible vertical/horizontal beams deflect and buckle under the imposed motion.
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Parameters === final geometry / section / sim constants (no bare literals downstream)
L = 1.2                 # horizontal beam length [m]
H = 0.3                 # vertical beam height [m]
K = 0.07                # crank length [m]

beam_density = 1000.0   # beam material density [kg/m^3]
beam_E = 0.02e10        # Young's modulus [Pa]
beam_G = beam_E * 0.38  # shear modulus [Pa]
beam_damp = 0.0001      # Rayleigh damping

hbeam_wy = 0.12         # horizontal beam section width, Y [m]
hbeam_wz = 0.012        # horizontal beam section width, Z [m]
vbeam_diam = 0.03       # vertical beam circular section diameter [m]
cbeam_diam = 0.054      # crank beam circular section diameter [m]

n_horizontal = 3        # Euler elements along horizontal beam
n_vertical = 6          # Euler elements along vertical beam
n_crank = 5             # Euler elements along crank beam

crank_speed = -math.pi  # imposed crank angular speed [rad/s]
glyph_scale = 0.015     # FEA node glyph symbol scale

time_step = 1.0e-3
sim_end = 6.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once

# === System & gravity === SMC + direct MKL solver (required by FEA stiffness matrices)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
# Closed-loop FEA mechanism: HHT can stall on the constraint loop, so use the
# linearized Euler-implicit integrator with the direct MKL solver for stability.
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# Strong references prevent SWIG from garbage-collecting FEA temporaries (segfault guard).
keepalive = []

# === Rigid bodies === fixed truss + the driven crank (origin at truss hinge)
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
truss.AddVisualShape(chrono.ChVisualShapeBox(0.03, 0.25, 0.12))
sys.Add(truss)

crank = chrono.ChBody()
crank.SetPos(chrono.ChVector3d(-K, 0, 0))
crank.SetName("crank")
crank.AddVisualShape(chrono.ChVisualShapeBox(K, 0.03, 0.03))
sys.Add(crank)

# === FEA mesh & sections === three Euler beams forming the flexible loop
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)
keepalive.append(mesh)


def make_section(circular=None, rect=None):
    """Build an Euler beam section (circular OR rectangular) and keep it alive."""
    sec = fea.ChBeamSectionEulerAdvanced()
    if circular is not None:
        sec.SetAsCircularSection(circular)
    else:
        sec.SetAsRectangularSection(rect[0], rect[1])
    sec.SetYoungModulus(beam_E)
    sec.SetShearModulus(beam_G)
    sec.SetDensity(beam_density)
    sec.SetRayleighDamping(beam_damp)
    keepalive.append(sec)
    return sec


def build_beam(section, n_elements, start, end, up):
    """Build one Euler beam with its OWN builder, capturing end nodes immediately.

    A dedicated builder per beam is essential: GetLastBeamNodes() rebinds to the
    most recent BuildBeam, so a shared builder would corrupt earlier node handles.
    Returns (front_node, back_node) and keeps every handle alive (SWIG GC guard).
    """
    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(mesh, section, n_elements, start, end, up)
    nodes = builder.GetLastBeamNodes()
    front = nodes.front()
    back = nodes.back()
    keepalive.extend([builder, nodes, front, back])
    return front, back


up_z = chrono.ChVector3d(0, 0, 1)   # lateral reference for in-plane beams
up_y = chrono.ChVector3d(0, 1, 0)   # lateral reference for the crank beam

# Horizontal beam: far slider end (X = L) back to the elbow at (0, H).
node_h_slider, node_h_elbow = build_beam(
    make_section(rect=(hbeam_wy, hbeam_wz)), n_horizontal,
    chrono.ChVector3d(L, 0, 0), chrono.ChVector3d(0, H, 0), up_z)

# Vertical beam: elbow (0, H) down to the base (0, 0).
node_v_top, node_v_bot = build_beam(
    make_section(circular=vbeam_diam), n_vertical,
    chrono.ChVector3d(0, H, 0), chrono.ChVector3d(0, 0, 0), up_z)

# Crank beam: crank pin (-K, 0) to the vertical-beam base (0, 0).
node_c_root, node_c_tip = build_beam(
    make_section(circular=cbeam_diam), n_crank,
    chrono.ChVector3d(-K, 0, 0), chrono.ChVector3d(0, 0, 0), up_y)

sys.Add(mesh)

# Sanity: confirm the captured node handles sit at their intended loop joints.
assert abs(node_h_slider.GetPos().x - L) < 1e-6, "slider node misplaced"
assert abs(node_h_elbow.GetPos().y - H) < 1e-6, "elbow node misplaced"
assert abs(node_v_bot.GetPos().y) < 1e-6 and abs(node_c_tip.GetPos().y) < 1e-6, "base nodes misplaced"

# === Joints / constraints === motor + loop-closing mate constraints
# FEA beams: no contact material needed — driven by the motor, gravity, and joints only.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, truss,
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetMotorFunction(chrono.ChFunctionConst(crank_speed))
sys.Add(motor)
keepalive.append(motor)

# Weld the crank beam root to the rigid crank body.
mate_crank = chrono.ChLinkMateGeneric()
mate_crank.Initialize(node_c_root, crank, False, node_c_root.Frame(), node_c_root.Frame())
mate_crank.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(mate_crank)
keepalive.append(mate_crank)

# Join crank-beam tip to vertical-beam base.
mate_cv = chrono.ChLinkMateGeneric()
mate_cv.Initialize(node_c_tip, node_v_bot, False, node_c_tip.Frame(), node_c_tip.Frame())
mate_cv.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(mate_cv)
keepalive.append(mate_cv)

# Join vertical-beam top to horizontal-beam elbow.
mate_vh = chrono.ChLinkMateGeneric()
mate_vh.Initialize(node_v_top, node_h_elbow, False, node_v_top.Frame(), node_v_top.Frame())
mate_vh.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(mate_vh)
keepalive.append(mate_vh)

# Slider: the horizontal beam far end is constrained to the truss (free along X).
mate_slider = chrono.ChLinkMateGeneric()
mate_slider.Initialize(node_h_slider, truss, False, node_h_slider.Frame(), node_h_slider.Frame())
mate_slider.SetConstrainedCoords(False, True, True, True, True, True)
sys.Add(mate_slider)
keepalive.append(mate_slider)

# Constraint visualization markers (size 0.012 generic, 0.014 crank/vertical join).
truss.AddVisualShape(chrono.ChVisualShapeSphere(0.012),
                     chrono.ChFramed(chrono.ChVector3d(L, 0, 0)))
truss.AddVisualShape(chrono.ChVisualShapeSphere(0.014),
                     chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# === FEA visualization === colored bending-moment surface + node-frame glyphs
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColormapRange(chrono.ChVector2d(-0.4, 0.4))
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)
keepalive.append(vis_beam)

vis_nodes = chrono.ChVisualShapeFEA()
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetSymbolsThickness(0.004)
vis_nodes.SetSymbolsScale(glyph_scale)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)
keepalive.append(vis_nodes)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y here
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Flexible slider-crank buckling")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(0.3, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.4, 0),
                               chrono.Q_ROTATE_Y_TO_Z),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics + logging in the inner batch

# cache: node handles fetched once, reused every logged step
slider_node = node_h_slider     # cache: slider position source
vbot_node = node_v_bot          # cache: vertical-beam base position source


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

# === Post-processing === assemble review video + timeseries plot, then clean frames
