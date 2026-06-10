"""Jeffcott rotor modeled with a PyChrono IGA beam.

This SMC finite-element simulation builds a flexible Cosserat/IGA shaft, welds a
rigid flywheel to its center node, supports the shaft ends through motor/bearing
constraints, and drives the left end with a prescribed rotational speed. The
Irrlicht window renders FEM deformation and flywheel motion while the shaft spins
and bends under gravity.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants: define rotor geometry and time scales ===
SHAFT_LENGTH = 1.2
SHAFT_RADIUS_OUTER = 0.018
SHAFT_RADIUS_INNER = 0.010
SHAFT_DENSITY = 7800.0
SHAFT_YOUNG_MODULUS = 2.1e11
SHAFT_POISSON = 0.30
SHAFT_AREA = math.pi * (SHAFT_RADIUS_OUTER**2 - SHAFT_RADIUS_INNER**2)
SHAFT_IYY = math.pi * (SHAFT_RADIUS_OUTER**4 - SHAFT_RADIUS_INNER**4) / 4.0
SHAFT_IZZ = SHAFT_IYY
SHAFT_J = 2.0 * SHAFT_IYY
BEAM_SPANS = 24
IGA_ORDER = 3

FLYWHEEL_RADIUS = 0.14
FLYWHEEL_WIDTH = 0.050
FLYWHEEL_DENSITY = 7800.0
DRIVE_SPEED = 80.0
TIME_STEP = 0.002
SIM_END = 2.0

LEFT_END = chrono.ChVector3d(0.0, 0.0, 0.0)
RIGHT_END = chrono.ChVector3d(SHAFT_LENGTH, 0.0, 0.0)
CENTER_POS = chrono.ChVector3d(0.5 * SHAFT_LENGTH, 0.0, 0.0)
SHAFT_AXIS_FRAME = chrono.ChFramed(LEFT_END, chrono.Q_ROTATE_Z_TO_X)


# === System & solver: use direct FEA-capable integration ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Fixed reference: provide a frame for motor and bearings ===
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.AddBody(truss)


# === IGA shaft: build a hollow circular Cosserat beam ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

shaft_inertia = fea.ChInertiaCosseratSimple()
shaft_inertia.SetDensity(SHAFT_DENSITY)
shaft_inertia.SetArea(SHAFT_AREA)
shaft_inertia.SetIyy(SHAFT_IYY)
shaft_inertia.SetIzz(SHAFT_IZZ)

shaft_elasticity = fea.ChElasticityCosseratSimple()
shaft_elasticity.SetYoungModulus(SHAFT_YOUNG_MODULUS)
shaft_elasticity.SetShearModulusFromPoisson(SHAFT_POISSON)
shaft_elasticity.SetArea(SHAFT_AREA)
shaft_elasticity.SetIyy(SHAFT_IYY)
shaft_elasticity.SetIzz(SHAFT_IZZ)
shaft_elasticity.SetJ(SHAFT_J)

shaft_section = fea.ChBeamSectionCosserat(shaft_inertia, shaft_elasticity)
shaft_section.SetCircular(True)
shaft_section.SetDrawCircularRadius(SHAFT_RADIUS_OUTER)

beam_builder = fea.ChBuilderBeamIGA()
beam_builder.BuildBeam(mesh, shaft_section, BEAM_SPANS, LEFT_END, RIGHT_END, chrono.VECT_Y, IGA_ORDER)
beam_node_container = beam_builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]  # cache: reused for links/logs
left_node = beam_nodes[0]  # cache: left driven support node
center_node = beam_nodes[len(beam_nodes) // 2]  # cache: flywheel attachment node
right_node = beam_nodes[-1]  # cache: right bearing support node

# FEA beam: no contact material needed; constraints, gravity, and motor drive the rotor.


# === Flywheel and spindle: rigid bodies coupled to beam nodes ===
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, FLYWHEEL_RADIUS, FLYWHEEL_WIDTH, FLYWHEEL_DENSITY)
flywheel.SetPos(CENTER_POS)
flywheel.SetRot(chrono.Q_ROTATE_Z_TO_X)
flywheel.EnableCollision(False)
sys.AddBody(flywheel)

drive_spindle = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.035, 0.050, FLYWHEEL_DENSITY)
drive_spindle.SetPos(LEFT_END)
drive_spindle.SetRot(chrono.Q_ROTATE_Z_TO_X)
drive_spindle.EnableCollision(False)
sys.AddBody(drive_spindle)


# === Constraints & motor: attach shaft, flywheel, bearing, and drive ===
flywheel_weld = chrono.ChLinkMateFix()
flywheel_weld.Initialize(center_node, flywheel)
sys.Add(flywheel_weld)

spindle_weld = chrono.ChLinkMateFix()
spindle_weld.Initialize(left_node, drive_spindle)
sys.Add(spindle_weld)

right_bearing = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
right_bearing.Initialize(right_node, truss, False, right_node.Frame(), right_node.Frame())
sys.Add(right_bearing)

drive_motor = chrono.ChLinkMotorRotationSpeed()
drive_motor.Initialize(drive_spindle, truss, SHAFT_AXIS_FRAME)
drive_motor.SetSpeedFunction(chrono.ChFunctionConst(DRIVE_SPEED))
sys.Add(drive_motor)


# === FEM visualization: surface field plus node coordinate glyphs ===
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.004)
vis_glyph.SetSymbolsScale(0.02)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)
sys.Setup()
sys.Update()
sys.DoStaticLinear()


# === Irrlicht visualization: initialize first, then add scene elements ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono IGA Jeffcott Rotor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.65, 0.55, 1.05), chrono.ChVector3d(0.60, 0.0, 0.0))
vis.AddTypicalLights()
grid_frame = chrono.ChCoordsysd(chrono.ChVector3d(0.6, -0.28, 0.0), chrono.Q_ROTATE_Z_TO_Y)
vis.AddGrid(0.10, 0.10, 16, 16, grid_frame, chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop: render, optionally record review data, and advance dynamics ===

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        current_time = sys.GetChTime()  # cache: reused for logging and stopping
        center_pos = center_node.GetPos()  # cache: central shaft response
        flywheel_angvel = flywheel.GetAngVelParent()  # cache: flywheel speed readout

        sys.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError, OSError) as exc:
    # RuntimeError/ValueError guard solver divergence; OSError guards review capture I/O.
    traceback.print_exc()
    raise
finally:
    pass
