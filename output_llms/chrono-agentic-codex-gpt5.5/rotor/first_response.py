"""Jeffcott rotor modeled with a PyChrono SMC FEA system.

The simulation builds an IGA Cosserat shaft, welds a rigid flywheel and small
eccentric mass to the shaft center, and drives the left end with a
prescribed-speed rotational motor. HHT time integration and Pardiso MKL solve
the beam dynamics while Irrlicht displays the deforming beam and rotating
flywheel.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === geometry, material, and loop values are fixed once for repeatable rotor dynamics
SHAFT_LENGTH = 1.2
SHAFT_OUTER_RADIUS = 0.015
SHAFT_INNER_RADIUS = 0.010
SHAFT_DENSITY = 7800.0
YOUNG_MODULUS = 2.1e11
POISSON_RATIO = 0.3
BEAM_SPANS = 12
BEAM_ORDER = 3

FLYWHEEL_RADIUS = 0.12
FLYWHEEL_WIDTH = 0.04
FLYWHEEL_DENSITY = 7800.0
ECCENTRIC_RADIUS = 0.026
ECCENTRIC_DENSITY = 12000.0
ECCENTRIC_OFFSET = chrono.ChVector3d(FLYWHEEL_WIDTH * 0.75, FLYWHEEL_RADIUS * 0.8, 0.0)
SPIN_SPEED = 45.0

TIME_STEP = 0.002
SIM_END = 2.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

LEFT_END = chrono.ChVector3d(-SHAFT_LENGTH / 2.0, 0.0, 0.0)
RIGHT_END = chrono.ChVector3d(SHAFT_LENGTH / 2.0, 0.0, 0.0)
CENTER_POS = chrono.ChVector3d(0.0, 0.0, 0.0)
SHAFT_AXIS_FRAME = chrono.ChFramed(LEFT_END, chrono.Q_ROTATE_Z_TO_X)


# === System & gravity === SMC FEA rotor uses a direct solver and Y-up gravity
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
hht = chrono.ChTimestepperHHT(sys)
hht.SetStepControl(False)
sys.SetTimestepper(hht)


# === FEA shaft === IGA Cosserat beam carries bending/torsion; no contact surface is needed
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

area = math.pi * (SHAFT_OUTER_RADIUS**2 - SHAFT_INNER_RADIUS**2)
iyy = (math.pi / 4.0) * (SHAFT_OUTER_RADIUS**4 - SHAFT_INNER_RADIUS**4)
izz = iyy
polar_j = 2.0 * iyy

shaft_inertia = fea.ChInertiaCosseratSimple()
shaft_inertia.SetDensity(SHAFT_DENSITY)
shaft_inertia.SetArea(area)
shaft_inertia.SetIyy(iyy)
shaft_inertia.SetIzz(izz)

shaft_elasticity = fea.ChElasticityCosseratSimple()
shaft_elasticity.SetYoungModulus(YOUNG_MODULUS)
shaft_elasticity.SetShearModulusFromPoisson(POISSON_RATIO)
shaft_elasticity.SetArea(area)
shaft_elasticity.SetIyy(iyy)
shaft_elasticity.SetIzz(izz)
shaft_elasticity.SetJ(polar_j)

shaft_section = fea.ChBeamSectionCosserat(shaft_inertia, shaft_elasticity)
shaft_section.SetCircular(True)
shaft_section.SetDrawCircularRadius(SHAFT_OUTER_RADIUS)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, shaft_section, BEAM_SPANS, LEFT_END, RIGHT_END, chrono.VECT_Y, BEAM_ORDER)
beam_node_refs = builder.GetLastBeamNodes()  # cache: keep SWIG container alive for node shared_ptrs
beam_nodes = [beam_node_refs[i] for i in range(beam_node_refs.size())]  # cache: reused by constraints/logging
left_node = beam_nodes[0]  # cache: driven end node
center_node = beam_nodes[len(beam_nodes) // 2]  # cache: flywheel attachment node
right_node = beam_nodes[-1]  # cache: bearing support node

vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
vis_surface.SetColorscaleMinMax(0.0, 0.006)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.02)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)
# FEA beam: no contact material needed; the shaft is driven by constraints, gravity, and a motor only.


# === Bodies & constraints === fixed truss, central flywheel, motorized end, and right bearing
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, FLYWHEEL_RADIUS, FLYWHEEL_WIDTH, FLYWHEEL_DENSITY)
flywheel.SetPos(CENTER_POS)
flywheel.EnableCollision(False)
sys.AddBody(flywheel)

eccentric_mass = chrono.ChBodyEasySphere(ECCENTRIC_RADIUS, ECCENTRIC_DENSITY)
eccentric_mass.SetPos(CENTER_POS + ECCENTRIC_OFFSET)
eccentric_mass.EnableCollision(False)
eccentric_mass.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.05, 0.02))
sys.AddBody(eccentric_mass)

flywheel_weld = chrono.ChLinkMateFix()
flywheel_weld.Initialize(center_node, flywheel)
sys.Add(flywheel_weld)

eccentric_weld = chrono.ChLinkMateFix()
eccentric_weld.Initialize(eccentric_mass, flywheel)
sys.Add(eccentric_weld)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(left_node, truss, SHAFT_AXIS_FRAME)
motor.SetSpeedFunction(chrono.ChFunctionConst(SPIN_SPEED))
sys.Add(motor)

right_bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
right_bearing.Initialize(right_node, truss, False, right_node.Frame(), right_node.Frame())
sys.Add(right_bearing)

sys.Setup()
sys.Update()
sys.DoStaticLinear()


# === Visualization === Irrlicht observes the FEA mesh and flywheel from an oblique view
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor - IGA Beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.6, 0.05, 0.35), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.AddTypicalLights()


# === Main loop === render at video cadence while stepping the rotor with the FEA timestep
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: reused for log row and stop condition
            center_pos = center_node.GetPos()  # cache: node position read once per step
            eccentric_pos = eccentric_mass.GetPos()  # cache: visible unbalance marker position read once per step
            flywheel_omega = flywheel.GetAngVelParent()  # cache: rigid body angular speed read once per step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # disk/output failures
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state
    traceback.print_exc()
    raise
finally:
    pass
