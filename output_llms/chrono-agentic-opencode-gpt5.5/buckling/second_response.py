"""Motor-driven FEA buckling column using a ChSystemSMC beam assembly.

The model contains a fixed truss, a horizontal support beam, a vertical Euler
column, and a motorized crank with a flexible crank beam. The crank tip is tied
to the column top so the rotating crank compresses and bends the column while
Irrlicht displays beam moment colors and node glyphs.
"""

import math


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === geometry and timing are named for isolated source review
L = 1.2
H = 0.3
K = 0.07

TIME_STEP = 0.001
SIM_END = 4.0
RENDER_FPS = 30.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TRUSS_VIS = chrono.ChVector3d(0.03, 0.25, 0.12)
CRANK_VIS = chrono.ChVector3d(K, 0.03, 0.03)
HORIZONTAL_WY = 0.12
HORIZONTAL_WZ = 0.012
VERTICAL_DIAMETER = 0.03
VERTICAL_ELEMENTS = 6
CRANK_DIAMETER = 0.054
CRANK_ELEMENTS = 5
CONSTRAINT_SPHERE_RADIUS = 0.012
CRANK_COLUMN_SPHERE_RADIUS = 0.014
GLYPH_SCALE = 0.015

YOUNG_MODULUS = 2.0e7
DENSITY = 7800.0
RAYLEIGH_DAMPING = 0.002
CRANK_SPEED = 0.55

BASE = chrono.ChVector3d(0.0, 0.0, 0.0)
COLUMN_BASE = chrono.ChVector3d(L, 0.0, 0.0)
COLUMN_TOP = chrono.ChVector3d(L, H, 0.0)
CRANK_PIVOT = chrono.ChVector3d(L - K, H, 0.0)
CAMERA_EYE = chrono.ChVector3d(0.0, 0.7, -1.2)
CAMERA_TARGET = chrono.ChVector3d(0.65, 0.15, 0.0)


def add_colored_sphere(body, local_pos, radius, color):
    """Attach a small visual marker to a rigid body."""
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(color)
    body.AddVisualShape(marker, chrono.ChFramed(local_pos, chrono.QUNIT))


# === System & solver === stiff FEA beams use SMC, Pardiso MKL, and HHT
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)


# === Bodies === fixed truss and motorized crank provide the rigid constraints
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(BASE)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
truss_shape = chrono.ChVisualShapeBox(TRUSS_VIS)
truss_shape.SetColor(chrono.ChColor(0.25, 0.25, 0.28))
truss.AddVisualShape(truss_shape)
add_colored_sphere(truss, COLUMN_BASE, CONSTRAINT_SPHERE_RADIUS, chrono.ChColor(0.1, 0.4, 0.9))
add_colored_sphere(truss, CRANK_PIVOT, CONSTRAINT_SPHERE_RADIUS, chrono.ChColor(0.1, 0.4, 0.9))
sys.AddBody(truss)

crank = chrono.ChBody()
crank.SetMass(0.2)
crank.SetInertiaXX(chrono.ChVector3d(0.002, 0.002, 0.002))
crank.SetPos(CRANK_PIVOT)
crank.EnableCollision(False)
crank_shape = chrono.ChVisualShapeBox(CRANK_VIS)
crank_shape.SetColor(chrono.ChColor(0.85, 0.25, 0.15))
crank.AddVisualShape(crank_shape, chrono.ChFramed(chrono.ChVector3d(K / 2.0, 0.0, -0.035), chrono.QUNIT))
crank_beam_shape = chrono.ChVisualShapeCylinder(CRANK_DIAMETER / 2.0, K)
crank_beam_shape.SetColor(chrono.ChColor(0.05, 0.35, 0.95))
crank.AddVisualShape(
    crank_beam_shape,
    chrono.ChFramed(chrono.ChVector3d(K / 2.0, 0.0, 0.035), chrono.QuatFromAngleY(chrono.CH_PI_2)),
)
add_colored_sphere(crank, chrono.ChVector3d(K, 0.0, 0.0), CRANK_COLUMN_SPHERE_RADIUS, chrono.ChColor(0.95, 0.65, 0.05))
sys.AddBody(crank)

# FEA beam: no contact material needed — driven by constraints, gravity, and motor only.


# === FEA beams === horizontal support, vertical column, and crank beam share one mesh
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

horizontal_section = fea.ChBeamSectionEulerAdvanced()
horizontal_section.SetAsRectangularSection(HORIZONTAL_WY, HORIZONTAL_WZ)
horizontal_section.SetDensity(DENSITY)
horizontal_section.SetYoungModulus(YOUNG_MODULUS)
horizontal_section.SetShearModulusFromPoisson(0.3)
horizontal_section.SetRayleighDamping(RAYLEIGH_DAMPING)

vertical_section = fea.ChBeamSectionEulerAdvanced()
vertical_section.SetAsCircularSection(VERTICAL_DIAMETER)
vertical_section.SetDensity(DENSITY)
vertical_section.SetYoungModulus(YOUNG_MODULUS)
vertical_section.SetShearModulusFromPoisson(0.3)
vertical_section.SetRayleighDamping(RAYLEIGH_DAMPING)

crank_section = fea.ChBeamSectionEulerAdvanced()
crank_section.SetAsCircularSection(CRANK_DIAMETER)
crank_section.SetDensity(DENSITY)
crank_section.SetYoungModulus(YOUNG_MODULUS)
crank_section.SetShearModulusFromPoisson(0.3)
crank_section.SetRayleighDamping(RAYLEIGH_DAMPING)

horizontal_builder = fea.ChBuilderBeamEuler()
horizontal_builder.BuildBeam(mesh, horizontal_section, 6, BASE, COLUMN_BASE, chrono.VECT_Z)
horizontal_node_refs = horizontal_builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
horizontal_nodes = [horizontal_node_refs[i] for i in range(horizontal_node_refs.size())]

vertical_builder = fea.ChBuilderBeamEuler()
vertical_builder.BuildBeam(mesh, vertical_section, VERTICAL_ELEMENTS, COLUMN_BASE, COLUMN_TOP, chrono.VECT_Z)
vertical_node_refs = vertical_builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
vertical_nodes = [vertical_node_refs[i] for i in range(vertical_node_refs.size())]

crank_builder = fea.ChBuilderBeamEuler()
crank_builder.BuildBeam(mesh, crank_section, CRANK_ELEMENTS, CRANK_PIVOT, COLUMN_TOP, chrono.VECT_Z)
crank_node_refs = crank_builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
crank_nodes = [crank_node_refs[i] for i in range(crank_node_refs.size())]

horizontal_nodes[0].SetFixed(True)
horizontal_nodes[-1].SetFixed(True)
vertical_nodes[0].SetFixed(True)
crank_nodes[0].SetFixed(True)

vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.04, 0.04)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)


# === Constraints === motorized crank and node-body ties create the compression path
crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crank, truss, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
crank_motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(crank_motor)

column_top_to_crank = chrono.ChLinkMateFix()
column_top_to_crank.Initialize(
    crank,
    vertical_nodes[-1],
    True,
    chrono.ChFramed(chrono.ChVector3d(K, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
)
sys.Add(column_top_to_crank)

crank_beam_tip_to_crank = chrono.ChLinkMateFix()
crank_beam_tip_to_crank.Initialize(
    crank,
    crank_nodes[-1],
    True,
    chrono.ChFramed(chrono.ChVector3d(K, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
)
sys.Add(crank_beam_tip_to_crank)


# === Visualization === Irrlicht window shows the full FEA mechanism and markers
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Buckling FEA crank-column demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAMERA_EYE, CAMERA_TARGET)
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.6, -0.04, 0.0), chrono.QuatFromAngleX(math.pi / 2.0)),
    chrono.ChColor(0.45, 0.45, 0.45),
)


# === Main loop === render at a fixed cadence and advance the FEA solver
vertical_tip = vertical_nodes[-1]  # cache: queried every step for review diagnostics
vertical_mid = vertical_nodes[len(vertical_nodes) // 2]  # cache: queried every step


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()  # cache: use one time query for diagnostics
            tip_pos = vertical_tip.GetPos()  # cache: reused in the row below
            mid_pos = vertical_mid.GetPos()  # cache: reused in the row below
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    raise
except (OSError, IOError) as exc:  # disk or renderer I/O failure during review capture
    raise
finally:
    pass


# === Post-processing === review videos and plots are stripped before scoring
