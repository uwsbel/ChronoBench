"""FEA buckling mechanism with a fixed truss, rotating crank, and flexible beams.

The model uses a PyChrono SMC system with Euler-Bernoulli beam elements, a
Pardiso MKL direct solver, and HHT time integration. A fixed truss supports a
motor-driven crank; flexible crank, vertical, and horizontal beam spans are
connected by node constraints so the rotating crank compresses and bends the
slender vertical beam.
"""

import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Parameters: final isolated prompt values ===
L = 1.2
H = 0.3
K = 0.07

TIME_STEP = 0.001
SIM_END = 2.5
CRANK_SPEED = 0.6

TRUSS_SIZE = chrono.ChVector3d(0.03, 0.25, 0.12)
CRANK_VIS_SIZE = chrono.ChVector3d(K, 0.03, 0.03)
HORIZONTAL_WIDTH_Y = 0.12
HORIZONTAL_WIDTH_Z = 0.012
VERTICAL_DIAMETER = 0.03
CRANK_DIAMETER = 0.054
VERTICAL_ELEMENTS = 6
CRANK_ELEMENTS = 5
HORIZONTAL_ELEMENTS = 8
CONSTRAINT_SPHERE_SIZE = 0.012
CRANK_VERTICAL_SPHERE_SIZE = 0.014
GLYPH_SCALE = 0.015

TRUSS_POS = chrono.ChVector3d(-0.08, H * 0.5, 0.0)
CRANK_CENTER = chrono.ChVector3d(0.0, 0.0, 0.0)
CRANK_TIP_INITIAL = chrono.ChVector3d(K, 0.0, 0.0)
VERTICAL_BOTTOM = CRANK_TIP_INITIAL
VERTICAL_TOP = chrono.ChVector3d(K, H, 0.0)
HORIZONTAL_RIGHT = chrono.ChVector3d(L, H, 0.0)


# === Helpers: body and constraint visualization ===
def add_box_visual(body, size, color):
    """Attach one full-extent box visual to a body."""
    shape = chrono.ChVisualShapeBox(size)
    shape.SetColor(color)
    body.AddVisualShape(shape)


def add_sphere_visual(body, radius, color, local_pos=chrono.VNULL):
    """Attach a small sphere marker used to show constrained nodes."""
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(color)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos, chrono.QUNIT))


def collect_nodes(builder):
    """Keep a strong-reference list from the builder node container."""
    nodes_container = builder.GetLastBeamNodes()  # cache: SWIG container retained before indexing
    return nodes_container, [nodes_container[i] for i in range(nodes_container.size())]


# === System & solver: stiff FEA beam mechanism ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Rigid bodies: fixed truss and actuated crank ===
truss = chrono.ChBody()
truss.SetName("fixed_truss")
truss.SetFixed(True)
truss.SetPos(TRUSS_POS)
truss.SetMass(10.0)
truss.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
add_box_visual(truss, TRUSS_SIZE, chrono.ChColor(0.35, 0.36, 0.40))
sys.AddBody(truss)

crank = chrono.ChBody()
crank.SetName("rigid_crank")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.004, 0.004, 0.004))
crank.SetPos(chrono.ChVector3d(K * 0.5, 0.0, 0.0))
add_box_visual(crank, CRANK_VIS_SIZE, chrono.ChColor(0.18, 0.34, 0.82))
add_sphere_visual(crank, CRANK_VERTICAL_SPHERE_SIZE, chrono.ChColor(0.9, 0.25, 0.15), chrono.ChVector3d(K * 0.5, 0.0, 0.0))
sys.AddBody(crank)

crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crank, truss, chrono.ChFramed(CRANK_CENTER, chrono.QUNIT))
crank_motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(crank_motor)


# === FEA mesh: horizontal, vertical, and crank beam spans ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

horizontal_section = fea.ChBeamSectionEulerAdvanced()
horizontal_section.SetAsRectangularSection(HORIZONTAL_WIDTH_Y, HORIZONTAL_WIDTH_Z)
horizontal_section.SetDensity(7850)
horizontal_section.SetYoungModulus(2.1e11)
horizontal_section.SetShearModulusFromPoisson(0.3)
horizontal_section.SetRayleighDamping(0.000)

vertical_section = fea.ChBeamSectionEulerAdvanced()
vertical_section.SetAsCircularSection(VERTICAL_DIAMETER)
vertical_section.SetDensity(7850)
vertical_section.SetYoungModulus(2.1e11)
vertical_section.SetShearModulusFromPoisson(0.3)
vertical_section.SetRayleighDamping(0.000)

crank_section = fea.ChBeamSectionEulerAdvanced()
crank_section.SetAsCircularSection(CRANK_DIAMETER)
crank_section.SetDensity(7850)
crank_section.SetYoungModulus(2.1e11)
crank_section.SetShearModulusFromPoisson(0.3)
crank_section.SetRayleighDamping(0.000)

horizontal_builder = fea.ChBuilderBeamEuler()
horizontal_builder.BuildBeam(
    mesh,
    horizontal_section,
    HORIZONTAL_ELEMENTS,
    VERTICAL_TOP,
    HORIZONTAL_RIGHT,
    chrono.VECT_Z,
)
horizontal_nodes_container, horizontal_nodes = collect_nodes(horizontal_builder)

vertical_builder = fea.ChBuilderBeamEuler()
vertical_builder.BuildBeam(
    mesh,
    vertical_section,
    VERTICAL_ELEMENTS,
    VERTICAL_BOTTOM,
    VERTICAL_TOP,
    chrono.VECT_Z,
)
vertical_nodes_container, vertical_nodes = collect_nodes(vertical_builder)

crank_builder = fea.ChBuilderBeamEuler()
crank_builder.BuildBeam(
    mesh,
    crank_section,
    CRANK_ELEMENTS,
    CRANK_CENTER,
    CRANK_TIP_INITIAL,
    chrono.VECT_Z,
)
crank_nodes_container, crank_nodes = collect_nodes(crank_builder)

# FEA beam: no contact material needed because constraints and the motor drive the buckling load.
sys.Add(mesh)


# === Constraints: node-to-frame couplings among the mechanism parts ===
root_fix = chrono.ChLinkMateGeneric()
root_fix.Initialize(crank_nodes[0], truss, False, crank_nodes[0].Frame(), crank_nodes[0].Frame())
root_fix.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(root_fix)

tip_to_crank = chrono.ChLinkMateGeneric()
tip_to_crank.Initialize(crank_nodes[-1], crank, False, crank_nodes[-1].Frame(), crank_nodes[-1].Frame())
tip_to_crank.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(tip_to_crank)

bottom_pin = chrono.ChLinkMateGeneric()
bottom_pin.Initialize(vertical_nodes[0], crank_nodes[-1], False, vertical_nodes[0].Frame(), crank_nodes[-1].Frame())
bottom_pin.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(bottom_pin)

top_pin = chrono.ChLinkMateGeneric()
top_pin.Initialize(vertical_nodes[-1], horizontal_nodes[0], False, vertical_nodes[-1].Frame(), horizontal_nodes[0].Frame())
top_pin.SetConstrainedCoords(True, True, True, False, False, False)
sys.Add(top_pin)

right_support = chrono.ChLinkMateGeneric()
right_support.Initialize(horizontal_nodes[-1], truss, False, horizontal_nodes[-1].Frame(), horizontal_nodes[-1].Frame())
right_support.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(right_support)

add_sphere_visual(truss, CONSTRAINT_SPHERE_SIZE, chrono.ChColor(0.0, 0.55, 0.85), HORIZONTAL_RIGHT - TRUSS_POS)


# === FEA visualization: moment field plus node glyphs ===
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
vis_glyph.SetSymbolsScale(GLYPH_SCALE)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization: Irrlicht window, camera, lights, and grid ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Buckling Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(0.45, 0.18, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 14, 8, chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0.0, 0.0), chrono.QUNIT), chrono.ChColor(0.35, 0.35, 0.35))
vis.SetSymbolScale(GLYPH_SCALE)


# === Main loop: render and advance the flexible mechanism ===

try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        # cache: node positions fetched once per loop for output and validation.
        time_now = sys.GetChTime()
        vertical_mid = vertical_nodes[len(vertical_nodes) // 2].GetPos()
        vertical_top = vertical_nodes[-1].GetPos()


        sys.DoStepDynamics(TIME_STEP)

except (RuntimeError, ValueError, OSError) as exc:
    print(f"Simulation failed: {exc}")
    raise
finally:
    pass
