"""Beam buckling simulation (PyChrono 9.0.0, Irrlicht).

Models the lateral buckling of a slender flexible beam built from IGA Cosserat
finite elements. The beam runs horizontally between a fixed truss (left) and a
moving crank disk (right). A revolute-angle motor driven by a CUSTOM ChFunction
slowly rotates the crank; the crank is tied to the beam's right end through a
constraint, so the prescribed rotation forces an axial compression/twist on the
beam. As the imposed displacement grows the straight beam loses stability and
buckles out of plane.

System type : ChSystemSMC (required for stiff FEA beam stiffness matrices).
Main parts  : fixed truss body, IGA Cosserat beam (FEA mesh), crank disk body.
Constraints : truss<->beam-left (clamp, ChLinkMateGeneric), beam-right<->crank
              (clamp, ChLinkMateGeneric), crank<->truss (ChLinkMotorRotationAngle
              with a custom motion law).
Solver      : Pardiso MKL (direct) ; Timestepper : HHT.
Expected    : the beam compresses then buckles laterally; no fall-through, the
              truss end stays fixed while the crank end follows the motor law.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics (no bare literals downstream)
time_step = 1e-3
sim_end = 6.0
render_fps = 50.0

beam_L = 1.4                      # beam length along X (m)
beam_wy = 0.010                   # rectangular section width (m)
beam_wz = 0.010                   # rectangular section height (m)
beam_density = 1000.0            # kg/m^3
beam_E = 1.2e6                    # Young's modulus (Pa) — slender/flexible
beam_nu = 0.3                     # Poisson ratio
beam_spans = 12                   # number of IGA spans
beam_order = 3                    # cubic IGA

crank_radius = 0.10              # crank disk radius (m)
motor_rate = 0.5                  # base angular rate of the custom law (rad/s)

beam_A0 = chrono.ChVector3d(0, 0, 0)            # left (truss) end
beam_B0 = chrono.ChVector3d(beam_L, 0, 0)       # right (crank) end

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


# === Custom motor function === smooth ramp-and-hold angle law (custom ChFunction)
class BucklingAngle(chrono.ChFunction):
    """Custom motion law: smoothly ramps the crank to a held twist angle.

    angle(t) = A * (1 - cos(pi * t / T)) / 2 for t < T, then holds A.
    This eases the imposed displacement in so the beam buckles smoothly
    instead of being shock-loaded."""

    def __init__(self, amplitude, ramp_time):
        chrono.ChFunction.__init__(self)        # MUST call base ctor
        self.amplitude = amplitude
        self.ramp_time = ramp_time

    def GetVal(self, x):                          # x = time -> returns angle (rad)
        if x >= self.ramp_time:
            return self.amplitude
        return self.amplitude * (1.0 - math.cos(math.pi * x / self.ramp_time)) / 2.0


# === System & gravity === SMC system, Y-up world (FEA convention)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Direct solver + HHT timestepper are required for the stiff beam stiffness matrix.
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Bodies === fixed truss (left support) and crank disk (right driver)
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(beam_A0)
truss.AddVisualShape(chrono.ChVisualShapeBox(0.05, 0.18, 0.18),
                     chrono.ChFramed(chrono.ChVector3d(-0.03, 0, 0), chrono.QUNIT))
sys.Add(truss)

crank = chrono.ChBody()
crank.SetPos(beam_B0)
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
crank_disk = chrono.ChVisualShapeCylinder(crank_radius, 0.03)
crank.AddVisualShape(crank_disk,
                     chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.Add(crank)

# === FEA beam === IGA Cosserat slender beam from truss end to crank end
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)   # buckling is a forced static-like response

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, beam_density)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(beam_E)
melasticity.SetShearModulusFromPoisson(beam_nu)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

section = fea.ChBeamSectionCosserat(minertia, melasticity)
section.SetDrawThickness(beam_wy, beam_wz)

# FEA beam: no contact material needed — driven by constraints + motor only.
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, section, beam_spans,
                  beam_A0, beam_B0,
                  chrono.VECT_Y,        # suggested section Y direction
                  beam_order)

# Keep strong references to avoid SWIG GC of the node container (segfault guard).
beam_nodes = builder.GetLastBeamNodes()
node_left = beam_nodes.front()
node_right = beam_nodes.back()
node_mid = beam_nodes[beam_nodes.size() // 2]   # mid-span node — its lateral sweep is the buckle
refs = {"mesh": mesh, "builder": builder, "section": section, "nodes": beam_nodes}

sys.Add(mesh)

# === Joints / constraints === clamp the two beam ends to truss and crank
clamp_left = chrono.ChLinkMateGeneric()
clamp_left.Initialize(node_left, truss, False, node_left.Frame(), node_left.Frame())
clamp_left.SetConstrainedCoords(True, True, True, True, True, True)   # all 6 DOF
sys.Add(clamp_left)

clamp_right = chrono.ChLinkMateGeneric()
clamp_right.Initialize(node_right, crank, False, node_right.Frame(), node_right.Frame())
clamp_right.SetConstrainedCoords(True, True, True, True, True, True)  # all 6 DOF
sys.Add(clamp_right)

# Motor between crank and fixed truss, driven by the custom angle law.
motor = chrono.ChLinkMotorRotationAngle()
# Rotate the clamped right end about world Z (default motor axis): tilting the end
# forces the straight beam to bow laterally — i.e. it buckles in the XY plane.
motor.Initialize(crank, truss, chrono.ChFramed(beam_B0, chrono.QUNIT))
angle_law = BucklingAngle(1.0 * math.pi, sim_end * 0.7)   # keep a strong ref (SWIG GC guard)
motor.SetAngleFunction(angle_law)
refs["angle_law"] = angle_law
sys.Add(motor)

# === FEA visualization === scalar bending field + node-coordinate glyphs
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
vis.SetWindowTitle("Beam buckling (IGA Cosserat FEA)")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, 0.8, 1.6), chrono.ChVector3d(0.6, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0.6, -0.3, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics in inner batch
os.makedirs("cam", exist_ok=True)                                    # guard output dir

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
except (RuntimeError, ValueError) as exc:    # solver divergence / bad solver state
    import traceback
    traceback.print_exc()
    raise
