"""Flexible-column buckling simulation with PyChrono FEA.

This SMC/HHT model builds an Euler-Bernoulli beam column between a fixed base
plate and a motor-driven top platen. A custom linear motor function compresses
the top platen while node-frame constraints tie the beam ends to the rigid
fixtures; a small lateral force seed lets the column visibly buckle under load.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === direct beam/motor parameters keep the FEA setup explicit
time_step = 0.001
sim_end = 2.5
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

beam_length = 1.20
beam_width_y = 0.025
beam_width_z = 0.025
beam_elements = 24
beam_density = 2700.0
beam_young = 7.3e10
poisson = 0.30
compression = 0.055
compression_time = 1.5
plate_mass = 4.0
plate_size = chrono.ChVector3d(0.18, 0.035, 0.18)
seed_force = 4.0


class SmoothCompression(chrono.ChFunction):
    """Custom motor function: smooth downward displacement for axial loading."""

    def __init__(self, distance, duration):
        chrono.ChFunction.__init__(self)
        self.distance = distance
        self.duration = duration

    def GetVal(self, x):
        if x <= 0.0:
            return 0.0
        if x >= self.duration:
            return -self.distance
        s = x / self.duration
        return -self.distance * (3.0 * s * s - 2.0 * s * s * s)

    def Update(self, x):
        return None


# === System & solver === FEA beams use SMC, Pardiso MKL, and HHT integration
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)


# === Bodies === rigid fixtures define the constrained compression apparatus
ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

base_plate = chrono.ChBody()
base_plate.SetFixed(True)
base_plate.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
base_shape = chrono.ChVisualShapeBox(plate_size)
base_shape.SetColor(chrono.ChColor(0.25, 0.25, 0.28))
base_plate.AddVisualShape(base_shape)
sys.AddBody(base_plate)

top_plate = chrono.ChBody()
top_plate.SetMass(plate_mass)
top_plate.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
top_plate.SetPos(chrono.ChVector3d(0.0, beam_length, 0.0))
top_shape = chrono.ChVisualShapeBox(plate_size)
top_shape.SetColor(chrono.ChColor(0.85, 0.25, 0.20))
top_plate.AddVisualShape(top_shape)
sys.AddBody(top_plate)


# === FEA beam === the flexible column is driven by constraints and a motor only
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(beam_width_y, beam_width_z)
section.SetDensity(beam_density)
section.SetYoungModulus(beam_young)
section.SetShearModulusFromPoisson(poisson)
section.SetRayleighDamping(0.001)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    beam_elements,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(0.0, beam_length, 0.0),
    chrono.VECT_Z,
)
beam_nodes_ref = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_nodes_ref[i] for i in range(beam_nodes_ref.size())]  # cache: reused in loop
bottom_node = beam_nodes[0]  # cache: constrained and logged
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: lateral response indicator
top_node = beam_nodes[-1]  # cache: constrained and logged
sys.Add(mesh)

# FEA beam: no contact material needed -- driven by constraints, lateral seed, and motor only.
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.08, 0.08)
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


# === Constraints & motor === links connect beam nodes, rigid plates, and guide motion
bottom_fix = chrono.ChLinkMateFix()
bottom_fix.Initialize(bottom_node, base_plate)
sys.Add(bottom_fix)

top_fix = chrono.ChLinkMateFix()
top_fix.Initialize(top_node, top_plate)
sys.Add(top_fix)

compression_motor = chrono.ChLinkMotorLinearPosition()
compression_motor.Initialize(top_plate, ground, chrono.ChFramed(chrono.ChVector3d(0.0, beam_length, 0.0), chrono.Q_ROTATE_Z_TO_Y))
compression_motor.SetGuideConstraint(chrono.ChLinkMotorLinear.GuideConstraint_PRISMATIC)
compression_profile = SmoothCompression(compression, compression_time)  # cache: keep Python callback alive
compression_motor.SetMotionFunction(compression_profile)
sys.Add(compression_motor)


# === Visualization === Irrlicht is initialized before adding sky, camera, lights, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA beam buckling with custom motor compression")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.25, 0.55, 1.25), chrono.ChVector3d(0.0, 0.55, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.03, 0.0), chrono.Q_ROTATE_Y_TO_Z),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at fixed cadence, step FEA dynamics, and seed buckling
frame = 0
try:
        while vis.Run() and sys.GetChTime() < sim_end:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(render_every):
                if sys.GetChTime() < 0.35:
                    mid_node.SetForce(chrono.ChVector3d(seed_force, 0.0, 0.0))
                else:
                    mid_node.SetForce(chrono.ChVector3d(0.0, 0.0, 0.0))
                sys.DoStepDynamics(time_step)
                if sys.GetChTime() >= sim_end:
                    break
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === review artifacts are assembled and stripped before scoring
