"""Flexible SMC rotor with an IGA hollow beam, constrained bearings, and a flywheel.

The model uses PyChrono FEA Cosserat beam elements for a longer hollow shaft,
a rigid cylindrical flywheel welded to the beam midpoint, low downward gravity,
and a sinusoidal spin torque. The expected behavior is a flexible rotor that
spins and deflects smoothly while the Irrlicht view shows the longer beam from
the requested camera position.
"""

import contextlib
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === named simulation parameters and derived section properties
beam_L = 10.0
beam_ro = 0.060
beam_ri = 0.055
beam_density = 7800.0
beam_E = 210.0e9
beam_nu = 0.3
beam_spans = 20
beam_order = 3

flywheel_radius = 0.30
flywheel_width = 0.10
flywheel_density = 7800.0

time_step = 0.002
sim_end = 4.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

section_area = math.pi * (beam_ro * beam_ro - beam_ri * beam_ri)  # precomputed once
section_I = math.pi * (beam_ro**4 - beam_ri**4) / 4.0  # precomputed once
section_J = 2.0 * section_I  # precomputed once


# === System & gravity === FEA rotor uses SMC, Pardiso, and HHT integration
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA beam and flywheel === hollow IGA shaft plus welded rigid disk
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(beam_density)
minertia.SetArea(section_area)
minertia.SetIyy(section_I)
minertia.SetIzz(section_I)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(beam_E)
melasticity.SetShearModulusFromPoisson(beam_nu)
melasticity.SetIyy(section_I)
melasticity.SetIzz(section_I)
melasticity.SetJ(section_J)

beam_section = fea.ChBeamSectionCosserat(minertia, melasticity)
beam_section.SetCircular(True)
beam_section.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    beam_section,
    beam_spans,
    chrono.ChVector3d(-0.5 * beam_L, 0, 0),
    chrono.ChVector3d(0.5 * beam_L, 0, 0),
    chrono.VECT_Y,
    beam_order,
)
beam_nodes_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]  # cache: reused for constraints and logging
left_node = beam_nodes[0]  # cache: bearing node
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: flywheel weld node
right_node = beam_nodes[-1]  # cache: bearing node

sys.Add(mesh)

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)
mbodyflywheel.SetPos(mid_node.GetPos())
mbodyflywheel.EnableCollision(False)
sys.Add(mbodyflywheel)

flywheel_weld = chrono.ChLinkMateFix()
flywheel_weld.Initialize(mid_node, mbodyflywheel)
sys.Add(flywheel_weld)

# FEA beam: no contact material needed -- driven by constraints + gravity + motor only.


# === Joints / constraints === fixed truss with end bearings that leave shaft spin free
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

left_bearing = chrono.ChLinkMateGeneric()
left_bearing.Initialize(left_node, truss, False, left_node.Frame(), left_node.Frame())
left_bearing.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(left_bearing)

right_bearing = chrono.ChLinkMateGeneric()
right_bearing.Initialize(right_node, truss, False, right_node.Frame(), right_node.Frame())
right_bearing.SetConstrainedCoords(True, True, True, False, True, True)
sys.Add(right_bearing)

f_ramp = chrono.ChFunctionSine(60, 0.1)


# === Visualization === Irrlicht window with FEA surface, node glyphs, camera, lights, and grid
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Flexible rotor with longer hollow beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    24,
    24,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, 0), chrono.Q_ROTATE_Y_TO_Z),
    chrono.ChColor(0.4, 0.4, 0.4),
)

sys.DoStaticLinear()


# === Main loop === render-cadence recording with per-step rotor loading
def run_simulation(data_writer):
    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            current_time = sys.GetChTime()  # cache: used for load and logging this step
            drive_torque = f_ramp.GetVal(current_time)
            mid_node.SetTorque(chrono.ChVector3d(drive_torque, 0, 0))
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break


# === Run & cleanup === named handlers preserve partial outputs on file or solver failure
data_writer = None
try:
    with contextlib.ExitStack() as stack:
        run_simulation(data_writer)
except (OSError, IOError) as exc:  # disk or permissions while preparing review outputs
    raise RuntimeError(f"Output file setup failed: {exc}") from exc
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid PyChrono state
    raise
finally:
    data_writer = None
