"""Flexible Euler beam chain in a Y-up SMC finite-element system.

The model builds a fixed-root beam segment, then adds a second beam segment with
builder.BuildBeam() using the first segment's last node as the A node and
(0.2, 0.1, -0.1) as the B point. The connected FEA beams deform under Y-down
gravity while Irrlicht displays the beam moment field and node coordinate glyphs.
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === beam geometry, solver, and render cadence are fixed once.
time_step = 0.001
sim_end = 2.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

beam_diameter = 0.012
beam_density = 1200.0
beam_young_modulus = 2.0e7
beam_poisson = 0.3
beam_rayleigh = 0.001
first_elements = 8
second_elements = 8
first_start = chrono.ChVector3d(0.0, 0.0, 0.0)
first_end = chrono.ChVector3d(0.12, 0.0, 0.0)
second_end = chrono.ChVector3d(0.2, 0.1, -0.1)
section_y_direction = chrono.ChVector3d(0.0, 1.0, 0.0)


# === System & Solver === FEA beams use SMC, Pardiso MKL, and HHT integration.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Beam Mesh === build two connected Euler beam segments with shared endpoint node.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetDensity(beam_density)
section.SetYoungModulus(beam_young_modulus)
section.SetShearModulusFromPoisson(beam_poisson)
section.SetRayleighDamping(beam_rayleigh)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, section, first_elements, first_start, first_end, section_y_direction)
first_beam_nodes_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
first_beam_nodes = [first_beam_nodes_container[i] for i in range(first_beam_nodes_container.size())]
root_node = first_beam_nodes[0]  # cache: reused for fixed support and logging
previous_segment_last_node = first_beam_nodes[-1]  # cache: A node passed to the added beam segment
root_node.SetFixed(True)

builder.BuildBeam(mesh, section, second_elements, previous_segment_last_node, second_end, section_y_direction)
second_beam_nodes_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
second_beam_nodes = [second_beam_nodes_container[i] for i in range(second_beam_nodes_container.size())]
shared_node = second_beam_nodes[0]  # cache: connected A node after the builder refreshes wrappers
tip_node = second_beam_nodes[-1]  # cache: reused for loop logging

# FEA beam: no contact material needed because this chain is constrained and gravity-driven only.
sys.Add(mesh)


# === FEA Visualization Shapes === show bending-moment colors and node coordinate glyphs.
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.03, 0.03)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.002)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization === Irrlicht initializes before sky, camera, lights, and grid.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Connected Euler Beam Segments")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.35, 0.18, 0.32), chrono.ChVector3d(0.1, 0.02, -0.02))
vis.AddTypicalLights()
vis.AddGrid(
    0.05,
    0.05,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.1, -0.04, 0.0), chrono.Q_ROTATE_Z_TO_Y),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop === render at frame cadence and advance FEA dynamics in small steps.
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()  # cache: reused by logging and time bound
            tip_pos = tip_node.GetPos()  # cache: avoids repeated node getter calls
            shared_pos = shared_node.GetPos()  # cache: avoids repeated node getter calls
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid FEA state
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # disk or output permission failure
    print(f"Output failure: {exc}")
    raise
finally:
    pass
