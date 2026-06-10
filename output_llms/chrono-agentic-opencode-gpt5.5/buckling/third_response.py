"""FEA buckling column simulation using PyChrono SMC.

This standalone script models a slender Euler-Bernoulli beam column clamped at
its lower end and compressed at its upper end. The small lateral component in
the load breaks symmetry so the column visibly bends while the HHT time stepper
and Pardiso MKL solver handle the stiff finite-element system.
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === direct beam/load values keep the benchmark setup explicit
beam_length = 3.0
beam_diameter = 0.035
beam_density = 1200.0
young_modulus = 2.5e9
poisson_ratio = 0.30
rayleigh_damping = 0.002
num_elements = 16
time_step = 1.0e-3
sim_end = 2.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
compressive_load = -10.0
lateral_load = 10.0


# === System & FEA mesh === SMC + direct solver for a stiff beam column
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetDensity(beam_density)
section.SetYoungModulus(young_modulus)
section.SetShearModulusFromPoisson(poisson_ratio)
section.SetRayleighDamping(rayleigh_damping)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    num_elements,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(0.0, beam_length, 0.0),
    chrono.VECT_Z,
)
beam_nodes_ref = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_nodes_ref[i] for i in range(beam_nodes_ref.size())]  # cache: reused in loop
root_node = beam_nodes[0]  # cache: clamped support node
tip_node = beam_nodes[-1]  # cache: loaded end node
root_node.SetFixed(True)

# FEA beam: no contact material needed — driven by constraints and nodal load only.
tip_node.SetForce(chrono.ChVector3d(lateral_load, compressive_load, 0.0))
sys.Add(mesh)


# === FEA visualization === scalar bending color plus node glyphs on the beam mesh
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-80.0, 80.0)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.04)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization === Irrlicht window shows the Y-up buckling column and support grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA buckling column")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, 1.6, 4.0), chrono.ChVector3d(0.0, 1.4, 0.0))
vis.AddTypicalLights()
grid_frame = chrono.ChCoordsysd(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleX(chrono.CH_PI_2),
)
vis.AddGrid(0.25, 0.25, 16, 16, grid_frame, chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop === render at video cadence and advance FEA dynamics in small steps
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            tip_node.SetForce(chrono.ChVector3d(lateral_load, compressive_load, 0.0))
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid finite-element state
    raise
finally:
    pass
