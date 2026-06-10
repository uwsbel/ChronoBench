"""Euler beam finite-element demonstration using an SMC Chrono system.

The model builds a cantilever beam from Euler-Bernoulli finite elements and
xyzrot nodes, clamps the root to a fixed truss, applies a downward tip load,
and visualizes bending moment plus node coordinate glyphs in Irrlicht.
"""

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === beam geometry and integration settings are fixed once for reuse
time_step = 0.001
sim_end = 4.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

beam_length = 1.4
beam_diameter = 0.03
beam_elements = 12
beam_density = 7800.0
beam_young = 2.0e7
beam_poisson = 0.30
rayleigh_damping = 0.000
tip_force_y = -80.0


# === System & solver === stiff FEA beams use SMC, Pardiso MKL, and HHT stepping
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA mesh === an Euler beam mesh demonstrates beam elements and node properties
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetDensity(beam_density)
section.SetYoungModulus(beam_young)
section.SetShearModulusFromPoisson(beam_poisson)
section.SetRayleighDamping(rayleigh_damping)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    beam_elements,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_length, 0, 0),
    chrono.VECT_Y,
)
beam_node_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]
root_node = beam_nodes[0]  # cache: root is reused for the clamp
tip_node = beam_nodes[-1]  # cache: tip is reused for force and logging
tip_node.SetForce(chrono.ChVector3d(0, tip_force_y, 0))
sys.Add(mesh)

# FEA beam: no contact material needed — driven by constraints and nodal load only.


# === Constraints === a fixed truss and generic mate fully clamp the beam root
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

root_clamp = chrono.ChLinkMateGeneric()
root_clamp.Initialize(root_node, truss, False, root_node.Frame(), root_node.Frame())
root_clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(root_clamp)


# === FEA visualization === two standard FEA shapes show bending moment and nodes
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-2.0, 2.0)
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


# === Visualization === Irrlicht renders the beam, sky, camera, lights, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono beam finite elements")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.9, 0.35, 1.7), chrono.ChVector3d(0.7, -0.25, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.6, -0.25, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at frame cadence and advance beam dynamics in batches
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
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # output directory or file I/O failure
    print(f"Recording output failed: {exc}")
    raise
finally:
    pass
