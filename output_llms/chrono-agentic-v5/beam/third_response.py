"""IGA (Cosserat) beam FEA simulation with two chained beam segments.

Models a deformable beam built with the isogeometric (IGA) Cosserat beam family
on a single ChMesh. System type: ChSystemSMC (required for stiff FEA beams), with
the Pardiso MKL direct solver and an HHT timestepper. A first straight beam runs
along +X from the origin; its root node is clamped. A SECOND beam segment is then
built so that it begins at the LAST node of the first beam (its 'A' node) and ends
at the point (0.2, 0.1, -0.1) ('B' point), with the up reference direction (0,1,0).
Both segments share one mesh, so the chained structure deflects under gravity as a
single continuous deformable assembly. Expected behavior: the clamped end stays put
while the free chained tip sags and settles under gravity.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Parameters === geometry / material / time-stepping named constants
time_step = 1e-3
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))            # precomputed once

beam_L = 0.4                  # length of the first straight beam along +X (m)
beam_wy = 0.012               # rectangular section width in local Y (m)
beam_wz = 0.025               # rectangular section width in local Z (m)
n_spans = 10                  # IGA spans per beam segment
order = 3                     # IGA order (3 = cubic)

# Endpoint of the second (chained) beam segment and its up reference direction.
seg2_B = chrono.ChVector3d(0.2, 0.1, -0.1)
seg2_up = chrono.ChVector3d(0, 1, 0)

# === System & gravity === ChSystemSMC + Y-up gravity (FEA-beam world convention)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True, 2)   # >=2 integration pts/element for cubic IGA gravity
sys.Add(mesh)

# === Beam section === Cosserat inertia + elasticity for a rectangular cross-section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 1000)   # sets area/inertia from w,h,density

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(0.02e10)
melasticity.SetShearModulus(0.02e10 * 0.38)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)      # sets A, Ixx, Iyy, Ksy, Ksz, J

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetDrawThickness(beam_wy, beam_wz)

# === Beams === first straight beam, then a second segment chained onto its last node
# FEA beam: no contact material needed — driven by constraints + gravity only.

# First beam segment: origin -> (beam_L, 0, 0) along +X.
builder1 = fea.ChBuilderBeamIGA()
builder1.BuildBeam(mesh, msection, n_spans,
                   chrono.ChVector3d(0, 0, 0),         # A: start point
                   chrono.ChVector3d(beam_L, 0, 0),    # B: end point
                   chrono.VECT_Y,                      # suggested section Y direction
                   order)

# Materialize the first beam's node shared_ptrs immediately into a stable list
# (the builder's GetLastBeamNodes container is reused/overwritten by later builds).
b1_container = builder1.GetLastBeamNodes()            # cache: SWIG container kept alive
seg1_nodes = [b1_container[i] for i in range(b1_container.size())]
seg1_nodes[0].SetFixed(True)                          # clamp the root -> cantilever

# The last node created by the first beam is the 'A' node of the new segment.
seg2_A_node = seg1_nodes[-1]                          # cache: chaining node reused below
seg2_A = seg2_A_node.GetPos()

# Second beam segment: a fresh builder so the first beam's nodes stay valid. Its 'A'
# point is the previous beam's last node; its 'B' point is (0.2, 0.1, -0.1).
builder2 = fea.ChBuilderBeamIGA()
builder2.BuildBeam(mesh, msection, n_spans,
                   seg2_A,                             # A: last node of the previous beam
                   seg2_B,                             # B: point (0.2, 0.1, -0.1)
                   seg2_up,                            # 'Y' up direction (0, 1, 0)
                   order)
b2_container = builder2.GetLastBeamNodes()            # cache: SWIG container kept alive
seg2_nodes = [b2_container[i] for i in range(b2_container.size())]

# Weld the second segment's first node to the previous beam's last node so the two
# segments form one continuous chained structure sharing that junction point.
junction = chrono.ChLinkMateFix()
junction.Initialize(seg2_A_node, seg2_nodes[0])
sys.Add(junction)

# === Solver & timestepper === Pardiso MKL direct solver + HHT for stiff beams
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA visualization === surface (deformed) shape + node coordinate-system glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
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
vis.SetWindowTitle("FEA IGA beam: chained two-segment structure")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2, 0.4, 0.8), chrono.ChVector3d(0.25, 0.0, -0.05))
vis.AddTypicalLights()
vis.AddGrid(0.05, 0.05, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid

# === Main loop === render-cadence outer loop; batch physics steps between frames
tip_node = seg2_nodes[-1]        # cache: free chained tip, logged each frame
os.makedirs("cam", exist_ok=True)                   # guard against missing output dir

try:
    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
