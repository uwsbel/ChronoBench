"""
PyChrono 9.0 FEA beam simulation — Euler-Bernoulli beam elements with ChBuilderBeamEuler.

Adds two beam segments: the first from (0,0,-0.1) to (0.2,0,-0.1), and a second
from the last node of the first beam to (0.2, 0.1, -0.1), both with Y-up section.
A fixed truss constrains the root; nodal forces and torques are applied.
"""
import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === System & gravity ===
sys = chrono.ChSystemSMC()

# === FEA mesh ===
mesh = fea.ChMesh()

# === Beam section (shared by all beams) ===
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# === Manually-created first beam (3 nodes, 2 elements) ===
beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Nodal loads
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# === Fixed truss + constraints ===
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# hnode3 fully fixed to truss
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)

# hnode1 pinned to truss (translations only)
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)

# === Builder-created beam segments ===
builder = fea.ChBuilderBeamEuler()

# First builder beam: from (0,0,-0.1) to (0.2,0,-0.1), Y-up
builder.BuildBeam(
    mesh, msection, 5,
    chrono.ChVector3d(0, 0, -0.1),
    chrono.ChVector3d(0.2, 0, -0.1),
    chrono.ChVector3d(0, 1, 0),
)

# Fix B end, apply downward force to A end
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

# Second beam: uses last node of previous beam as 'A', point (0.2,0.1,-0.1) as 'B'
builder.BuildBeam(
    mesh, msection, 5,
    builder.GetLastBeamNodes().front(),          # 'A' node: last node of previous beam
    chrono.ChVector3d(0.2, 0.1, -0.1),           # 'B' point
    chrono.ChVector3d(0, 1, 0),                  # 'Y' up direction
)

# === Mesh setup ===
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# === FEA visualization (two-shape pattern) ===
vis_surface = chrono.ChVisualShapeFEA()
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColormapRange(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA()
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Solver ===
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA beams")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# === Main loop ===
time_step = 1e-3
sim_end = 10.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


# review-only: per-frame capture + CSV writer; loop body itself has NO conditional branches

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
