import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up world, g = 9.81 down

mesh = fea.ChMesh()                                                  # FEA container for nodes/elements
mesh.SetAutomaticGravity(False)                                     # static/forced response — no FEA gravity

beam_wy = 0.012                                                      # beam cross-section width (y)
beam_wz = 0.025                                                      # beam cross-section height (z)

msection = fea.ChBeamSectionEulerAdvanced()                         # Euler-Bernoulli section
msection.SetAsRectangularSection(beam_wy, beam_wz)                  # rectangular cross section
msection.SetYoungModulus(0.01e9)                                    # E (soft beam for visible bending)
msection.SetShearModulusFromPoisson(0.22)                          # G derived from Poisson ratio
msection.SetRayleighDamping(0.01)                                  # structural damping
msection.SetCentroid(0, 0.02)                                       # offset centroid (advanced section)
msection.SetShearCenter(0, 0.1)                                    # offset shear center
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)             # twist the section frame

# --- Beam 1: explicit nodes + ChElementBeamEuler, root clamped, tip loaded ---
beam_L1 = 0.1                                                       # first beam length
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))      # root node
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L1 * 0.5, 0, 0)))  # mid node
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L1, 0, 0)))        # tip node
mesh.AddNode(hnode1)                                               # register nodes with the mesh
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()                              # first beam element
belement1.SetNodes(hnode1, hnode2)                               # spans node1 -> node2
belement1.SetSection(msection)                                   # assign the Euler section
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()                             # second beam element
belement2.SetNodes(hnode2, hnode3)                              # spans node2 -> node3
belement2.SetSection(msection)
mesh.AddElement(belement2)

hnode1.SetFixed(True)                                            # clamp the root node to ground

hnode3.SetForce(chrono.ChVector3d(4, 2, 0))                    # apply a tip force (N)
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))              # apply a tip torque (Nm)

# --- Beam 2: built with ChBuilderBeamEuler from a fresh A point to a B point ---
msection2 = fea.ChBeamSectionEulerAdvanced()                   # a second, simpler section
msection2.SetAsCircularSection(0.012)                         # round cross section, diameter 12 mm
msection2.SetYoungModulus(0.01e9)                            # same modulus
msection2.SetShearModulusFromPoisson(0.22)
msection2.SetRayleighDamping(0.01)

builder = fea.ChBuilderBeamEuler()                           # helper that auto-creates nodes/elements
builder.BuildBeam(mesh,                                      # target mesh
                  msection2,                                 # section to use
                  5,                                         # number of elements in this beam
                  chrono.ChVector3d(0, 0, -0.1),            # A point (start)
                  chrono.ChVector3d(0.2, 0, -0.1),          # B point (end)
                  chrono.ChVector3d(0, 1, 0))               # Y up direction

prev_nodes = builder.GetLastBeamNodes()                     # keep a strong ref (SWIG GC) to this beam's nodes
prev_nodes.front().SetFixed(True)                          # clamp the start node of this beam to ground

prev_last_node = prev_nodes.back()                         # last node created by the previous beam

# --- Beam 3 (the added segment): A = last node of the previous beam, B = (0.2, 0.1, -0.1) ---
builder.BuildBeam(mesh,                                      # same mesh
                  msection2,                                 # reuse the circular section
                  5,                                         # element count for the new segment
                  prev_last_node,                            # 'A' node = last node created by previous beam
                  chrono.ChVector3d(0.2, 0.1, -0.1),         # 'B' point endpoint
                  chrono.ChVector3d(0, 1, 0))               # 'Y' up direction

builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -1, 0))  # load the free tip of the new segment (N)

sys.Add(mesh)                                               # register the mesh with the system

# --- FEA visualization: a moment-field surface shape + a node-CSYS glyph shape ---
vis_surface = chrono.ChVisualShapeFEA(mesh)                # surface/scalar field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz field
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                # color range for the field
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                  # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate-system triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# --- Solver + timestepper for stiff Euler beams ---
msolver = mkl.ChSolverPardisoMKL()                         # direct sparse solver (stiff matrices)
sys.SetSolver(msolver)

ts = chrono.ChTimestepperHHT(sys)                          # HHT implicit timestepper
ts.SetStepControl(False)                                  # canonical-minimal HHT config
sys.SetTimestepper(ts)

# --- Irrlicht visualization window (Initialize first, then scene elements, no grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Euler beams FEA")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.5), chrono.ChVector3d(0.1, 0, 0))   # eye, target
vis.AddTypicalLights()

time_step = 1e-3                                          # stiff-beam timestep
sim_end = 5.0                                             # simulation duration
render_fps = 50.0                                         # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged render-cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
