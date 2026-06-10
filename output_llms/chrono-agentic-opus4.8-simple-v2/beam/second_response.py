import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))         # forced response: no gravity, only tip loads

mesh = fea.ChMesh()                                                    # the FEA mesh that holds beam nodes/elements
mesh.SetAutomaticGravity(False)                                       # static/forced response: no FEA self-weight

# --- Euler-Bernoulli beam section (rectangular cross-section) ---
beam_wy = 0.012                                                       # cross-section width along Y (m)
beam_wz = 0.025                                                       # cross-section width along Z (m)
msection = fea.ChBeamSectionEulerAdvanced()                          # Euler-Bernoulli advanced section
msection.SetAsRectangularSection(beam_wy, beam_wz)                   # rectangular cross-section wy x wz
msection.SetYoungModulus(0.01e9)                                     # E = 10 MPa (soft, large visible bending)
msection.SetShearModulusFromPoisson(0.3)                             # G derived from Poisson nu = 0.3
msection.SetRayleighDamping(0.000)                                   # no internal Rayleigh damping
msection.SetDensity(1000)                                            # density 1000 kg/m^3

# === Euler-Bernoulli beam built by hand (point A to point B, 3 nodes / 2 elements) ===
beam_L = 0.1                                                          # length of the manual beam (m)
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))        # node 1 at A
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 0.5, 0, 0)))  # node 2 at mid-span
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))   # node 3 at B
mesh.AddNode(hnode1)                                                 # register node 1
mesh.AddNode(hnode2)                                                 # register node 2
mesh.AddNode(hnode3)                                                 # register node 3

belement1 = fea.ChElementBeamEuler()                                 # first beam element (node1 -> node2)
belement1.SetNodes(hnode1, hnode2)                                   # connect node 1 and node 2
belement1.SetSection(msection)                                       # assign the Euler section
mesh.AddElement(belement1)                                           # register element 1

belement2 = fea.ChElementBeamEuler()                                 # second beam element (node2 -> node3)
belement2.SetNodes(hnode2, hnode3)                                   # connect node 2 and node 3
belement2.SetSection(msection)                                       # assign the Euler section
mesh.AddElement(belement2)                                           # register element 2

# Apply a force and a torque at the free tip (node 3) of the manual beam
hnode3.SetForce(chrono.ChVector3d(0, 4, 0))                          # tip force 4 N along +Y
hnode3.SetTorque(chrono.ChVector3d(0, 0, 0.02))                      # tip torque 0.02 Nm about Z

# === Modified node-fixing approach: constrain node 1 with ChLinkMateGeneric ===
# hnode1.SetFixed(True)  # replaced: fix node 1 through a constraint to a fixed truss instead
truss = chrono.ChBody()                                              # rigid truss the constraints anchor to
truss.SetFixed(True)                                                 # truss is the world anchor
sys.Add(truss)                                                       # add truss to system

constraint1 = chrono.ChLinkMateGeneric()                            # general 6-DOF constraint to fix node 1
constraint1.Initialize(hnode1, truss, False, hnode1.Frame(), hnode1.Frame())  # tie node 1 to the truss frame
sys.Add(constraint1)                                                 # register the constraint
constraint1.SetConstrainedCoords(True, True, True,                  # tx, ty, tz fixed
                                 True, True, True)                  # rx, ry, rz fixed

# === Euler-Bernoulli beam built with the ChBuilderBeamEuler helper object ===
builder = fea.ChBuilderBeamEuler()                                  # helper that builds nodes + elements
builder.BuildBeam(mesh,                                             # target mesh
                  msection,                                         # Euler section (shared)
                  5,                                                # 5 beam elements
                  chrono.ChVector3d(0, 0, -0.1),                    # point A
                  chrono.ChVector3d(0.2, 0, -0.1),                  # point B
                  chrono.ChVector3d(0, 1, 0))                       # 'Y' up suggested section direction

# Keep a strong reference to the SWIG container before indexing (GC safety)
built_nodes = builder.GetLastBeamNodes()                            # built beam node container
built_nodes.back().SetFixed(True)                                   # fix the LAST node of the built beam
built_nodes.front().SetForce(chrono.ChVector3d(0, -1, 0))           # force (0,-1,0) on the FIRST node

sys.Add(mesh)                                                       # register the mesh in the system

# === FEA visualization shapes (surface bending field + node glyphs) ===
vis_beam_surf = chrono.ChVisualShapeFEA(mesh)                       # coloured surface/bending field shape
vis_beam_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz field
vis_beam_surf.SetColorscaleMinMax(-0.4, 0.4)                        # colour range for Mz (lo, hi)
vis_beam_surf.SetSmoothFaces(True)                                  # smooth shaded faces
vis_beam_surf.SetWireframe(False)                                   # solid (not wireframe)
mesh.AddVisualShapeFEA(vis_beam_surf)                               # register surface shape

vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)                      # node coordinate-system glyphs
vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # draw node triads
vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)         # no field on the glyph shape
vis_beam_nodes.SetSymbolsThickness(0.006)                          # triad line thickness
vis_beam_nodes.SetSymbolsScale(0.01)                               # triad size
vis_beam_nodes.SetZbufferHide(False)                               # always draw glyphs on top
mesh.AddVisualShapeFEA(vis_beam_nodes)                             # register glyph shape

# === Irrlicht visualization (Initialize first, then scene elements; NO grid) ===
vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht render window
vis.AttachSystem(sys)                                              # bind the system's visual assets
vis.SetWindowSize(1024, 768)                                       # window resolution
vis.SetWindowTitle("FEA Euler-Bernoulli beams")                   # window title
vis.Initialize()                                                  # create the device (BEFORE scene elements)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo overlay
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2),                    # camera eye
              chrono.ChVector3d(0.05, 0, -0.05))                  # camera target near the beams
vis.AddTypicalLights()                                             # standard two-light setup

# === Solver + timestepper for stiff Euler beams ===
pardiso_solver = mkl.ChSolverPardisoMKL()                          # direct sparse Pardiso MKL solver
sys.SetSolver(pardiso_solver)                                      # required for stiff beam stiffness matrices

ts = chrono.ChTimestepperHHT(sys)                                 # implicit HHT timestepper
ts.SetStepControl(False)                                          # canonical-minimal HHT (no adaptive control)
sys.SetTimestepper(ts)                                            # assign the timestepper

# === Time-stepping loop (real-time render-cadence) ===
time_step = 1e-3                                                  # integration step (s)
sim_end = 5.0                                                     # total simulated time (s)
render_fps = 50.0                                                 # target review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
