import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity, g = 9.81

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(True)                                       # let gravity load the beam

beam_L = 2.0                                                         # beam length (m)
n_elements = 16                                                      # number of beam elements
beam_d = 0.010                                                       # circular cross-section diameter (m)

sec = fea.ChBeamSectionEulerAdvanced()                              # Euler-Bernoulli beam section
sec.SetAsCircularSection(beam_d)                                    # circular cross-section
sec.SetDensity(2700)                                               # aluminium density (kg/m^3)
sec.SetYoungModulus(73e9)                                          # Young's modulus E (Pa)
sec.SetShearModulusFromPoisson(0.3)                                # derive G from Poisson nu = 0.3
sec.SetRayleighDamping(0.001)                                      # small structural damping

up = chrono.ChVector3d(0, 1, 0)                                     # lateral reference direction
builder = fea.ChBuilderBeamEuler()                                 # Euler beam builder
builder.BuildBeam(mesh, sec, n_elements,                           # build the beam mesh
                  chrono.ChVector3d(0, 0, 0),                       # start point A
                  chrono.ChVector3d(beam_L, 0, 0),                  # end point B
                  up)                                              # section Y reference

beam_nodes = builder.GetLastBeamNodes()                            # keep strong ref (SWIG GC)
beam_nodes.front().SetFixed(True)                                  # clamp the root node -> cantilever
tip = beam_nodes.back()                                           # tip node (kept alive)

sys.Add(mesh)                                                     # register mesh in the system

vis_surface = chrono.ChVisualShapeFEA(mesh)                       # scalar field shape (mesh is ctor arg)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz field
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                        # color scale (lo, hi)
vis_surface.SetSmoothFaces(True)                                 # smooth shaded faces
vis_surface.SetWireframe(False)                                  # solid, not wireframe
mesh.AddVisualShapeFEA(vis_surface)                              # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                        # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node coordinate triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                             # glyph line thickness
vis_glyph.SetSymbolsScale(0.01)                                  # glyph size scale
vis_glyph.SetZbufferHide(False)                                 # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)                               # register glyph shape

solver = mkl.ChSolverPardisoMKL()                               # direct solver for stiff beam matrix
sys.SetSolver(solver)                                          # use Pardiso MKL

ts = chrono.ChTimestepperHHT(sys)                              # HHT timestepper (canonical-minimal)
ts.SetStepControl(False)                                      # disable adaptive step control
sys.SetTimestepper(ts)                                        # use HHT

vis = chronoirr.ChVisualSystemIrrlicht()                      # Irrlicht visualization
vis.AttachSystem(sys)                                         # attach the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)            # Y-up world
vis.SetWindowSize(1280, 720)                                # window resolution
vis.SetWindowTitle("FEA Beam Elements")                     # window title
vis.Initialize()                                            # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                            # sky box
vis.AddCamera(chrono.ChVector3d(1.0, 0.4, 2.6),           # camera eye
              chrono.ChVector3d(1.0, -0.3, 0))             # look-at target
vis.AddTypicalLights()                                    # standard lights

time_step = 1e-3                                          # stiff-beam timestep
sim_end = 5.0                                             # simulation duration (s)
render_fps = 50.0                                         # target render frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))       # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
