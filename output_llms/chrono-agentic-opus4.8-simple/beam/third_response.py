import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up world, g down

mesh = fea.ChMesh()                                                   # FEA mesh container
mesh.SetAutomaticGravity(False)                                       # static/forced response, no FEA gravity

sec = fea.ChBeamSectionEulerAdvanced()                               # Euler-Bernoulli beam section
sec.SetAsRectangularSection(0.012, 0.025)                            # wy, wz cross-section
sec.SetYoungModulus(0.01e9)                                          # soft modulus
sec.SetShearModulusFromPoisson(0.3)                                  # G from Poisson nu = 0.3
sec.SetRayleighDamping(0.000)                                        # no Rayleigh damping
sec.SetDensity(1000)                                                 # kg/m^3

builder = fea.ChBuilderBeamEuler()                                   # Euler beam builder

builder.BuildBeam(mesh, sec, 5,                                      # 5 elements, first beam segment
                  chrono.ChVector3d(0, 0, -0.1),                     # A point
                  chrono.ChVector3d(0.2, 0, -0.1),                   # B point
                  chrono.ChVector3d(0, 1, 0))                        # Y up direction

builder.GetLastBeamNodes().back().SetFixed(True)                    # clamp the first node of the first beam

last_node = builder.GetLastBeamNodes().front()                       # last node created by the previous beam ('A' node)
last_node.SetForce(chrono.ChVector3d(0, -1, 0))                      # apply a tip load on the first segment end

builder.BuildBeam(mesh, sec, 5,                                      # 5 elements, second beam segment
                  last_node,                                          # 'A' node = last node of previous beam
                  chrono.ChVector3d(0.2, 0.1, -0.1),                 # 'B' point endpoint
                  chrono.ChVector3d(0, 1, 0))                        # 'Y' up direction

builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -2, 0))   # tip load on the second segment end

sys.Add(mesh)                                                        # register the mesh

vis_surface = chrono.ChVisualShapeFEA(mesh)                          # surface/scalar field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)   # bending moment Mz
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                          # color range (lo, hi)
vis_surface.SetSmoothFaces(True)                                    # smooth shading
vis_surface.SetWireframe(False)                                     # solid surface
mesh.AddVisualShapeFEA(vis_surface)                                 # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                           # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)   # coordinate-system triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)     # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                               # glyph thickness
vis_glyph.SetSymbolsScale(0.01)                                    # glyph scale
vis_glyph.SetZbufferHide(False)                                    # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)                                  # register glyph shape

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht render window
vis.AttachSystem(sys)                                              # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                 # Y-up camera
vis.SetWindowSize(1280, 720)                                      # window size
vis.SetWindowTitle("FEA beams (Euler)")                          # window title
vis.Initialize()                                                  # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                   # sky box
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.5),                  # eye
              chrono.ChVector3d(0.1, 0.0, -0.1))                  # target
vis.AddTypicalLights()                                            # standard lights

solver = mkl.ChSolverPardisoMKL()                                 # direct solver for stiff beams
sys.SetSolver(solver)                                             # use Pardiso MKL

ts = chrono.ChTimestepperHHT(sys)                                 # HHT timestepper
ts.SetStepControl(False)                                         # canonical-minimal HHT
sys.SetTimestepper(ts)                                           # install timestepper

time_step = 1e-3                                                 # step size for stiff beam
sim_end = 5.0                                                    # simulation duration
render_fps = 50.0                                               # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
tip = builder.GetLastBeamNodes().back()                        # tip node of the second segment
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
