import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                          # SMC system for FEA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # Y-up gravity

mesh = fea.ChMesh()                                                 # FEA mesh container
mesh.SetAutomaticGravity(True)                                      # let gravity act on the cable

sec_cable = fea.ChBeamSectionCable()                                # ANCF cable cross-section
sec_cable.SetDiameter(0.015)                                        # cable diameter [m]
sec_cable.SetYoungModulus(0.01e9)                                   # flexible cable Young modulus [Pa]
sec_cable.SetRayleighDamping(0.000)                                 # no structural damping

builder = fea.ChBuilderCableANCF()                                  # ANCF cable builder
builder.BuildBeam(mesh, sec_cable, 10,                              # 10 ANCF cable elements
                  chrono.ChVector3d(0, 0, -0.1),                    # node A
                  chrono.ChVector3d(0.5, 0, -0.1))                  # node B

cable_nodes = builder.GetLastBeamNodes()                            # keep strong ref (SWIG GC pitfall)

truss = chrono.ChBody()                                             # fixed ground/truss body
truss.SetFixed(True)                                                # immovable
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()                                       # hinge one end to ground
hinge.Initialize(cable_nodes.back(), truss)                         # pin the last node to the truss
sys.Add(hinge)

sys.Add(mesh)                                                       # register the mesh

vis_surface = chrono.ChVisualShapeFEA(mesh)                         # scalar-field shape (deformation)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending-moment Mz field
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                          # color range (lo, hi)
vis_surface.SetSmoothFaces(True)                                    # smooth surface shading
vis_surface.SetWireframe(False)                                     # solid surface
mesh.AddVisualShapeFEA(vis_surface)

vis_nodes = chrono.ChVisualShapeFEA(mesh)                           # node-glyph shape (nodal positions)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # dots at node positions
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)     # no field on the glyphs
vis_nodes.SetSymbolsThickness(0.006)                                # glyph dot size
vis_nodes.SetSymbolsScale(0.01)                                     # glyph scale
vis_nodes.SetZbufferHide(False)                                     # always draw glyphs on top
mesh.AddVisualShapeFEA(vis_nodes)

solver = chrono.ChSolverSparseQR()                                  # direct sparse solver for ANCF
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)                              # learn sparsity pattern
solver.LockSparsityPattern(True)                                    # then lock it for speed
solver.SetVerbose(False)                                            # quiet solver

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)               # implicit linearized stepper
sys.SetTimestepper(ts)

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                   # Y-up camera
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetWindowTitle("ANCF cable under gravity")                      # window title
vis.Initialize()                                                    # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo
vis.AddSkyBox()                                                     # sky box
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1),                        # eye
              chrono.ChVector3d(0, 0, 0))                           # target
vis.AddTypicalLights()                                              # standard lights

time_step = 0.01                                                    # ANCF cable timestep [s]
render_fps = 50.0                                                   # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per frame
while vis.Run():                                                    # real-time render loop
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
