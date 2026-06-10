import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # SMC system for FEA

mesh = fea.ChMesh()                                                  # FEA mesh container

section = fea.ChBeamSectionCable()                                   # ANCF cable section
section.SetDiameter(0.015)                                           # cable diameter [m]
section.SetYoungModulus(0.01e9)                                      # Young's modulus [Pa]
section.SetRayleighDamping(0.0001)                                   # Rayleigh damping

builder = fea.ChBuilderCableANCF()                                   # ANCF cable builder
builder.BuildBeam(mesh, section, 10,                                 # 10 cable elements
                  chrono.ChVector3d(0, 0, -0.1),                     # endpoint A
                  chrono.ChVector3d(0.5, 0, -0.1))                   # endpoint B

cable_nodes = builder.GetLastBeamNodes()                             # keep ref (SWIG GC)
front_node = cable_nodes.front()                                     # front node of cable
back_node = cable_nodes.back()                                       # back node of cable

front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))                   # applied force on front node

truss = chrono.ChBody()                                              # fixed truss anchor
truss.SetFixed(True)                                                 # immovable
sys.Add(truss)

constraint = fea.ChLinkNodeFrame()                                   # hinge back node to truss
constraint.Initialize(back_node, truss)                              # attach to fixed truss
sys.Add(constraint)

sys.Add(mesh)                                                        # add mesh to system

vis_surface = chrono.ChVisualShapeFEA(mesh)                          # scalar field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                           # color range
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                            # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

solver = chrono.ChSolverMINRES()                                     # MINRES iterative solver
print("Using MINRES solver")                                         # solver-type announcement
solver.SetMaxIterations(200)                                         # max solver iterations
solver.SetTolerance(1e-10)                                           # solver tolerance
solver.EnableDiagonalPreconditioner(True)                            # diagonal preconditioner
solver.EnableWarmStart(True)                                         # warm start
solver.SetVerbose(False)                                             # quiet solver
sys.SetSolver(solver)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)                # linearized implicit stepper
sys.SetTimestepper(ts)

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF cable")
vis.Initialize()                                                     # device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))                         # camera eye
vis.AddTypicalLights()

time_step = 0.01                                                     # ANCF cable timestep
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
