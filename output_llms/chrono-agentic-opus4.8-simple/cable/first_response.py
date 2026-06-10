import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # FEA truths use SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity

mesh = fea.ChMesh()                                                  # FEA container
mesh.SetAutomaticGravity(True)                                       # let gravity load the cable

sec_cable = fea.ChBeamSectionCable()                                 # ANCF cable section
sec_cable.SetDiameter(0.015)                                         # cable diameter (m)
sec_cable.SetYoungModulus(0.01e9)                                    # soft, flexible cable
sec_cable.SetRayleighDamping(0.000)                                  # no structural damping

builder = fea.ChBuilderCableANCF()                                   # ANCF cable builder
builder.BuildBeam(mesh, sec_cable, 10,                               # 10 ANCF cable elements
                  chrono.ChVector3d(0, 0, -0.1),                     # A — hinged end
                  chrono.ChVector3d(0.5, 0, -0.1))                   # B — free end

beam_nodes = builder.GetLastBeamNodes()                              # keep strong ref (SWIG GC)
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]           # node list, no dangling

truss = chrono.ChBody()                                             # fixed ground truss
truss.SetFixed(True)                                                # immovable
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()                                       # hinge one end to ground
hinge.Initialize(nodes[0], truss)                                   # pin first node to truss
sys.Add(hinge)

vis_surface = chrono.ChVisualShapeFEA(mesh)                         # bending-moment field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)   # Mz colouring
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                          # colour range (lo, hi)
vis_surface.SetSmoothFaces(True)                                   # smooth shading
vis_surface.SetWireframe(False)                                    # solid surface
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                          # nodal-position glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)   # node dots
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)    # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                              # glyph thickness
vis_glyph.SetSymbolsScale(0.01)                                  # glyph scale
vis_glyph.SetZbufferHide(False)                                  # always draw node markers
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)                                                     # register the mesh

solver = chrono.ChSolverSparseQR()                               # direct solver for ANCF
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)                           # learn sparsity pattern
solver.LockSparsityPattern(True)                                 # lock it for speed
solver.SetVerbose(False)                                         # quiet solver

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)            # linearized implicit timestepper
sys.SetTimestepper(ts)

vis = chronoirr.ChVisualSystemIrrlicht()                         # Irrlicht render window
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)               # Y-up camera convention
vis.SetWindowSize(1280, 720)                                    # window resolution
vis.SetWindowTitle("ANCF Cable Beam")                           # window title
vis.Initialize()                                               # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo (after Initialize)
vis.AddSkyBox()                                                # sky box
vis.AddCamera(chrono.ChVector3d(0.0, 0.2, -1.0),               # eye
              chrono.ChVector3d(0.25, -0.1, -0.1))             # look-at cable midspan
vis.AddTypicalLights()                                         # standard lighting

time_step = 0.01                                               # ANCF cable timestep
sim_end = 10.0                                                 # simulation duration (s)
render_fps = 50.0                                              # target review fps
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
