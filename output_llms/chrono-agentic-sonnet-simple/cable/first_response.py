import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # SMC for FEA/ANCF

# ANCF cable solver: sparse QR + linearized implicit timestepper
solver = chrono.ChSolverSparseQR()                                   # sparse QR for ANCF
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)                               # learn sparsity once
solver.LockSparsityPattern(True)                                     # lock pattern for speed
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)               # linearized implicit for ANCF
sys.SetTimestepper(ts)

# Cable section properties
beam_L = 0.5                                                         # cable length [m]
sec_cable = fea.ChBeamSectionCable()                                 # ANCF cable section
sec_cable.SetDiameter(0.015)                                         # 15 mm diameter
sec_cable.SetYoungModulus(0.01e9)                                    # 10 MPa (flexible cable)
sec_cable.SetRayleighDamping(0.000)                                  # no Rayleigh damping

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(True)                                       # apply gravity to FEA nodes

builder = fea.ChBuilderCableANCF()                                   # ANCF cable builder
builder.BuildBeam(mesh, sec_cable, 10,
                  chrono.ChVector3d(0, 0, -0.1),                    # start node
                  chrono.ChVector3d(beam_L, 0, -0.1))               # end node

# Pin one end (hinge) to ground using ChLinkNodeFrame
truss = chrono.ChBody()                                              # fixed ground body
truss.SetFixed(True)                                                 # anchor to ground
sys.Add(truss)

# SWIG GC pitfall: store container before indexing
beam_nodes = builder.GetLastBeamNodes()                              # keep strong ref
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]           # list of all nodes

hinge = fea.ChLinkNodeFrame()                                        # hinge constraint (3 DOF)
hinge.Initialize(nodes[0], truss)                                    # pin first node to ground
sys.Add(hinge)

sys.Add(mesh)                                                        # register mesh with system

# FEA visualization: surface shape + glyph shape
vis_beam = chrono.ChVisualShapeFEA(mesh)                            # surface/scalar field shape
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
vis_beam.SetColorscaleMinMax(-0.4, 0.4)                             # colorscale range
vis_beam.SetSmoothFaces(True)                                        # smooth surface
vis_beam.SetWireframe(False)                                         # solid rendering
mesh.AddVisualShapeFEA(vis_beam)                                     # attach to mesh

vis_glyph = chrono.ChVisualShapeFEA(mesh)                           # glyph shape for nodes
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # node position dots
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)    # no scalar data on glyphs
vis_glyph.SetSymbolsThickness(0.006)                                # dot thickness
vis_glyph.SetSymbolsScale(0.01)                                     # dot scale
vis_glyph.SetZbufferHide(False)                                     # always visible
mesh.AddVisualShapeFEA(vis_glyph)                                   # attach to mesh

# Irrlicht visualization: Initialize FIRST, then add scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z-up convention
vis.SetWindowSize(1280, 720)                                         # window size
vis.SetWindowTitle("ANCF Cable Demo")
vis.Initialize()                                                     # MUST be called first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # Chrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddCamera(chrono.ChVector3d(0.0, -1.0, 0.2),                   # eye position
              chrono.ChVector3d(0.25, 0, -0.1))                     # look-at target (cable midpoint)
vis.AddTypicalLights()                                               # standard lighting

time_step = 0.01                                                     # 10 ms step for ANCF cable
sim_end = 10.0                                                       # run for 10 s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
