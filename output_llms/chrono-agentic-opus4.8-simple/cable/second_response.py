import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # FEA uses SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(True)                                       # let gravity load the cable

sec_cable = fea.ChBeamSectionCable()                                 # ANCF cable section
sec_cable.SetDiameter(0.015)                                         # cable diameter [m]
sec_cable.SetYoungModulus(0.01e9)                                    # soft cable stiffness [Pa]
sec_cable.SetRayleighDamping(0.0001)                                 # Rayleigh damping for the cable

builder = fea.ChBuilderCableANCF()                                   # ANCF cable builder
builder.BuildBeam(mesh, sec_cable, 10,                               # 10 ANCF beam elements
                  chrono.ChVector3d(0, 0, -0.1),                     # A — start point
                  chrono.ChVector3d(0.5, 0, -0.1))                   # B — end point

beam_nodes = builder.GetLastBeamNodes()                              # keep a strong ref (SWIG GC)
front_node = beam_nodes.front()                                      # front (free) node
back_node = beam_nodes.back()                                        # back node to pin

front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))                   # downward force on the front node

truss = chrono.ChBody()                                              # fixed truss anchor
truss.SetFixed(True)                                                 # truss does not move
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()                                        # pin the back node to the truss
hinge.Initialize(back_node, truss)
sys.Add(hinge)

sys.Add(mesh)                                                        # register the FEA mesh

vis_surface = chrono.ChVisualShapeFEA(mesh)                          # surface/scalar field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                          # color range (lo, hi)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                           # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)   # node coordinate systems
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

solver = chrono.ChSolverMINRES()                                    # MINRES iterative solver
sys.SetSolver(solver)
print("Using MINRES solver")                                        # solver-type banner
solver.SetMaxIterations(200)                                        # max solver iterations
solver.SetTolerance(1e-10)                                          # solver tolerance
solver.EnableDiagonalPreconditioner(True)                          # diagonal preconditioner
solver.EnableWarmStart(True)                                       # warm start
solver.SetVerbose(False)                                           # quiet solver

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)              # linearized implicit timestepper
sys.SetTimestepper(ts)

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable")
vis.Initialize()                                                  # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.25, 0.0, 1.0), chrono.ChVector3d(0.25, 0.0, -0.1))
vis.AddTypicalLights()

time_step = 0.01                                                   # ANCF cable timestep
sim_end = 10.0                                                     # simulation end time [s]
render_fps = 50.0                                                  # target render rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
