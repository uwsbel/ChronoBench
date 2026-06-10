import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

solver = chrono.ChSolverMINRES()                                      # MINRES iterative solver

sys = chrono.ChSystemSMC()                                            # SMC system for the FEA cable

mesh = fea.ChMesh()                                                   # container for FEA nodes/elements

sec_cable = fea.ChBeamSectionCable()                                  # ANCF cable section (shared)
sec_cable.SetDiameter(0.015)                                          # cable diameter 15 mm
sec_cable.SetYoungModulus(0.01e9)                                     # soft cable, E = 0.01 GPa
sec_cable.SetRayleighDamping(0.0001)                                  # Rayleigh damping beta = 0.0001

builder = fea.ChBuilderCableANCF()                                    # subdivides the beam into ANCF elements
builder.BuildBeam(mesh,                                               # mesh to receive nodes/elements
                  sec_cable,                                          # cable section to use
                  10,                                                 # number of ANCF cable elements
                  chrono.ChVector3d(0, 0, -0.1),                      # point A (beginning of beam)
                  chrono.ChVector3d(0.5, 0, -0.1))                    # point B (end of beam)

beam_nodes = builder.GetLastBeamNodes()                               # keep a strong ref (SWIG GC)
beam_nodes.front().SetForce(chrono.ChVector3d(0, -0.7, 0))            # constant pull on the front node

mtruss = chrono.ChBody()                                              # fixed truss to anchor the cable
mtruss.SetFixed(True)                                                 # immovable ground
sys.Add(mtruss)                                                       # register the truss

constraint_hinge = fea.ChLinkNodeFrame()                             # pin the B-end node to the truss
constraint_hinge.Initialize(beam_nodes.back(), mtruss)               # hinge back node to ground
sys.Add(constraint_hinge)                                             # register the constraint

sys.Add(mesh)                                                         # add the FEA mesh to the system

visualizebeamA = chrono.ChVisualShapeFEA(mesh)                       # surface/scalar field shape
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)                        # colour range (lo, hi)
visualizebeamA.SetSmoothFaces(True)                                  # smooth shading
visualizebeamA.SetWireframe(False)                                   # solid surface
mesh.AddVisualShapeFEA(visualizebeamA)                               # register surface shape

visualizebeamB = chrono.ChVisualShapeFEA(mesh)                       # node-glyph shape
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # node dots
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
visualizebeamB.SetSymbolsThickness(0.006)                           # glyph thickness
visualizebeamB.SetSymbolsScale(0.01)                                # glyph scale
visualizebeamB.SetZbufferHide(False)                                # always draw glyphs
mesh.AddVisualShapeFEA(visualizebeamB)                              # register glyph shape

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)                                               # bind the physical system
vis.SetWindowSize(1024, 768)                                        # window resolution
vis.SetWindowTitle('FEA cables')                                    # window title
vis.Initialize()                                                   # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))   # PyChrono logo
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))                        # camera eye position
vis.AddTypicalLights()                                             # standard two-light setup

print("Using MINRES solver")                                       # solver banner
sys.SetSolver(solver)                                              # install the MINRES solver
solver.SetMaxIterations(200)                                       # max iterations
solver.SetTolerance(1e-10)                                         # convergence tolerance
solver.EnableDiagonalPreconditioner(True)                         # diagonal preconditioner
solver.EnableWarmStart(True)                                       # warm start (needed w/ linearized integrator)
solver.SetVerbose(False)                                           # silence solver output

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)              # linearized implicit integrator
sys.SetTimestepper(ts)                                             # install integrator

time_step = 0.01                                                   # ANCF cable timestep
sim_end = 10.0                                                     # simulation end time
render_fps = 50.0                                                  # target review fps
render_every = max(1, round(1.0 / (render_fps * time_step)))      # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                             # advance one ANCF cable step
        if sys.GetChTime() >= sim_end:
            break
