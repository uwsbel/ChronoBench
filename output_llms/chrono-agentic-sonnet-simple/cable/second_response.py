import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# A beam composed of 10 ANCF cable elements, one end hinged to ground, with
# modified Rayleigh damping (0.0001), larger applied force (-0.7), and MINRES solver.

class Model1:
    def __init__(self, system, mesh):
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)              # diameter 15 mm
        msection_cable2.SetYoungModulus(0.01e9)         # Young's modulus 0.01 GPa
        msection_cable2.SetRayleighDamping(0.0001)      # Rayleigh damping changed from 0.000 to 0.0001

        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(
            mesh,                                        # mesh to add nodes/elements to
            msection_cable2,                             # beam section properties
            10,                                          # number of ANCF elements
            chrono.ChVector3d(0, 0, -0.1),               # start point A
            chrono.ChVector3d(0.5, 0, -0.1)              # end point B
        )

        # Apply force to front node - changed from (0,-0.2,0) to (0,-0.7,0)
        builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)                            # fixed reference frame

        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
        system.Add(constraint_hinge)                     # add hinge constraint to system

sys = chrono.ChSystemSMC()                               # SMC system for FEA
mesh = fea.ChMesh()

model = Model1(sys, mesh)
sys.Add(mesh)                                            # add mesh to physical system

# FEM visualization - surface color-coded by bending moment Mz
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # beam bending moment Mz
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)           # color scale range
visualizebeamA.SetSmoothFaces(True)                     # smooth face rendering
visualizebeamA.SetWireframe(False)                      # solid (non-wireframe) mode
mesh.AddVisualShapeFEA(visualizebeamA)                  # attach surface vis to mesh

# FEM visualization - node dot glyphs
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # dots at nodes
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no additional data
visualizebeamB.SetSymbolsThickness(0.006)               # symbol thickness
visualizebeamB.SetSymbolsScale(0.01)                    # symbol scale
visualizebeamB.SetZbufferHide(False)                    # don't hide behind geometry
mesh.AddVisualShapeFEA(visualizebeamB)                  # attach glyph vis to mesh

# Irrlicht visualization setup - Initialize() first, then scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)                                    # attach to Chrono system
vis.SetWindowSize(1024, 768)                             # window dimensions
vis.SetWindowTitle('FEA cables')                        # window title
vis.Initialize()                                         # MUST be called before scene elements
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Chrono logo
vis.AddSkyBox()                                         # sky background
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))            # camera position
vis.AddTypicalLights()                                   # standard lighting

# Set solver to MINRES (changed from SparseQR)
solver = chrono.ChSolverMINRES()                         # MINRES iterative solver
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")                         # confirm solver type
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)                         # max 200 iterations
    solver.SetTolerance(1e-10)                           # tight tolerance
    solver.EnableDiagonalPreconditioner(True)            # diagonal preconditioner for convergence
    solver.EnableWarmStart(True)                         # warm start improves convergence
    solver.SetVerbose(False)                             # suppress verbose output

# ANCF cable timestepper - linearized Euler implicit
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)                                   # set timestepper on system

time_step = 0.01                                         # simulation time step 0.01 s
sim_end = 10.0                                           # run for 10 seconds
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                     # begin frame rendering
    vis.Render()                                         # render scene
    vis.EndScene()                                       # end frame rendering
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                    # advance simulation by one step
        if sys.GetChTime() >= sim_end:
            break
