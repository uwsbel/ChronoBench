import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Model1: A set of beam elements with connected bodies, each with different
#         number of ANCF cable elements.

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.bodies = []  # end bodies for each chain
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)                            # cable diameter 15 mm
        msection_cable2.SetYoungModulus(0.01e9)                       # Young's modulus 0.01 GPa
        msection_cable2.SetRayleighDamping(0.0001)                    # Rayleigh damping

        mtruss = chrono.ChBody()                                       # fixed reference frame
        mtruss.SetFixed(True)

        for j in range(n_chains):
            builder = fea.ChBuilderCableANCF()

            # Build first beam segment for chain j
            builder.BuildBeam(
                mesh,                                                  # mesh to add nodes/elements to
                msection_cable2,                                       # cable section properties
                1 + j,                                                 # number of ANCF elements (increases per chain)
                chrono.ChVector3d(0, 0, -0.1 * j),                    # poA (beginning of beam)
                chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j)        # poB (end of beam)
            )

            # Apply downward force on the last node of first beam
            beam_nodes_first = builder.GetLastBeamNodes()              # keep strong ref (SWIG GC)
            beam_nodes_first.back().SetForce(chrono.ChVector3d(0, -0.2, 0))  # downward force

            # Hinge constraint: pin front of first beam to truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(beam_nodes_first.front(), mtruss)  # front node hinged to truss
            system.Add(constraint_hinge)

            msphere = chrono.ChVisualShapeSphere(0.02)                 # sphere marker at hinge
            constraint_hinge.AddVisualShape(msphere)

            # Create box body at end of first beam
            mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)        # connecting box body
            mbox.SetPos(beam_nodes_first.back().GetPos() + chrono.ChVector3d(0.1, 0, 0))  # position at beam end
            system.Add(mbox)

            # Connect first beam's last node to mbox
            constraint_pos = fea.ChLinkNodeFrame()
            constraint_pos.Initialize(beam_nodes_first.back(), mbox)  # position constraint
            system.Add(constraint_pos)

            constraint_dir = fea.ChLinkNodeSlopeFrame()
            constraint_dir.Initialize(beam_nodes_first.back(), mbox)  # slope/direction constraint
            constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir)

            # Build second beam segment: from mbox to another end
            builder.BuildBeam(
                mesh,
                msection_cable2,
                1 + (n_chains - j),                                    # decreasing elements per chain
                chrono.ChVector3d(mbox.GetPos().x + 0.1, 0, -0.1 * j),              # poA start at mbox
                chrono.ChVector3d(mbox.GetPos().x + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j)  # poB
            )

            beam_nodes_second = builder.GetLastBeamNodes()             # keep strong ref (SWIG GC)

            # Connect second beam's front to mbox
            constraint_pos2 = fea.ChLinkNodeFrame()
            constraint_pos2.Initialize(beam_nodes_second.front(), mbox)  # front of second beam to mbox
            system.Add(constraint_pos2)

            constraint_dir2 = fea.ChLinkNodeSlopeFrame()
            constraint_dir2.Initialize(beam_nodes_second.front(), mbox)  # slope constraint
            constraint_dir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir2)

            # Create end box body for this chain
            self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000))  # end body
            self.bodies[j].SetPos(beam_nodes_second.back().GetPos() + chrono.ChVector3d(0.1, 0, 0))  # at beam end
            system.Add(self.bodies[j])

            # Connect second beam's last node to end body
            constraint_pos3 = fea.ChLinkNodeFrame()
            constraint_pos3.Initialize(beam_nodes_second.back(), self.bodies[j])  # position constraint
            system.Add(constraint_pos3)

            constraint_dir3 = fea.ChLinkNodeSlopeFrame()
            constraint_dir3.Initialize(beam_nodes_second.back(), self.bodies[j])  # slope constraint
            constraint_dir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir3)

    def PrintBodyPositions(self):
        print("Time: " + str(self.bodies[0].GetChTime()))              # print current time
        for body in self.bodies:
            print("  " + str(body.GetPos()))                           # print position of each end body

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()                                             # SMC contact model for FEA
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)                                                          # add mesh to physical system

# Add visualization for FEM mesh — beam moment Mz color field
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # display moments along beam
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)                         # color scale for moment visualization
visualizebeamA.SetSmoothFaces(True)                                    # smooth faces for better rendering
visualizebeamA.SetWireframe(False)                                     # non-wireframe mode
mesh.AddVisualShapeFEA(visualizebeamA)                                 # register shape with mesh

# Add visualization for node positions — dots at each node
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # display nodes as dots
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no additional FEM data
visualizebeamB.SetSymbolsThickness(0.006)                              # symbol thickness
visualizebeamB.SetSymbolsScale(0.01)                                   # symbol scale
visualizebeamB.SetZbufferHide(False)                                   # symbols not hidden by z-buffer
mesh.AddVisualShapeFEA(visualizebeamB)                                 # register shape with mesh

# Create Irrlicht visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)                                                  # attach Irrlicht to Chrono system
vis.SetWindowSize(1024, 768)                                           # window size in pixels
vis.SetWindowTitle('FEA cables')                                       # window title
vis.Initialize()                                                       # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))       # logo after Initialize
vis.AddSkyBox()                                                        # skybox after Initialize
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))                          # camera position after Initialize
vis.AddTypicalLights()                                                 # lights after Initialize

# Set solver type — MINRES for ANCF cable
solver = chrono.ChSolverMINRES()                                       # MINRES solver for stiff cables
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)                                       # max iterations
    solver.SetTolerance(1e-10)                                         # solver tolerance
    solver.EnableDiagonalPreconditioner(True)                          # diagonal preconditioner
    solver.EnableWarmStart(True)                                       # warm start for convergence
    solver.SetVerbose(False)                                           # disable verbose output

# Set timestepper for ANCF cable simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)                  # Euler implicit linearized
sys.SetTimestepper(ts)

time_step = 0.01                                                       # timestep for ANCF cable
sim_end = 10.0                                                         # simulation end time
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # render cadence (untagged)
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                                   # begin scene rendering
    vis.Render()                                                       # render the scene
    vis.EndScene()                                                     # end scene rendering
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                  # advance simulation
        if sys.GetChTime() >= sim_end:
            break
