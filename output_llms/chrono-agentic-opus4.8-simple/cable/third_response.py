import os
import pychrono as chrono                                            # core
import pychrono.fea as fea                                            # finite-element module
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer


# ----------------------------------------------------------------------------
# Model1: multiple chains of ANCF cable beam elements with connected bodies.
#         Each chain is a fixed truss reference, a first cable beam, an end box,
#         a second cable beam, and a final end box. The number of cable elements
#         grows with each chain. n_chains controls how many chains are built.
# ----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.bodies = []                                             # end body of each chain (printed later)

        msection_cable = fea.ChBeamSectionCable()                    # shared cable section
        msection_cable.SetDiameter(0.015)                            # cable diameter (m)
        msection_cable.SetYoungModulus(0.01e9)                       # soft cable, E = 0.01 GPa
        msection_cable.SetRayleighDamping(0.000)                     # no extra structural damping

        for j in range(n_chains):                                    # build one chain per index
            mtruss = chrono.ChBody()                                 # fixed reference frame for this chain
            mtruss.SetFixed(True)                                    # truss is the ground anchor
            system.Add(mtruss)                                       # register the truss

            builder = fea.ChBuilderCableANCF()                       # ANCF cable beam builder

            builder.BuildBeam(mesh,                                  # mesh to receive nodes/elements
                              msection_cable,                        # the cable section
                              1 + j,                                 # more elements with each chain
                              chrono.ChVector3d(0, 0, -0.1 * j),            # A (start of first beam)
                              chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j))  # B (end of first beam)

            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.2, 0))  # tip load (N)

            constraint_hinge = fea.ChLinkNodeFrame()                 # hinge first node to the truss
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            msphere = chrono.ChVisualShapeSphere(0.02)               # mark the hinge with a sphere
            constraint_hinge.AddVisualShape(msphere)

            mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)       # intermediate box body
            mbox.SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(mbox)

            constraint_pos = fea.ChLinkNodeFrame()                   # pin first-beam end to the box
            constraint_pos.Initialize(builder.GetLastBeamNodes().back(), mbox)
            system.Add(constraint_pos)

            constraint_dir = fea.ChLinkNodeSlopeFrame()              # clamp the slope (direction) too
            constraint_dir.Initialize(builder.GetLastBeamNodes().back(), mbox)
            constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir)

            builder.BuildBeam(mesh,                                  # second beam in this chain
                              msection_cable,
                              1 + (n_chains - j),                    # element count decreases as chains grow
                              chrono.ChVector3d(mbox.GetPos().x + 0.1, 0, -0.1 * j),                          # A
                              chrono.ChVector3d(mbox.GetPos().x + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j))   # B

            constraint_pos2 = fea.ChLinkNodeFrame()                  # pin second-beam start to the box
            constraint_pos2.Initialize(builder.GetLastBeamNodes().front(), mbox)
            system.Add(constraint_pos2)

            constraint_dir2 = fea.ChLinkNodeSlopeFrame()             # clamp its slope
            constraint_dir2.Initialize(builder.GetLastBeamNodes().front(), mbox)
            constraint_dir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir2)

            self.bodies.append(chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000))  # final end box of this chain
            self.bodies[j].SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(self.bodies[j])

            constraint_pos3 = fea.ChLinkNodeFrame()                  # pin second-beam end to the end box
            constraint_pos3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
            system.Add(constraint_pos3)

            constraint_dir3 = fea.ChLinkNodeSlopeFrame()             # clamp the slope at the end box
            constraint_dir3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
            constraint_dir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir3)

    # Print the position of the end body of each chain at the current time.
    def PrintBodyPositions(self, system):
        print("Time: " + str(system.GetChTime()))                   # current simulation time
        for body in self.bodies:                                    # one line per chain end body
            p = body.GetPos()                                       # end body position
            print("  ", p.x, "  ", p.y, "  ", p.z)


solver = chrono.ChSolverSparseQR()                                   # sparse QR direct solver for ANCF

sys = chrono.ChSystemSMC()                                           # FEA system uses SMC

mesh = fea.ChMesh()                                                  # container for cable nodes/elements

model = Model1(sys, mesh, 6)                                         # build 6 chains of cable beams

sys.Add(mesh)                                                        # register the mesh with the system

visualizebeamA = chrono.ChVisualShapeFEA(mesh)                       # surface shape: bending moment Mz
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)                        # colour scale (two scalars)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamB = chrono.ChVisualShapeFEA(mesh)                       # glyph shape: node dots
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()                                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))                         # eye looking at the chains
vis.AddTypicalLights()

sys.SetSolver(solver)                                                # use the sparse QR solver
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)               # linearized implicit integrator
sys.SetTimestepper(ts)

time_step = 0.01                                                     # ANCF cable step size
sim_end = 5.0                                                        # total simulated time (s)
render_fps = 50.0                                                    # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                               # advance one ANCF cable step
        model.PrintBodyPositions(sys)                               # report each chain's end body
        if sys.GetChTime() >= sim_end:
            break
