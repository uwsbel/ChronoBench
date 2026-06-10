"""Multiple ANCF gradient-deficient cable chains with connected rigid bodies.

Model: an FEA scene (ChSystemSMC, Y-up gravity) in which a Model1 container builds
`n_chains` independent chains of ANCF cable beam elements. Each chain hangs from a
fixed truss via a hinge (ChLinkNodeFrame), has a downward force applied at its free
node, and is welded through position + slope-frame constraints to a rigid box body;
a second cable segment then links that box to a final end box. The number of cable
elements grows with each chain, so successive chains are longer and droop more under
the applied force and gravity. A PrintBodyPositions method reports the pose of every
chain's end body each step. ANCF cables use the sparse-QR solver with the linearized
implicit Euler timestepper. No contact/collision (pure FEA + node-body constraints),
so no collision system is configured.
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Model: multiple ANCF cable chains with connected bodies ===
# Builds n_chains beam chains; keeps strong refs to builders/bodies (SWIG GC safety)
# and stores each chain's end body for PrintBodyPositions.
class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.bodies = []      # end body of each chain (for PrintBodyPositions)
        self._keep = []       # strong refs: builders, sections, constraints

        # Shared cable section: thickness + material properties for all beams.
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.000)
        self._keep.append(msection_cable)

        # Single fixed truss used as the common reference frame for every hinge.
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)
        self._keep.append(mtruss)

        for j in range(n_chains):
            builder = fea.ChBuilderCableANCF()
            self._keep.append(builder)

            # First beam of this chain: element count increases with each chain,
            # placed at a distinct Z offset so chains do not overlap.
            builder.BuildBeam(mesh, msection_cable,
                              1 + j,                                     # n elements grows per chain
                              chrono.ChVector3d(0, 0, -0.1 * j),
                              chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j))
            front_nodes = builder.GetLastBeamNodes()                    # cache: container kept (SWIG GC)
            node_A = front_nodes.front()
            node_B = front_nodes.back()

            # Apply a downward force at the free end node.
            node_B.SetForce(chrono.ChVector3d(0, -0.2, 0))

            # Hinge the start node to the fixed truss (position only).
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(node_A, mtruss)
            system.Add(constraint_hinge)
            constraint_hinge.AddVisualShape(chrono.ChVisualShapeSphere(0.02))
            self._keep.append(constraint_hinge)

            # Connect the free end to an intermediate box (position + slope frame).
            mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)
            mbox.SetPos(node_B.GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(mbox)
            self._keep.append(mbox)

            cpos = fea.ChLinkNodeFrame()
            cpos.Initialize(node_B, mbox)
            system.Add(cpos)
            cdir = fea.ChLinkNodeSlopeFrame()
            cdir.Initialize(node_B, mbox)
            cdir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(cdir)
            self._keep += [cpos, cdir]

            # Second beam: links the intermediate box to the final end box.
            builder.BuildBeam(mesh, msection_cable,
                              1 + (n_chains - j),
                              chrono.ChVector3d(mbox.GetPos().x + 0.1, 0, -0.1 * j),
                              chrono.ChVector3d(mbox.GetPos().x + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j))
            seg2_nodes = builder.GetLastBeamNodes()                     # cache: container kept (SWIG GC)
            seg2_A = seg2_nodes.front()
            seg2_B = seg2_nodes.back()

            cpos2 = fea.ChLinkNodeFrame()
            cpos2.Initialize(seg2_A, mbox)
            system.Add(cpos2)
            cdir2 = fea.ChLinkNodeSlopeFrame()
            cdir2.Initialize(seg2_A, mbox)
            cdir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(cdir2)
            self._keep += [cpos2, cdir2]

            # Final end box of the chain, recorded for PrintBodyPositions.
            end_box = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)
            end_box.SetPos(seg2_B.GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(end_box)
            self.bodies.append(end_box)

            cpos3 = fea.ChLinkNodeFrame()
            cpos3.Initialize(seg2_B, end_box)
            system.Add(cpos3)
            cdir3 = fea.ChLinkNodeSlopeFrame()
            cdir3.Initialize(seg2_B, end_box)
            cdir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(cdir3)
            self._keep += [cpos3, cdir3]

    # Print the position of the end body of every chain at the current time.
    def PrintBodyPositions(self, time):
        print("Time: " + str(time))
        for k, body in enumerate(self.bodies):
            p = body.GetPos()
            print("  chain %d end body: (%.5f, %.5f, %.5f)" % (k, p.x, p.y, p.z))


# === System & gravity === FEA cable scene: SMC system, Y-up gravity
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Mesh & model === container of cable elements + their referenced nodes
mesh = fea.ChMesh()
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# === Solver & timestepper === ANCF cable: sparse-QR + linearized implicit Euler
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA visualization === beam-moment surface field + node-position glyphs
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA ANCF cable chains")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1), chrono.ChVector3d(0.5, -0.3, -0.3))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render + step; print end-body positions each step
time_step = 0.01
sim_end = 5.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

os.makedirs("cam", exist_ok=True)   # guard against missing output dir
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            model.PrintBodyPositions(sys.GetChTime())
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    print("Simulation finished at t = %.3f s" % sys.GetChTime())
