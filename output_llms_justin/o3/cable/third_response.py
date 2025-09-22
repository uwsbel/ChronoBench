import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# -----------------------------------------------------------------------------
# Model1
#   – builds several independent chains of ANCF cable elements
#   – every chain is hinged to its own fixed truss at one end
#   – the free end of every chain is connected to a small rigid box
#   – a helper routine prints the position of every end-box during the run
# -----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh, n_chains: int = 6):
        self.system = system
        self.mesh   = mesh
        self.n_chains = n_chains
        self.end_bodies = []            # will store the rigid boxes (one per chain)

        # ---------------------------------------------------------------------
        # Cable section (common to all chains)
        # ---------------------------------------------------------------------
        cable_section = fea.ChBeamSectionCable()
        cable_section.SetDiameter(0.015)        # 15 mm Ø
        cable_section.SetYoungModulus(0.01e9)   # 0.01 GPa
        cable_section.SetRayleighDamping(0.0001)

        # ---------------------------------------------------------------------
        # Build the requested number of chains
        # ---------------------------------------------------------------------
        for i in range(self.n_chains):

            # ---- fixed reference frame (truss) for this chain ---------------
            truss = chrono.ChBody()
            truss.SetFixed(True)
            truss.SetPos(chrono.ChVector3d(0, 0.20 * i, 0))      # shift along Y
            self.system.Add(truss)

            # ---- geometry & element count for this chain --------------------
            nel   = 6 + 2 * i                                    # grow with i
            Apos  = chrono.ChVector3d(0,           0.20 * i, 0)
            Bpos  = chrono.ChVector3d(0.50 + 0.05 * i,
                                      0.20 * i,
                                      0)

            # ---- build chain of cable elements ------------------------------
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(self.mesh,                 # where to store new items
                              cable_section,             # section definition
                              nel,                       # number of elements
                              Apos,                      # start
                              Bpos)                      # end

            # The helper returns the first and the last node of the chain:
            last_nodes = builder.GetLastBeamNodes()
            first_node = last_nodes[0]
            last_node  = last_nodes[1]

            # ---- force on first node ----------------------------------------
            first_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            # ---- hinge constraint (first node ↔ truss) ----------------------
            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(first_node, truss)
            self.system.Add(hinge)

            # ---- rigid box connected to free end ----------------------------
            box_size = 0.04
            box = chrono.ChBodyEasyBox(box_size, box_size, box_size,
                                       1000,         # density
                                       True,         # visual
                                       True)         # collide
            # place the box slightly after the last node
            box.SetPos(chrono.ChVector3d(Bpos.x + box_size * 0.5,
                                         Bpos.y,
                                         Bpos.z))
            self.system.Add(box)

            # point-to-frame link between the last node and the box
            pt2frame = fea.ChLinkPointFrame()
            pt2frame.Initialize(last_node, box)
            self.system.Add(pt2frame)

            # remember this body for run-time reporting
            self.end_bodies.append(box)

    # -------------------------------------------------------------------------
    # Helper: print position of every end-box
    # -------------------------------------------------------------------------
    def PrintBodyPositions(self):
        print("---- End-box positions ----")
        for k, body in enumerate(self.end_bodies):
            p = body.GetPos()
            print(f"  Chain {k}: (x={p.x: .3f}, y={p.y: .3f}, z={p.z: .3f})")
        print("----------------------------------------------------------------")
# =============================================================================
# Main driver
# =============================================================================
# ---------- physical system ---------------------------------------------------
sys  = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# ---------- build model -------------------------------------------------------
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)                     # very important: add mesh to the system!

# ---------- FEA visualisation assets -----------------------------------------
vis_beam_moment = chrono.ChVisualShapeFEA(mesh)
vis_beam_moment.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_moment.SetColorscaleMinMax(-0.4, 0.4)
vis_beam_moment.SetSmoothFaces(True)
vis_beam_moment.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_moment)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)

# ---------- Irrlicht visualisation system ------------------------------------
vsys = chronoirr.ChVisualSystemIrrlicht()
vsys.AttachSystem(sys)
vsys.SetWindowSize(1024, 768)
vsys.SetWindowTitle("Multiple FEA cable chains")
vsys.Initialize()
vsys.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vsys.AddSkyBox()
vsys.AddCamera(chrono.ChVector3d(0.7, 0.4, 1.2))
vsys.AddTypicalLights()

# ---------- solver & time integrator -----------------------------------------
solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

sys.SetTimestepper(chrono.ChTimestepperEulerImplicitLinearized(sys))

# ---------- simulation loop ---------------------------------------------------
step_size = 0.01
frame = 0
while vsys.Run():
    vsys.BeginScene()
    vsys.Render()
    vsys.EndScene()

    sys.DoStepDynamics(step_size)

    # print body locations every 20 frames
    if frame % 20 == 0:
        model.PrintBodyPositions()
    frame += 1