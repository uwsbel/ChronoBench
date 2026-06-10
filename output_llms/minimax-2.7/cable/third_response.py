# =============================================================================
# Modified PyChrono example – multiple ANCF cable chains with end bodies
# =============================================================================
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# --------------------------------------------------------------------
# Model1 – builds a number of independent ANCF cable chains.
# Each chain:
#   • starts at a truss (fixed body)
#   • contains a user‑defined number of ANCF cable elements
#   • ends on a box body that is linked to the last beam node
# --------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh, n_chains=6):
        """Create *n_chains* ANCF cable chains."""
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []                     # will hold the box at each chain end

        for i in range(n_chains):
            # ----- beam section (cable) ---------------------------------
            msection = fea.ChBeamSectionCable()
            msection.SetDiameter(0.015)                # 15 mm diameter
            msection.SetYoungModulus(0.01e9)            # 0.01 GPa
            msection.SetRayleighDamping(0.0001)

            # ----- number of elements grows with the chain index -----
            num_elements = 10 + i                       # chain 0 → 10 el., chain 1 → 11 el., …

            # ----- spatial offset to avoid overlapping beams ----------
            offset_x = i * 0.8
            pointA = chrono.ChVector3d(offset_x, 0.0, -0.1)
            pointB = chrono.ChVector3d(offset_x + 0.5, 0.0, -0.1)

            # ----- build the ANCF cable beam ---------------------------
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, msection, num_elements, pointA, pointB)

            # Nodes of the newly created beam
            beam_nodes = builder.GetLastBeamNodes()
            # The first node is the “front” (attached to the truss)
            # The last node is the “back” (attached to the box)
            node_front = beam_nodes[0]
            node_back = beam_nodes[-1]

            # ----- fixed truss body (reference frame) ------------------
            truss = chrono.ChBody()
            truss.SetFixed(True)
            system.Add(truss)

            # ----- hinge (Node‑Frame) constraint at the front node ----
            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(node_front, truss)
            system.Add(hinge)

            # ----- apply a constant force to the front node ------------
            node_front.SetForce(chrono.ChVector3d(0.0, -0.7, 0.0))

            # ----- box body at the back end ----------------------------
            box = chrono.ChBody()
            box.SetMass(1.0)
            box.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            box.SetPos(node_back.GetPos())               # start where the node is
            system.Add(box)

            # ----- Node‑Frame constraint to lock the beam end to the box
            link_node_box = fea.ChLinkNodeFrame()
            link_node_box.Initialize(node_back, box)
            system.Add(link_node_box)

            # ----- simple visual for the box (cube 5 cm side) ----------
            box_vis = chrono.ChBoxShape(0.05, 0.05, 0.05)
            box.AddVisualShape(box_vis)

            # ----- keep the end body for later printing ----------------
            self.end_bodies.append(box)

    # -----------------------------------------------------------------
    # Print the current positions of the end bodies (the boxes)
    # -----------------------------------------------------------------
    def PrintBodyPositions(self):
        for idx, body in enumerate(self.end_bodies):
            p = body.GetPos()
            print(f"Chain {idx:2d} – end‑body pos: ({p.x:8.4f}, {p.y:8.4f}, {p.z:8.4f})")


# =============================================================================
# Main simulation
# =============================================================================
# Create the physical system and the FEM mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Build the model (6 chains by default)
model = Model1(sys, mesh, n_chains=6)

# Add the mesh to the system (mandatory)
sys.Add(mesh)

# ---------------------------------------------------------------
# Visualisation – FEM beam (moments) and node symbols
# ---------------------------------------------------------------
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.4, 0.4)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_node = chrono.ChVisualShapeFEA(mesh)
vis_node.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_node.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_node.SetSymbolsThickness(0.006)
vis_node.SetSymbolsScale(0.01)
vis_node.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_node)

# ---------------------------------------------------------------
# Irrlicht visualisation window
# ---------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple ANCF cable chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# ---------------------------------------------------------------
# Solver configuration (MINRES with warm start)
# ---------------------------------------------------------------
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

# ---------------------------------------------------------------
# Timestepper – Euler implicit linearized (compatible with MINRES)
# ---------------------------------------------------------------
timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(timestepper)

# ---------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance dynamics
    sys.DoStepDynamics(0.01)

    # Print end‑body positions (every step)
    model.PrintBodyPositions()