import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: multiple chains of ANCF cable beams. Each chain's first node is
# hinged to ground, the last node is connected to a small box body.
# ----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.end_bodies = []

        # Material & cross section (reused for all chains)
        base_section = fea.ChBeamSectionCable()
        base_section.SetDiameter(0.015)         # 15 mm
        base_section.SetYoungModulus(0.01e9)    # 0.01 GPa
        base_section.SetRayleighDamping(0.0001)

        for i in range(n_chains):
            # create a builder for this chain
            builder = fea.ChBuilderCableANCF()

            # number of elements grows with chain index
            num_elems = 10 + i

            # place each chain parallel along the y–axis to avoid overlap
            y_offset = i * 0.1
            start_pt = chrono.ChVector3d(0, y_offset, 0)
            end_pt   = chrono.ChVector3d(0.5, y_offset, 0)

            # build the beam chain
            builder.BuildBeam(
                mesh,
                base_section,
                num_elems,
                start_pt,
                end_pt
            )

            # retrieve the two end nodes
            nodes = builder.GetLastBeamNodes()
            node_start = nodes[0]
            node_end   = nodes[-1]

            # 1) FIXED TRUSS + HINGE at the start
            mtruss = chrono.ChBody()
            mtruss.SetPos(start_pt)       # locate it at the start node
            mtruss.SetBodyFixed(True)
            system.Add(mtruss)

            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(node_start, mtruss)
            system.Add(hinge)

            # 2) APPLY A DOWNWARD PULLING FORCE at the start node
            node_start.SetForce(chrono.ChVector3d(0, -5.0, 0))

            # 3) A SMALL BOX AT THE END, CONNECTED TO THE END NODE
            box = chrono.ChBody()
            box.SetPos(end_pt)
            box.SetMass(0.2)
            # Simple diagonal inertia
            box.SetInertiaXX(chrono.ChVectorD(0.001, 0.001, 0.001))
            system.Add(box)

            # optional: add a little sphere for visualization on the box
            sphere_asset = chrono.ChSphereShape(0.025)
            box.AddVisualShape(sphere_asset)

            link_end = fea.ChLinkNodeFrame()
            link_end.Initialize(node_end, box)
            system.Add(link_end)

            # remember for later printing
            self.end_bodies.append(box)

        # finally add the mesh container (all beams) to the system
        system.Add(mesh)

    def PrintBodyPositions(self):
        # print the world position of each end box
        for idx, b in enumerate(self.end_bodies):
            p = b.GetPos()
            print(f"Chain {idx} end‐box pos = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")


# ------------------------------------------------------------
# MAIN SCRIPT
# ------------------------------------------------------------
if __name__ == "__main__":

    # 1) create the Chrono SMC system and set gravity
    sys = chrono.ChSystemSMC()
    sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

    # 2) create the FEA mesh
    mesh = fea.ChMesh()

    # 3) build the model: 6 chains by default
    model = Model1(sys, mesh, n_chains=6)

    # 4) FEA visualization (beam moments & node dots)
    visA = chrono.ChVisualShapeFEA(mesh)
    visA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    visA.SetColorscaleMinMax(-0.4, 0.4)
    visA.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(visA)

    visB = chrono.ChVisualShapeFEA(mesh)
    visB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    visB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    visB.SetSymbolsThickness(0.006)
    visB.SetSymbolsScale(0.01)
    mesh.AddVisualShapeFEA(visB)

    # 5) Irrlicht application for real-time rendering
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Multiple ANCF Cable Chains')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1, 1, 1))
    vis.AddTypicalLights()

    # 6) choose and configure solver
    solver = chrono.ChSolverMINRES()
    sys.SetSolver(solver)
    if solver.GetType() == chrono.ChSolver.Type_MINRES:
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableDiagonalPreconditioner(True)
        solver.EnableWarmStart(True)
        solver.SetVerbose(False)

    # 7) use Euler‐implicit linearized integrator
    ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
    sys.SetTimestepper(ts)

    # 8) simulation loop
    time_step = 0.01
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        sys.DoStepDynamics(time_step)
        # print end‐body positions each step
        model.PrintBodyPositions()