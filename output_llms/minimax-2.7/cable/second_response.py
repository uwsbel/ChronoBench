import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# Model 1 – ANCF cable beam with one end hinged to ground, gravity only
# Demonstrates the use of ChBuilderCableANCF
# -----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh):
        # -----------------------------------------------------------------
        # Cable section (beam section) – define geometry & material
        # -----------------------------------------------------------------
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)                # 15 mm diameter
        msection_cable2.SetYoungModulus(0.01e9)           # 0.01 GPa
        msection_cable2.SetRayleighDamping(0.0001)       # <-- Modified: 0.000 → 0.0001

        # -----------------------------------------------------------------
        # Build the ANCF cable beam using the helper builder
        # -----------------------------------------------------------------
        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(
            mesh,                                      # mesh that will hold the elements
            msection_cable2,                           # section properties
            10,                                        # number of elements
            chrono.ChVector3d(0, 0, -0.1),              # start point A
            chrono.ChVector3d(0.5, 0, -0.1)            # end point B
        )

        # -----------------------------------------------------------------
        # Apply a point force to the front (free) node of the beam
        # -----------------------------------------------------------------
        builder.GetLastBeamNodes().front().SetForce(
            chrono.ChVector3d(0, -0.7, 0)              # <-- Modified: -0.2 → -0.7
        )

        # -----------------------------------------------------------------
        # Create a fixed truss (ground) body and add it to the system
        # -----------------------------------------------------------------
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)                             # <-- Fixed: truss must be in the system

        # -----------------------------------------------------------------
        # Hinge the last (fixed) node of the beam to the truss
        # -----------------------------------------------------------------
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
        system.Add(constraint_hinge)


# =============================================================================
# Create the physical system and the FEA mesh
# =============================================================================
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Build the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)

# =============================================================================
# Visualisation of the beam (moment MZ) and of the nodes
# =============================================================================
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

# =============================================================================
# Irrlicht visualisation setup
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# =============================================================================
# Solver configuration – MINRES
# =============================================================================
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
sys.SetSolver(solver)

solver.SetMaxIterations(200)           # maximum number of iterations
solver.SetTolerance(1e-10)             # solver tolerance
solver.EnableDiagonalPreconditioner(True)   # enable diagonal preconditioner
solver.EnableWarmStart(True)           # enable warm start
solver.SetVerbose(False)              # disable verbose output

# =============================================================================
# Timestepper (Euler Implicit Linearized)
# =============================================================================
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# =============================================================================
# Main simulation loop
# =============================================================================
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)            # advance by 10 ms per frame