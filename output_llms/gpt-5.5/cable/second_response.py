import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of 10 ANCF cable elements, with one end hinged to
# ground, moving under gravity and an applied nodal force.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh):
        # Create a section: define diameter and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)            # Cable diameter: 15 mm
        msection_cable2.SetYoungModulus(0.01e9)       # Young's modulus: 0.01 GPa
        msection_cable2.SetRayleighDamping(0.0001)    # Rayleigh damping

        # Create ANCF beam builder
        builder = fea.ChBuilderCableANCF()

        # Build beam with 10 ANCF cable elements
        builder.BuildBeam(
            mesh,
            msection_cable2,
            10,
            chrono.ChVector3d(0, 0, -0.1),
            chrono.ChVector3d(0.5, 0, -0.1)
        )

        # Retrieve beam end nodes
        beam_nodes = builder.GetLastBeamNodes()
        front_node = beam_nodes[0]
        back_node = beam_nodes[len(beam_nodes) - 1]

        # Apply force to the front node
        front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

        # Create a fixed truss body to act as ground
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        mtruss.SetPos(back_node.GetPos())
        system.Add(mtruss)

        # Create and initialize a node-frame constraint at the beam end
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(back_node, mtruss)
        system.Add(constraint_hinge)


# ----------------------------------------------------------------------------
# Initialize physical system and mesh
# ----------------------------------------------------------------------------

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

# Create the model
model = Model1(sys, mesh)

# ----------------------------------------------------------------------------
# FEA visualization
# ----------------------------------------------------------------------------

# Visualize bending moment Mz in beam elements
visualizebeamA = fea.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Visualize node positions
visualizebeamB = fea.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# ----------------------------------------------------------------------------
# Irrlicht visualization
# ----------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA cables")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# ----------------------------------------------------------------------------
# Solver settings: MINRES
# ----------------------------------------------------------------------------

solver = chrono.ChSolverMINRES()

if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")

solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

sys.SetSolver(solver)

# ----------------------------------------------------------------------------
# Timestepper
# ----------------------------------------------------------------------------

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# ----------------------------------------------------------------------------
# Simulation loop
# ----------------------------------------------------------------------------

time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)