import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of 10 ANCF cable elements, one end hinged to ground,
# moving under gravity alone.
# ----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh):
        # 1) Create a cable‐beam section and set material properties
        msection = fea.ChBeamSectionCable()
        msection.SetDiameter(0.015)           # 15 mm
        msection.SetYoungModulus(0.01e9)      # 0.01 GPa
        msection.SetRayleighDamping(0.0001)   # UPDATED: changed to 0.0001

        # 2) Use the ANCF‐cable builder
        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(
            mesh,
            msection,
            10,
            chrono.ChVector3d(0, 0, -0.1),    # start
            chrono.ChVector3d(0.5, 0, -0.1)   # end
        )

        # 3) Apply a vertical load at the front (first) node
        beam_nodes = builder.GetLastBeamNodes()
        if len(beam_nodes) > 0:
            beam_nodes[0].SetForce(chrono.ChVector3d(0, -0.7, 0))  # UPDATED: changed to -0.7 N

        # 4) Create a fixed truss body to serve as ground reference
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        system.Add(mtruss)

        # 5) Hinge the last node to the truss
        constraint = fea.ChLinkNodeFrame()
        constraint.Initialize(beam_nodes[-1], mtruss)
        system.Add(constraint)


# ----------------------------------------------------------------------------
# Main script
# ----------------------------------------------------------------------------
# 1) Set up system and FEM mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
model = Model1(sys, mesh)
sys.Add(mesh)

# 2) Add FEM visualization for bending moment
vis_mz = chrono.ChVisualShapeFEA(mesh)
vis_mz.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_mz.SetColorscaleMinMax(-0.4, 0.4)
vis_mz.SetSmoothFaces(True)
vis_mz.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_mz)

# 3) Add FEM visualization for node positions
vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)

# 4) Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# 5) Solver setup: switch to MINRES and apply requested settings
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
solver.SetMaxIterations(200)                     # as requested
solver.SetTolerance(1e-10)                       # as requested
solver.EnableDiagonalPreconditioner(True)        # as requested
solver.EnableWarmStart(True)                     # as requested
solver.SetVerbose(False)                         # ensure no verbose output
sys.SetSolver(solver)

# 6) Use an implicit linearized Euler timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# 7) Run the simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)