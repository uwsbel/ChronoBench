import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Modified Model1: Multiple chains of ANCF cable elements with connected bodies
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.end_bodies = []
        
        for i in range(n_chains):
            # Chain-specific parameters
            z_offset = -0.1 + i * 0.2  # Offset along Z-axis to prevent overlap
            start_point = chrono.ChVector3d(0, 0, z_offset)
            end_point = chrono.ChVector3d(0.5, 0, z_offset)
            n_elements = 10 + i  # Increasing number of elements per chain

            # Create fixed truss for chain start
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(start_point)
            system.Add(mtruss)

            # Create beam section
            msection = fea.ChBeamSectionCable()
            msection.SetDiameter(0.015)
            msection.SetYoungModulus(0.01e9)
            msection.SetRayleighDamping(0.0001)

            # Build beam
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(mesh, msection, n_elements, start_point, end_point)

            # Apply force to last node (near end_point)
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Create end body (box) and constraint
            box_body = chrono.ChBody()
            box_body.SetPos(end_point)
            system.Add(box_body)
            
            # Add visualization for box
            box_shape = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            box_body.AddVisualShape(box_shape)

            # Connect last node to box body
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(builder.GetLastBeamNodes().back(), box_body)
            system.Add(constraint)

            self.end_bodies.append(box_body)

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i}: End body position = ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model and add components
model = Model1(sys, mesh)
sys.Add(mesh)

# Visualization setup for FEM
vis_fea = chrono.ChVisualShapeFEA(mesh)
vis_fea.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_fea.SetColorscaleMinMax(-0.4, 0.4)
vis_fea.SetSmoothFaces(True)
vis_fea.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_fea)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(vis_nodes)

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multi-Chain Cable FEM')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, 0.6, -1.2))
vis.AddTypicalLights()

# Solver settings
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()