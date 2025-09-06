import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of ANCF cable elements with connected bodies
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.system = system
        self.mesh = mesh
        self.bodies = []  # To store the end bodies for position tracking

        # Create chains
        for i in range(n_chains):
            self.create_chain(i)

    def create_chain(self, chain_index):
        # Create a section for the cable beam
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  # 15 mm diameter
        msection_cable.SetYoungModulus(0.01e9)  # 0.01 GPa
        msection_cable.SetRayleighDamping(0.0001)

        # Create a ChBuilderCableANCF helper object
        builder = fea.ChBuilderCableANCF()

        # Calculate positions for this chain (spread out in the y-direction)
        start_point = chrono.ChVector3d(0, chain_index * 0.2, -0.1)
        end_point = chrono.ChVector3d(0.5, chain_index * 0.2, -0.1)

        # Number of elements increases with chain index (minimum 5)
        n_elements = 5 + chain_index * 2

        # Create the beam
        builder.BuildBeam(
            self.mesh,  # The mesh to add to
            msection_cable,  # Beam section properties
            n_elements,  # Number of elements
            start_point,  # Starting point
            end_point  # Ending point
        )

        # Create a truss body (fixed reference frame)
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        self.system.Add(mtruss)

        # Create a box body at the end of the beam
        end_body = chrono.ChBody()
        end_body.SetPos(end_point)
        end_body.SetMass(0.1)
        end_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
        end_body.SetCollide(True)
        end_body.GetCollisionModel().AddBox(0.05, 0.05, 0.05)
        self.system.Add(end_body)
        self.bodies.append(end_body)

        # Connect beam end to the box body
        link_beam_body = fea.ChLinkNodeFrame()
        link_beam_body.Initialize(builder.GetLastBeamNodes().back(), end_body)
        self.system.Add(link_beam_body)

        # Connect beam start to the truss
        link_truss = fea.ChLinkNodeFrame()
        link_truss.Initialize(builder.GetLastBeamNodes().front(), mtruss)
        self.system.Add(link_truss)

        # Apply force to the end body
        end_body.SetForce(chrono.ChVector3d(0, -0.7, 0))

    def PrintBodyPositions(self):
        for i, body in enumerate(self.bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with 6 chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Add the mesh to the physical system

# Add visualization for the FEM mesh
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node positions
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Multiple Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, -2))
vis.AddTypicalLights()

# Set solver type and settings
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

# Set the timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()

    # Print body positions every 10 steps
    if sys.GetChTime() % 0.1 < 0.01:
        model.PrintBodyPositions()

    vis.EndScene()
    sys.DoStepDynamics(0.01)