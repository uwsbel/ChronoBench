import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


class Model1:
    def __init__(self, system, mesh):
        self.n_chains = 6  # Number of chains
        self.beam_length = 0.5
        self.beam_diameter = 0.015
        self.young_modulus = 0.01e9
        self.rayleigh_damping = 0.0001
        self.gravity = chrono.ChVectorD(0, -0.7, 0)
        self.step = 0.01

        self.beam_forces = []
        self.box_bodies = []

        self.truss_body = chrono.ChBody()
        self.truss_body.SetFixed(True)

        for i in range(self.n_chains):
            self.create_chain(i, system, mesh)

    def create_chain(self, index, system, mesh):
        offset = index * 0.1

        # Create and initialize a truss body as a fixed reference frame
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)

        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()

        # Use BuildBeam to create a beam structure consisting of ANCF elements
        start_point = chrono.ChVectorD(offset, 0, -0.1)
        end_point = chrono.ChVectorD(offset + self.beam_length, 0, -0.1)
        builder.BuildBeam(
            mesh,
            self.get_beam_section(),
            10 + index,
            start_point,
            end_point,
        )

        # Apply boundary conditions and loads including hinge constraints and forces
        front_node = builder.GetLastBeamNodes().front()
        front_node.SetForce(self.gravity)

        # Create and initialize a hinge constraint to fix beam's end point to the truss
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
        system.Add(constraint_hinge)

        # Connect each beam's endpoint to a chrono body (box) and establish further constraints between the beam endpoint and the box
        box_body = self.create_box(system, mesh)
        constraint_weld = fea.ChLinkWeld()
        constraint_weld.Initialize(builder.GetLastBeamNodes().back(), box_body)
        system.Add(constraint_weld)

        self.box_bodies.append(box_body)

    def create_box(self, system, mesh):
        box_shape = chrono.ChBox(0.05, 0.05, 0.05)
        box_body = chrono.ChBody()
        box_body.AddShape(box_shape)
        box_body.SetPos(self.beam_length * (self.n_chains - 1) / 2 * (1 + self.beam_diameter / 2) + self.beam_length / 2, 0, -0.1)
        box_body.SetMass(1)
        system.Add(box_body)
        mesh.AddBody(box_body)
        return box_body

    def get_beam_section(self):
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(self.beam_diameter)
        msection_cable2.SetYoungModulus(self.young_modulus)
        msection_cable2.SetRayleighDamping(self.rayleigh_damping)
        return msection_cable2

    def PrintBodyPositions(self, system):
        for i, body in enumerate(self.box_bodies):
            print(f"Chain {i + 1} end body position: {body.GetPos()}")


# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)

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

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
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

# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(model.step)
    model.PrintBodyPositions(sys)