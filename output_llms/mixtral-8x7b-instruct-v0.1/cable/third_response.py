import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh):
        self.n_chains = 6  # Number of chains
        self.beam_elements = []
        self.truss_bodies = []
        self.box_bodies = []
        self.hinge_constraints = []
        self.box_constraints = []

        self.beam_spacing = 0.15  # Spacing between chains
        self.beam_length = 1.0  # Length of each chain
        self.beam_elements_per_chain = 10  # Number of ANCF cable elements per chain
        self.beam_diameter = 0.015  # Diameter of the cable section
        self.beam_young_modulus = 0.01e9  # Young's modulus of the cable section
        self.beam_rayleigh_damping = 0.0001  # Rayleigh damping for the cable section

        self.box_size = 0.1  # Size of the box
        self.box_young_modulus = 1e9  # Young's modulus of the box
        self.box_density = 1000  # Density of the box

        self.gravity = chrono.ChVectorD(0, -0.7, 0)  # Gravity

        self.step = 0

        for i in range(self.n_chains):
            self.create_chain(i, system, mesh)

    def create_chain(self, chain_index, system, mesh):
        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(self.beam_diameter)  # Set the diameter of the cable section
        msection_cable2.SetYoungModulus(self.beam_young_modulus)  # Set the Young's modulus of the cable section
        msection_cable2.SetRayleighDamping(self.beam_rayleigh_damping)  # Set Rayleigh damping to zero for this section

        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()

        # Set starting and ending points for the chain
        start_point = chrono.ChVectorD(0, 0, -0.1 + chain_index * self.beam_spacing)
        end_point = start_point.Copy()
        end_point.x += self.beam_length

        # Use BuildBeam to create a beam structure consisting of ANCF elements:
        builder.BuildBeam(
            mesh,  # The mesh to which the created nodes and elements will be added
            msection_cable2,  # The beam section properties to use
            self.beam_elements_per_chain,  # Number of ANCF elements to create along the beam
            start_point,  # Starting point ('A' point) of the beam
            end_point  # Ending point ('B' point) of the beam
        )

        # Retrieve the end nodes of the beam
        last_nodes = builder.GetLastBeamNodes()

        # Apply forces to the front node
        last_nodes.front().SetForce(self.gravity)

        # Create a truss body (a fixed reference frame in the simulation)
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)  # Fix the truss body

        # Create and initialize a hinge constraint to fix beam's end point to the truss
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(last_nodes.back(), mtruss)
        system.Add(constraint_hinge)  # Add the constraint to the system
        self.hinge_constraints.append(constraint_hinge)

        # Create a box body
        mbox = chrono.ChBody()
        mbox.SetBox(chrono.ChVectorD(self.box_size / 2), self.box_size, self.box_size, self.box_size, True)
        mbox.SetMass(self.box_size ** 3 * self.box_density)
        mbox.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
        mbox.SetPos(end_point)

        # Add a visualization object for the box
        visualizebox = chrono.ChVisualBox(self.box_size, self.box_size, self.box_size, chrono.ChColor(1, 0, 0))
        mbox.AddVisualShape(visualizebox)

        # Add the box body to the system
        system.Add(mbox)
        self.box_bodies.append(mbox)

        # Create and initialize a constraint to fix the beam's end point to the box
        constraint_box = fea.ChLinkNodeFrame()
        constraint_box.Initialize(last_nodes.back(), mbox)
        system.Add(constraint_box)  # Add the constraint to the system
        self.box_constraints.append(constraint_box)

    def PrintBodyPositions(self, system):
        for i in range(self.n_chains):
            body = self.box_bodies[i]
            pos = body.GetPos()
            print(f"Chain {i + 1} end body position: {pos}")

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

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

# Add visualization for box bodies
visualizebox = chrono.ChVisualBox(model.box_size, model.box_size, model.box_size, chrono.ChColor(1, 0, 0))
for box in model.box_bodies:
    box.AddVisualShape(visualizebox)

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
    sys.DoStepDynamics(0.01)
    if model.step % 10 == 0:
        model.PrintBodyPositions(sys)
    model.step += 1