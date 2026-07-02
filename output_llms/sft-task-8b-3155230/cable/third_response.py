import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A system of multiple chains of ANCF cable elements with connected bodies.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.boxes = []  # To store the box bodies for each chain
        # Create a section for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)
        # Loop over each chain
        for chain_idx in range(n_chains):
            # Create a truss body (fixed reference frame)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetName(f"Truss_{chain_idx}")
            # Create a box body
            box = chrono.ChBody()
            box.SetFixed(False)
            box.GetCollisionModel().Clear()
            box.GetCollisionModel().AddBoxGeometry(0.05, 0.05, 0.05)
            box.EnableCollision(False)
            box.SetName(f"Box_{chain_idx}")
            # Add the box to the system
            system.Add(box)
            self.boxes.append(box)
            # Create a new ChBuilderCableANCF for this chain
            builder = fea.ChBuilderCableANCF()
            # Determine the start and end points for this chain
            y_offset = chain_idx * 0.2
            start_point = chrono.ChVector3d(0, y_offset, -0.1)
            end_point = chrono.ChVector3d(0.5, y_offset, -0.1)
            # Number of elements increases with each chain
            num_elements = 10 + chain_idx * 5
            # Build the beam
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_point,
                end_point
            )
            # Apply force to the end node
            end_node = builder.GetLastBeamNodes().back()
            end_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
            # Create a hinge constraint between the start node and the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)
            # Create a constraint between the end node and the box
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(end_node, box)
            system.Add(constraint_box)
            # Add visualization for the box
            visualizebox = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            visualizebox.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
            box.AddVisualShape(visualizebox)
        # Add visualization for the beam elements and nodes
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

    def PrintBodyPositions(self):
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i} box position: ({pos.x}, {pos.y}, {pos.z})")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach Irrlicht to the Chrono system
vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
vis.SetWindowTitle('FEA cables')  # Set the title of the rendering window
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  # Add a camera with specific position
vis.AddTypicalLights()  # Add typical lights for better illumination

# Set solver type and settings
solver = chrono.ChSolverMINRES()  # Choose SparseQR solver
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver.SetVerbose(False)
# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()  # Begin scene rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene rendering
    sys.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds
    model.PrintBodyPositions()  # Print the positions of the end bodies of each chain