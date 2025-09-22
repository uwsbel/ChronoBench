import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of beams composed of ANCF cable elements, each with one end hinged to ground
# and connected to a box body at the other end, demonstrating multiple connected chains.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # Store end bodies for printing positions later

        # Loop over each chain to create multiple beam structures
        for i in range(self.n_chains):
            # Create a fixed truss body as reference for each chain
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)

            # Define a different start position for each chain to avoid overlap
            start_x = i * 0.2  # spacing between chains
            start_point = chrono.ChVector3d(start_x, 0, 0)
            end_point = chrono.ChVector3d(start_x, 0, -0.5)

            # Create a section for the cable
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)  # 15 mm diameter
            msection_cable.SetYoungModulus(0.01e9)  # 0.01 GPa
            msection_cable.SetRayleighDamping(0.0001)

            # Build the beam with increasing number of elements for variety
            num_elements = 10 + i  # e.g., 10, 11, 12, ...
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )

            # Apply force at the front node of each beam (at the end point)
            front_node = builder.GetLastBeamNodes().front()
            front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Create a box body at the end of the beam to connect to
            end_body = chrono.ChBody()
            end_body.SetMass(0.1)  # small mass
            end_body.SetPos(end_point)
            self.system.Add(end_body)
            self.end_bodies.append(end_body)

            # Create a hinge constraint to connect beam's last node to the box body
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_hinge)

            # Optionally, connect the start of the beam to the truss (fixed body)
            # (If desired, uncomment below)
            # start_node = builder.GetFirstBeamNode()
            # constraint_start = fea.ChLinkNodeFrame()
            # constraint_start.Initialize(start_node, mtruss)
            # self.system.Add(constraint_start)

    def PrintBodyPositions(self):
        # Print positions of each end body
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i} end body position: x={pos.x}, y={pos.y}, z={pos.z}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with multiple chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Remember to add the mesh to the system

# Add visualization for the FEM mesh:
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node positions:
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

# Set the timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop with print statement for body positions
while vis.Run():
    # Print positions of end bodies at each step
    model.PrintBodyPositions()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)