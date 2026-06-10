import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Several chains of ANCF cable elements, each with one end hinged to
# ground, and the other end connected to a falling box body.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        # Store the end bodies of each chain so we can print their positions later
        self.end_bodies = []

        # Loop to build the requested number of chains
        for i in range(n_chains):
            # Create a section, i.e. define thickness and material properties for the cable beam
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)             # Cable diameter = 15 mm
            msection_cable2.SetYoungModulus(0.01e9)        # Young's modulus = 0.01 GPa
            msection_cable2.SetRayleighDamping(0.0001)     # Rayleigh damping

            # Create a ChBuilderCableANCF helper object
            builder = fea.ChBuilderCableANCF()

            # Number of elements increases with each chain
            n_elements = 1 + i

            # Offset the chains along Z so they do not overlap
            z_offset = -0.1 * i

            # Build the beam: start point A and end point B
            builder.BuildBeam(
                mesh,                                       # mesh to receive nodes/elements
                msection_cable2,                            # beam section
                n_elements,                                 # number of ANCF elements
                chrono.ChVector3d(0, 0, z_offset),          # Starting point 'A'
                chrono.ChVector3d(0.1 + 0.1 * i, 0, z_offset)  # Ending point 'B'
            )

            # Apply a downward force to the last node of the beam
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Create a fixed truss body for this chain
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)

            # Hinge the FIRST node of the beam to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            # Create a box body connected to the end of the beam
            mbox = chrono.ChBodyEasyBox(0.04, 0.1, 0.04, 1000)
            end_node = builder.GetLastBeamNodes().back()
            mbox.SetPos(end_node.GetPos() + chrono.ChVector3d(0.02, 0, 0))
            system.Add(mbox)
            self.end_bodies.append(mbox)

            # Constrain the end node position to the box
            constraint_pos = fea.ChLinkNodeFrame()
            constraint_pos.Initialize(end_node, mbox)
            system.Add(constraint_pos)

            # Constrain the end node direction (slope) to the box
            constraint_dir = fea.ChLinkNodeSlopeFrame()
            constraint_dir.Initialize(end_node, mbox)
            constraint_dir.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir)

    def PrintBodyPositions(self):
        # Print the positions of the end bodies of each chain
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print("Chain {0} end body position: x={1:.4f}, y={2:.4f}, z={3:.4f}".format(
                i, pos.x, pos.y, pos.z))


# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model (multiple chains) and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Add visualization for the FEM mesh (forces/moments in the beam elements):
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Color scale for moment visualization
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node positions:
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Nodes as dots
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
solver = chrono.ChSolverMINRES()  # Choose MINRES solver
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)  # IMPORTANT for EULER_IMPLICIT_LINEARIZED convergence
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
    # Print the positions of the end bodies of each chain at each step
    model.PrintBodyPositions()