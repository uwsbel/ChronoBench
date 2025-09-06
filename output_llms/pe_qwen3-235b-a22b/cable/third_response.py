import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        # Create a contact material for the boxes
        self.material = chrono.ChContactMaterialNSC()
        self.material.SetFriction(0.3)
        self.boxes = []  # Store box bodies for position tracking

        for i in range(n_chains):
            # Calculate chain-specific parameters
            chain_offset = i * 0.6  # Offset along X-axis to avoid overlap
            start_x = chain_offset
            end_x = chain_offset + 0.5
            n_elements = 10 + i  # Increasing number of elements per chain

            # Define beam start and end points
            start_point = chrono.ChVector3d(start_x, 0, -0.1)
            end_point = chrono.ChVector3d(end_x, 0, -0.1)

            # Create cable section
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)
            msection_cable2.SetYoungModulus(0.01e9)
            msection_cable2.SetRayleighDamping(0.0001)

            # Create beam using ChBuilderCableANCF
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh,
                msection_cable2,
                n_elements,
                start_point,
                end_point
            )

            # Get beam nodes
            front_node = builder.GetLastBeamNodes().front()
            back_node = builder.GetLastBeamNodes().back()

            # Apply force to front node
            front_node.SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Create truss body (fixed reference)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(start_point)
            system.Add(mtruss)

            # Hinge constraint between beam's back node and truss
            constraint_hinge_back = fea.ChLinkNodeFrame()
            constraint_hinge_back.Initialize(back_node, mtruss)
            system.Add(constraint_hinge_back)

            # Create box body connected to beam's front node
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True, self.material)
            box.SetPos(end_point)
            box_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
            box.AddVisualShape(box_shape)
            system.Add(box)
            self.boxes.append(box)  # Store for position tracking

            # Constraint between beam's front node and box
            constraint_hinge_front = fea.ChLinkNodeFrame()
            constraint_hinge_front.Initialize(front_node, box)
            system.Add(constraint_hinge_front)

    def PrintBodyPositions(self):
        """Print positions of all end boxes in each chain"""
        for idx, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {idx+1} Box Position: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Visualization for beam moments
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamA)

# Visualization for node positions
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(visualizebeamB)

# Irrlicht visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Cables with Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# Solver settings
solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)

# Timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()  # Print positions at each step