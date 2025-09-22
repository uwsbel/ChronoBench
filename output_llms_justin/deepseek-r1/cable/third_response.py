import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of ANCF cable elements with connected boxes
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.boxes = []  # Store end bodies for position printing
        
        # Create a reusable cable section
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)
        
        # Create multiple chains
        for i in range(n_chains):
            # Create truss at distinct position
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            truss_pos = chrono.ChVector3d(i * 0.6, 0.2, -0.1)
            mtruss.SetPos(truss_pos)
            system.Add(mtruss)
            
            # Create builder for this chain
            builder = fea.ChBuilderCableANCF()
            n_elements = 10 + i * 2  # Elements increase per chain
            
            # Calculate beam endpoints
            start_point = truss_pos
            end_point = truss_pos + chrono.ChVector3d(0.5, 0, 0)
            
            # Build the beam
            builder.BuildBeam(
                mesh,
                msection_cable2,
                n_elements,
                start_point,
                end_point
            )
            
            # Fix first node to truss
            constraint_hinge_truss = fea.ChLinkNodeFrame()
            constraint_hinge_truss.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge_truss)
            
            # Create box at endpoint
            end_node = builder.GetLastBeamNodes().back()
            box = chrono.ChBody()
            box.SetPos(end_node.GetPos())
            box.SetMass(0.1)
            box.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            system.Add(box)
            
            # Add box visualization
            box_shape = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            box_shape.SetColor(chrono.ChColor(0.6, 0.3, 0.0))
            box.AddVisualShape(box_shape)
            
            # Connect end node to box
            constraint_hinge_box = fea.ChLinkNodeFrame()
            constraint_hinge_box.Initialize(end_node, box)
            system.Add(constraint_hinge_box)
            
            self.boxes.append(box)  # Store for position printing
    
    def PrintBodyPositions(self):
        """Print positions of end bodies for each chain"""
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i} end body position: x={pos.x:.4f} y={pos.y:.4f} z={pos.z:.4f}")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains (default)
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  # Add mesh to system after model creation

# Visualization for FEM mesh (moments)
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Visualization for nodes
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 0.6, -1.0))
vis.AddTypicalLights()

# Configure solver
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

# Set timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
step_count = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    # Print positions every 10 steps
    step_count += 1
    if step_count % 10 == 0:
        model.PrintBodyPositions()