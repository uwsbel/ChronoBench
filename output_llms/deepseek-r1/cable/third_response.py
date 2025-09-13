import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Modified Model1: Multiple chains of beam elements with connected bodies
# Each chain has a fixed truss, ANCF cable elements, and a box body at the end
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.boxes = []  # Store end bodies for position printing
        
        for i in range(n_chains):
            # Create a fixed truss for this chain
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(chrono.ChVector3d(i * 0.6, 0, -0.1))
            system.Add(mtruss)
            
            # Create beam section
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)
            msection_cable2.SetYoungModulus(0.01e9)
            msection_cable2.SetRayleighDamping(0.0001)
            
            # Create beam builder and build beam
            builder = fea.ChBuilderCableANCF()
            num_elements = 10 + i  # Increasing number of elements per chain
            start_point = chrono.ChVector3d(i * 0.6, 0, -0.1)
            end_point = chrono.ChVector3d(i * 0.6 + 0.5, 0, -0.1)
            
            builder.BuildBeam(
                mesh, 
                msection_cable2, 
                num_elements, 
                start_point, 
                end_point
            )
            
            # Fix first node to truss
            constraint_hinge_start = fea.ChLinkNodeFrame()
            constraint_hinge_start.Initialize(
                builder.GetLastBeamNodes().front(), 
                mtruss
            )
            system.Add(constraint_hinge_start)
            
            # Create box body at endpoint
            box_body = chrono.ChBody()
            box_body.SetPos(end_point)
            box_body.SetMass(0.1)
            box_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            system.Add(box_body)
            
            # Add box visualization
            box_shape = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
            box_body.AddVisualShape(box_shape)
            
            # Connect last node to box
            constraint_hinge_end = fea.ChLinkNodeFrame()
            constraint_hinge_end.Initialize(
                builder.GetLastBeamNodes().back(), 
                box_body
            )
            system.Add(constraint_hinge_end)
            
            # Apply force to box
            box_body.SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Store box for position printing
            self.boxes.append(box_body)
    
    def PrintBodyPositions(self):
        """Print positions of end bodies for all chains"""
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i}: Box position = ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains (default)
model = Model1(sys, mesh)
sys.Add(mesh)

# Visualization for FEM mesh (moment)
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

# Setup Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables with multiple chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 0.6, -1))  # Centered camera view
vis.AddTypicalLights()

# Setup solver
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

# Setup timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop with position printing
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