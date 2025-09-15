import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of ANCF cable elements, each with one end hinged to
# ground and the other end connected to a rigid body (box)
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_boxes = []  # Store end bodies for position printing
        
        spacing = 0.6  # Horizontal spacing between chains
        for i in range(n_chains):
            # Create truss body (fixed reference frame)
            mtruss = chrono.ChBody()
            start_pos = chrono.ChVector3d(i * spacing, 0, -0.1)
            mtruss.SetPos(start_pos)
            mtruss.SetFixed(True)
            system.Add(mtruss)

            # Create beam section
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)
            msection_cable2.SetYoungModulus(0.01e9)
            msection_cable2.SetRayleighDamping(0.0001)
            
            # Create beam builder
            builder = fea.ChBuilderCableANCF()
            end_pos = chrono.ChVector3d(i * spacing + 0.5, 0, -0.1)
            n_elements = 10 + i * 2  # Increasing element count per chain
            builder.BuildBeam(mesh, msection_cable2, n_elements, start_pos, end_pos)
            
            # Fix first node to truss
            constraint_start = fea.ChLinkNodeFrame()
            constraint_start.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_start)
            
            # Create end box body
            box = chrono.ChBody()
            box.SetPos(end_pos)
            box.SetMass(0.1)
            box.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
            system.Add(box)
            self.end_boxes.append(box)  # Store for position tracking
            
            # Connect last node to box
            constraint_end = fea.ChLinkNodeFrame()
            constraint_end.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_end)
            
            # Apply force to the box
            box.SetForce(chrono.ChVector3d(0, -0.7, 0))
    
    def PrintBodyPositions(self):
        for i, box in enumerate(self.end_boxes):
            pos = box.GetPos()
            print(f"Chain {i} end body position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains (default)
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Visualization setup
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

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables - Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Center camera on chain array
center_x = (model.n_chains - 1) * 0.6 / 2.0
vis.AddCamera(chrono.ChVector3d(center_x, 0.6, -1))
vis.AddTypicalLights()

# Solver settings
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:  # Fixed syntax error
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

# Timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()  # Print body positions each step