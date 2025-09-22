import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Modified Model: Multiple chains of ANCF cable elements with connected bodies
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_boxes = []  # Store end bodies for position printing
        
        for i in range(n_chains):
            # Create truss body (fixed reference frame) for this chain
            start_pos = chrono.ChVector3d(i * 0.6, 0.0, -0.1)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetPos(start_pos)
            system.Add(mtruss)
            
            # Add visualization for truss
            truss_shape = chrono.ChBoxShape()
            truss_shape.GetBoxGeometry().Size = chrono.ChVector3d(0.02, 0.02, 0.02)
            truss_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
            mtruss.AddVisualShape(truss_shape)
            
            # Create beam section
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)
            msection_cable2.SetYoungModulus(0.01e9)
            msection_cable2.SetRayleighDamping(0.0001)
            
            # Create beam builder
            builder = fea.ChBuilderCableANCF()
            end_pos = chrono.ChVector3d(i * 0.6 + 0.5, 0.0, -0.1)
            num_elements = 10 + i  # Increasing elements per chain
            
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_pos,
                end_pos
            )
            
            # Apply force to first node
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Create end body (box)
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)  # Size 10cm, density 1000 kg/m³
            box.SetPos(end_pos)
            system.Add(box)
            self.end_boxes.append(box)  # Store for position printing
            
            # Add visualization for box
            box_shape = chrono.ChBoxShape()
            box_shape.GetBoxGeometry().Size = chrono.ChVector3d(0.05, 0.05, 0.05)
            box_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
            box.AddVisualShape(box_shape)
            
            # Create constraint between last node and box
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_hinge)
    
    def PrintBodyPositions(self):
        """Print positions of all end bodies in the chains"""
        for i, box in enumerate(self.end_boxes):
            pos = box.GetPos()
            print(f"Chain {i}: End body position = ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains (default)
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Visualization settings for FEM mesh
# 1. Visualization of beam moments
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# 2. Visualization of nodes
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
vis.SetWindowTitle('Multiple FEA Chains with End Bodies')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.5, -2))
vis.AddTypicalLights()

# Configure solver
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

# Configure timestepper
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    # Print positions every 10 steps
    if step % 10 == 0:
        model.PrintBodyPositions()
    step += 1