import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

# ----------------------------------------------------------------------------
# Model1: Multiple beams composed of ANCF cable elements, with one end hinged to
# ground and the other end connected to a box body, moving under gravity alone.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # To store end bodies for position tracking
        
        # Create multiple chains
        for i in range(n_chains):
            # Create a section for the cable
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)
            
            # Create a ChBuilderCableANCF helper object
            builder = fea.ChBuilderCableANCF()
            
            # Calculate positions for each chain to spread them out
            angle = (i * 2 * math.pi) / n_chains
            radius = 0.3  # Radius from center
            start_x = radius * math.cos(angle)
            start_y = radius * math.sin(angle)
            
            # Create beam with increasing number of elements per chain
            num_elements = 10 + i  # Start with 10 elements and increase
            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                chrono.ChVector3d(start_x, start_y, -0.1),
                chrono.ChVector3d(start_x + 0.5, start_y, -0.1)
            )
            
            # Apply forces to the front node
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Create truss body (fixed reference frame)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            # Create hinge constraint
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            # Create box body at the end of the beam
            box = chrono.ChBody()
            box.SetBodyFixed(False)
            box.SetMass(1.0)
            box.SetPos(builder.GetLastBeamNodes().back().GetPos())
            box_shape = chrono.ChBoxShape()
            box_shape.SetDimensions(chrono.ChVector3d(0.05, 0.05, 0.05))
            box.AddVisualShape(box_shape)
            system.Add(box)
            
            # Add constraint between beam end and box
            constraint_lock = fea.ChLinkLockRevolute()
            constraint_lock.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_lock)
            
            # Add visualization for the beam
            visualize_beam = chrono.ChVisualShapeFEA(mesh)
            visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
            visualize_beam.SetSmoothFaces(True)
            visualize_beam.SetWireframe(False)
            mesh.AddVisualShapeFEA(visualize_beam)
            
            # Add visualization for nodes
            visualize_nodes = chrono.ChVisualShapeFEA(mesh)
            visualize_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualize_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualize_nodes.SetSymbolsThickness(0.006)
            visualize_nodes.SetSymbolsScale(0.01)
            visualize_nodes.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(visualize_nodes)
            
            # Store the end box body for position tracking
            self.end_bodies.append(box)
            
    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i+1} end position: {body.GetPos()}")

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with multiple chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Cables with Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# Set solver and timestepper
solver = chrono.ChSolverMINRES()
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()