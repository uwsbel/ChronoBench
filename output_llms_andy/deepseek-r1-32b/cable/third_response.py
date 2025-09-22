import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

# ----------------------------------------------------------------------------
# Model1: Multiple beams composed of ANCF cable elements, with one end hinged to
# ground and connected to bodies, moving under gravity alone.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # To store end bodies for position tracking
        
        # Create multiple chains
        for i in range(self.n_chains):
            # Calculate angular position for each chain
            angle = 2 * math.pi * i / self.n_chains
            radius = 0.3
            
            # Define start and end points for each chain
            start_point = chrono.ChVector3d(
                radius * math.cos(angle),
                radius * math.sin(angle),
                -0.1
            )
            end_point = chrono.ChVector3d(
                radius * math.cos(angle) + 0.5,
                radius * math.sin(angle),
                -0.1
            )
            
            # Create a section for the cable
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)
            
            # Create a builder for the cable
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                10 + i,  # Increase number of elements per chain
                start_point,
                end_point
            )
            
            # Apply force to the front node
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Create truss and box bodies
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            end_body = chrono.ChBody()
            end_body.SetBodyType(chrono.ChBodyType.BODY_TYPE_RIGID)
            end_body.SetMass(1.0)
            end_body.SetPos(end_point)
            
            # Add visualization for end body
            end_body_shape = chrono.ChVisualShape()
            end_body_shape.SetGeometry(chrono.ChBoxGeometry(0.05, 0.05, 0.05))
            end_body_shape.SetMaterialSurface(chrono.ChVisualMaterial())
            end_body_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
            end_body.AddVisualShape(end_body_shape)
            
            # Create constraints
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_hinge)
            
            constraint_fixed = chrono.ChLinkLockFrameNode()
            constraint_fixed.Initialize(mtruss, end_body)
            self.system.Add(constraint_fixed)
            
            # Add to lists
            self.end_bodies.append(end_body)
            
            # Add visualization for the beam
            visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
            visualizebeamA.SetSmoothFaces(True)
            visualizebeamA.SetWireframe(False)
            self.mesh.AddVisualShapeFEA(visualizebeamA)
            
            visualizebeamB = chrono.ChVisualShapeFEA(self.mesh)
            visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualizebeamB.SetSymbolsThickness(0.006)
            visualizebeamB.SetSymbolsScale(0.01)
            visualizebeamB.SetZbufferHide(False)
            self.mesh.AddVisualShapeFEA(visualizebeamB)
            
            # Add end body to system
            self.system.Add(end_body)
            
    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: ({pos.x}, {pos.y}, {pos.z})")

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
vis.SetWindowTitle('FEA Multiple Chains')
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

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()