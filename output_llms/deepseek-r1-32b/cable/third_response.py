import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # To store the end bodies of each chain

        # Create multiple chains
        for i in range(self.n_chains):
            # Calculate position for each chain to spread them around a circle
            angle = i * (2 * math.pi / self.n_chains)
            
            # Create a section for the cable
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)
            msection_cable.SetYoungModulus(0.01e9)
            msection_cable.SetRayleighDamping(0.0001)
            
            # Create a ChBuilderCableANCF helper
            builder = fea.ChBuilderCableANCF()
            
            # Build the beam with increasing number of elements per chain
            num_elements = 10 + i  # Increase number of elements per chain
            
            # Define start and end points with offset to avoid overlap
            radius = 0.2  # Radius for spreading chains
            start_point = chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), -0.1)
            end_point = chrono.ChVector3d((radius + 0.5) * math.cos(angle), (radius + 0.5) * math.sin(angle), -0.1)
            
            builder.BuildBeam(
                self.mesh,
                msection_cable,
                num_elements,
                start_point,
                end_point
            )
            
            # Apply force to the front node
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Create truss body for each chain
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            
            # Create hinge constraint between beam end and truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)
            
            # Create end body (box) for each chain
            end_body = chrono.ChBody()
            end_body.SetMass(0.1)  # Add some mass to the end body
            end_body.SetBodyFixed(False)
            
            # Add visualization to end body
            end_body_vis = chrono.ChVisualShape()
            end_body_vis.SetGeometry(chrono.ChBoxGeometry(0.05, 0.05, 0.05))
            end_body_vis.SetMaterialSurface(chrono.ChMaterialSurface())
            end_body.AddVisualShape(end_body_vis)
            
            # Create constraint between beam end and end body
            constraint_beam_body = fea.ChLinkNodeBody()
            constraint_beam_body.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_beam_body)
            
            # Add end body to the system
            self.system.Add(end_body)
            self.end_bodies.append(end_body)
            
            # Add truss to the system
            self.system.Add(mtruss)

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end body position: ({pos.x}, {pos.y}, {pos.z})")

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with multiple chains
model = Model1(sys, mesh)
sys.Add(mesh)

# Add visualization for the FEM mesh
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