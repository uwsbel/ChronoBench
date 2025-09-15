import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # To store end bodies for position tracking
        
        # Create sections
        self.msection_cable = fea.ChBeamSectionCable()
        self.msection_cable.SetDiameter(0.015)
        self.msection_cable.SetYoungModulus(0.01e9)
        self.msection_cable.SetRayleighDamping(0.0001)
        
        # Create beams for each chain
        self.create_chains()

    def create_chains(self):
        angle_increment = 2 * math.pi / self.n_chains
        for i in range(self.n_chains):
            # Calculate position for each chain
            angle = i * angle_increment
            start_point = chrono.ChVector3d(0.2 * math.cos(angle), 0.2 * math.sin(angle), -0.1)
            end_point = start_point + chrono.ChVector3d(0.5 * math.cos(angle), 0.5 * math.sin(angle), 0)
            
            # Create builder and beam
            builder = fea.ChBuilderCableANCF()
            num_elements = 10 + 2*i  # Increase number of elements per chain
            
            builder.BuildBeam(
                self.mesh,
                self.msection_cable,
                num_elements,
                start_point,
                end_point
            )
            
            # Create truss body
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)
            
            # Create hinge constraint
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)
            
            # Create end body (box)
            end_body = chrono.ChBody()
            end_body.SetBodyFixed(False)
            end_body.SetMass(0.1)
            end_body.SetPos(end_point)
            self.system.Add(end_body)
            
            # Add visualization for end body
            vis_box = chrono.ChVisualShape()
            vis_box.SetPos(end_point)
            vis_box.SetSize(chrono.ChVector3d(0.05, 0.05, 0.05))
            vis_box.SetColor(chrono.ChColor(0, 1, 0))
            end_body.AddVisualShape(vis_box)
            
            # Constraint between beam endpoint and box
            constraint_box = fea.ChLinkNodeBody()
            constraint_box.Initialize(builder.GetLastBeamNodes().back(), end_body)
            self.system.Add(constraint_box)
            
            self.end_bodies.append(end_body)
            
            # Add visualization for beam
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

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i+1} end position: ({pos.x}, {pos.y}, {pos.z})")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create model with 6 chains
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables with multiple chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# Solver and timestepper setup
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
step_counter = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    
    # Print positions every 100 steps
    step_counter += 1
    if step_counter % 100 == 0:
        print("\nCurrent step:", step_counter)
        model.PrintBodyPositions()