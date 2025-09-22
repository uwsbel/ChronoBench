import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of multiple chains of ANCF cable elements, each with increasing number of elements
# Each chain is connected to a truss body with hinge constraints and forces applied
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh):
        self.n_chains = 6  # Default number of chains
        self.chain_elements = [10, 15, 20, 15, 10, 10]  # Number of ANCF elements per chain
        
        # Create visualization objects
        self.visualize_beams = []
        self.visualize_nodes = []
        self.bodies = []
        
        # Create a section for the cable
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)  # 15 mm diameter
        msection_cable.SetYoungModulus(0.01e9)  # 0.01 GPa
        msection_cable.SetRayleighDamping(0.0001)  # No damping
        
        # Create a helper object for building ANCF beams
        builder = fea.ChBuilderCableANCF()
        
        # Initialize the truss body for the first chain
        self.truss_body = chrono.ChBody()
        self.truss_body.SetFixed(True)
        self.bodies.append(self.truss_body)
        
        # Generate each chain
        for i in range(self.n_chains):
            # Create a new truss body for each chain
            chain_truss = chrono.ChBody()
            chain_truss.SetFixed(True)
            self.bodies.append(chain_truss)
            
            # Create the beam for the current chain
            beam = builder.BuildBeam(
                mesh,
                msection_cable,
                self.chain_elements[i],
                chrono.ChVector3d(0, 0, -0.1 - (i * 0.5)),  # Starting position
                chrono.ChVector3d(0.5, 0, -0.1 - (i * 0.5))  # Ending position
            )
            
            # Apply boundary conditions and loads
            # Get the end nodes of the beam
            last_nodes = beam.GetLastBeamNodes()
            # Apply forces to the front node
            last_nodes.front().SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            # Create hinge constraint between beam end and truss
            constraint = fea.ChLinkNodeFrame()
            constraint.Initialize(last_nodes.back(), chain_truss)
            system.Add(constraint)
            
            # Add visualization for the current chain
            visualize_beam = chrono.ChVisualShapeFEA(mesh)
            visualize_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            visualize_beam.SetColorscaleMinMax(-0.4, 0.4)
            visualize_beam.SetSmoothFaces(True)
            visualize_beam.SetWireframe(False)
            mesh.AddVisualShapeFEA(visualize_beam)
            
            # Add node visualization
            visualize_node = chrono.ChVisualShapeFEA(mesh)
            visualize_node.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            visualize_node.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualize_node.SetSymbolsThickness(0.006)
            visualize_node.SetSymbolsScale(0.01)
            visualize_node.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(visualize_node)
            
            # Add the beam and nodes to the system
            system.Add(beam)
            system.Add(last_nodes)
            
        # Add the truss body to the system
        system.Add(self.truss_body)
    
    def PrintBodyPositions(self):
        """Print the positions of the end bodies of each chain during simulation"""
        while vis.Run():
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            # Print positions of end bodies
            for body in self.bodies:
                pos = body.GetPosition()
                print(f"Chain {self.bodies.index(body)+1} end position: {pos}")
            vis.DoStepDynamics(0.01)

# ----------------------------------------------------------------------------
# Main simulation setup
# ----------------------------------------------------------------------------

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

model = Model1(sys, mesh)
sys.Add(mesh)

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# Set solver
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
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)