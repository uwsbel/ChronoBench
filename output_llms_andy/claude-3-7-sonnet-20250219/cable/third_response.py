import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of ANCF cable elements connected to bodies,
# with one end hinged to ground, moving under gravity.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []  # List to store the end bodies for position tracking
        
        # Generate multiple chains
        for i in range(n_chains):
            # Create a section for this chain
            msection_cable = fea.ChBeamSectionCable()
            msection_cable.SetDiameter(0.015)  # 15 mm diameter
            msection_cable.SetYoungModulus(0.01e9)  # 0.01 GPa
            msection_cable.SetRayleighDamping(0.0001)
            
            # Number of elements increases with each chain
            n_elements = 10 + i * 2
            
            # Position offset for each chain to avoid overlap
            x_offset = i * 0.2
            z_offset = -0.1 - i * 0.1
            
            # Create a builder for this chain
            builder = fea.ChBuilderCableANCF()
            
            # Build the beam
            builder.BuildBeam(
                mesh,
                msection_cable,
                n_elements,
                chrono.ChVector3d(x_offset, 0, z_offset),  # Start point
                chrono.ChVector3d(x_offset + 0.5, 0, z_offset)  # End point
            )
            
            # Create a truss body (fixed reference frame)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            system.Add(mtruss)
            
            # Create a hinge constraint at one end
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)
            
            # Create a box body at the other end
            mbox = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)  # Size and density
            mbox.SetPos(chrono.ChVector3d(x_offset, 0, z_offset))
            system.Add(mbox)
            self.end_bodies.append(mbox)  # Store for position tracking
            
            # Connect the front node to the box
            constraint_box = fea.ChLinkNodeBody()
            constraint_box.Initialize(builder.GetLastBeamNodes().front(), mbox)
            system.Add(constraint_box)
            
            # Apply a force to the box
            mbox.SetForce(chrono.ChVector3d(0, -0.7 * (i + 1), 0))  # Force increases with each chain

    def PrintBodyPositions(self):
        """Print the positions of all end bodies in the chains"""
        print("Chain end body positions:")
        for i, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model with multiple chains and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Add the mesh to the physical system

# Add visualization for the FEM mesh:
# Visualize beam moments
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node positions:
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables with multiple chains')
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

# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
step_count = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)  # 0.01 second time step
    
    # Print body positions every 20 steps
    step_count += 1
    if step_count % 20 == 0:
        model.PrintBodyPositions()