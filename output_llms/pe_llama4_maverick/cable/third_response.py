import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of multiple ANCF cable elements, with one end hinged to ground,
# moving under gravity alone. This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []

        for i in range(n_chains):
            # Create a section, i.e., define thickness and material properties for the cable beam
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  

            # Create a ChBuilderCableANCF helper object
            builder = fea.ChBuilderCableANCF()

            # Use BuildBeam to create a beam structure
            num_elements = 10 + i  # Increase the number of elements for each chain
            start_point = chrono.ChVector3d(i * 0.1, 0, -0.1)  # Relative positioning to avoid overlap
            end_point = chrono.ChVector3d(i * 0.1 + 0.5, 0, -0.1)
            builder.BuildBeam(mesh, msection_cable2, num_elements, start_point, end_point)

            # Apply boundary conditions and loads
            builder.GetLastBeamNodes().front().SetFixed(True)  # Fix the front node
            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.7, 0))  # Apply force to the back node

            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  
            system.Add(mtruss)

            # Create and initialize a hinge constraint
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)  

            # Create a box body at the end of the beam
            end_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
            end_body.SetPos(end_point + chrono.ChVector3d(0, -0.5, 0))  # Position below the beam's end
            system.Add(end_body)

            # Establish a constraint between the beam's endpoint and the box
            link = chrono.ChLinkMateFix()
            link.Initialize(builder.GetLastBeamNodes().back(), end_body)
            system.Add(link)

            self.end_bodies.append(end_body)

            # Visualization for the new beam
            visualizebeamA = chrono.ChVisualShapeFEA(mesh)
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
            visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
            visualizebeamA.SetSmoothFaces(True)  
            visualizebeamA.SetWireframe(False)  
            mesh.AddVisualShapeFEA(visualizebeamA)  

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i+1} end body position: {body.GetPos()}")

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  

# Add visualization for node positions
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
vis.SetWindowTitle('FEA cables')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  
vis.AddTypicalLights()  

# Set solver type and settings
solver = chrono.ChSolverMINRES() 
if solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
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
    model.PrintBodyPositions()  # Print positions of end bodies