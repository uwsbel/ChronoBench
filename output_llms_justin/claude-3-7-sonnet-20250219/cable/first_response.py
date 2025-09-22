import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Global simulation parameters
beam_length = 2.0       # meters
beam_density = 1000     # kg/m^3
beam_radius = 0.01      # meters
num_elements = 10       # number of ANCF elements
time_step = 1e-3        # seconds
simulation_time = 10    # seconds

# Initialize the ChSystem
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create material for the beam
beam_material = chrono.ChMaterialSurface()
beam_material.SetFriction(0.4)

# Calculate cross-sectional properties
beam_area = np.pi * beam_radius**2
beam_Iyy = np.pi * beam_radius**4 / 4
beam_Izz = beam_Iyy
E = 2e6  # Young's modulus (Pa)
G = 0.8e6  # Shear modulus (Pa)

# Create the beam elements
beam_nodes = []
beam_elements = []
element_length = beam_length / num_elements

# Create the ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Initialize the first node at origin
node0 = chrono.ChNodeFEAxyzD(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
node0.SetMass(0)  # Fixed node
beam_nodes.append(node0)
system.Add(node0)

# Add a constraint to fix the first node to the ground (hinge)
constraint = chrono.ChLinkPointFrame()
constraint.Initialize(node0, ground)
system.Add(constraint)

# Create all other nodes and elements
for i in range(1, num_elements + 1):
    # Create a node
    node_pos = chrono.ChVectorD(i * element_length, 0, 0)
    node_dir = chrono.ChVectorD(1, 0, 0)  # Initial direction (tangent)
    node = chrono.ChNodeFEAxyzD(node_pos, node_dir)
    
    # Set node properties
    node.SetMass(beam_density * beam_area * element_length / 2)  # Lumped mass at node
    beam_nodes.append(node)
    system.Add(node)
    
    # Create an ANCF cable element connecting to the previous node
    element = chrono.ChElementCableANCF()
    element.SetNodes(beam_nodes[i-1], beam_nodes[i])
    element.SetDensity(beam_density)
    element.SetSection(beam_area)
    element.SetYoungModulus(E)
    element.SetShearModulus(G)
    element.SetIyy(Iyy=beam_Iyy)
    element.SetIzz(Izz=beam_Izz)
    
    beam_elements.append(element)
    system.Add(element)

# Set up the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "ANCF Cable Beam Demo", 
                                chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(1, 0.5, 2), chronoirr.vector3df(beam_length/2, 0, 0))
application.AddTypicalLights()
application.AddLightWithShadow(chronoirr.vector3df(1.5, 5.5, -2.5), 
                             chronoirr.vector3df(0, 0, 0), 
                             3, 2.2, 7.2, 40, 512)

# Add visualization assets for the beam
for element in beam_elements:
    vis_shape = chrono.ChVisualShapeFEA(element.GetMesh())
    vis_shape.SetFEMdataType(chronoirr.ChVisualShapeFEA.DataType_BEAM)
    vis_shape.SetColorscale(0, 5)
    vis_shape.SetSmoothFaces(True)
    vis_shape.SetWireframe(False)
    element.AddVisualShapeFEA(vis_shape)

# Add visualization for the nodes
for node in beam_nodes:
    vis_node = chrono.ChVisualShapePointPoint(0.01, chrono.ChColor(0.2, 0.3, 0.4))
    node.AddVisualShape(vis_node)

# Add visualization for the ground
ground_box = chrono.ChBoxShape(0.1, 0.1, 0.1)
ground.AddVisualShape(ground_box, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

# Setup solver
solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)

# Setup integrator
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
integrator = chrono.ChTimestepperHHT(system)
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(5)
integrator.SetAbsTolerances(1e-05)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
integrator.SetScaling(True)
integrator.SetVerbose(False)

# Initialize the rendering system
application.AssetBindAll()
application.AssetUpdateAll()

# Set the time step and run the simulation
system.SetChTime(0)
application.SetTimestep(time_step)

# Run the simulation loop
while application.GetDevice().run() and system.GetChTime() < simulation_time:
    # Render scene
    application.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.2))
    application.DrawAll()
    
    # Print some info about node positions
    if int(system.GetChTime() * 100) % 50 == 0:  # Print every 0.5 seconds
        print(f"Time: {system.GetChTime():.2f}")
        for i, node in enumerate(beam_nodes):
            pos = node.GetPos()
            print(f"  Node {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    
    # Update the simulation
    application.DoStep()
    application.EndScene()

print("Simulation completed")