import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# Initialize system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable gravity

# Beam parameters
L = 1.0              # Length
radius = 0.02        # Radius
num_elements = 20    # Mesh resolution
num_nodes = num_elements + 1

# Create mesh
mesh = fea.ChMesh()
nodes = []

# Create beam nodes with small lateral imperfection
for i in range(num_nodes):
    x = i * L / num_elements
    y = 0.0
    z = 0.0
    
    # Add small random lateral perturbation to trigger buckling
    if i != 0:
        y += 0.001 * (chrono.ChRandom() - 0.5)
    
    pos = chrono.ChVectorD(x, y, z)
    node = fea.ChNodeFEAxyzrot(pos)
    if i == 0:
        node.SetFixed(True)  # Fix first node
    mesh.AddNode(node)
    nodes.append(node)

# Material properties
E = 2e5e6    # Young's modulus (Pa)
nu = 0.3     # Poisson ratio
area = chrono.CH_C_PI * radius**2
Iy = chrono.CH_C_PI * radius**4 / 4
Iz = Iy
G = E / (2 * (1 + nu))
J = Iy + Iz

# Create beam elements
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    
    # Set element properties
    element.SetSectionArea(area)
    element.SetSectionIy(Iy)
    element.SetSectionIz(Iz)
    element.SetSectionYoungModulus(E)
    element.SetSectionShearModulus(G)
    element.SetSectionRadius(radius)
    
    mesh.AddElement(element)

# Add mesh to system
system.Add(mesh)

# Create prismatic joint with motor for compression
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Connect last node to ground via prismatic joint
node_end = nodes[-1]
joint = chrono.ChLinkLockPrismatic()
joint.Initialize(ground, node_end, 
                chrono.ChCoordsysD(chrono.ChVectorD(L, 0, 0), 
                                   chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)

# Define motor function (linear displacement over time)
motor = chrono.ChFunction_Ramp()
motor.SetCoeff(0.0, -0.02)  # 0.02 m/s compression rate
joint.SetMotion_x(motor)

# Solver and timestepper setup
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-8)
system.SetSolver(solver)

timestepper = chrono.ChTimestepperHHT(system)
timestepper.SetStepControl(True)
timestepper.SetAlpha(-0.1)
timestepper.SetMaxiters(20)
timestepper.SetAbsTolerances(1e-8)
timestepper.SetMode(chrono.ChTimestepperHHT.POSITION)
system.SetTimestepper(timestepper)

# Visualization setup
application = irr.ChIrrApp(system, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5))
application.AddTypicalLights()

# Add beam visualization
vis_beam = fea.ChVisualizationFEAmesh(mesh)
vis_beam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_beam.SetWireframe(True)
vis_beam.SetDrawInUndeformedReference(False)
mesh.AddAsset(vis_beam)

# Finalize visualization
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    application.DoStep()