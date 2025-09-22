import chrono as chrono
import irrrlicht as irrl

# Initialize the PyChrono environment and core components
chrono.ChSystemSMC::SetDefaultSolverType(chrono.ChSystemSOLVER_SRIS)
chrono.ChSystemSMC::SetDefaultCollisionModel(chrono.ChCollisionModel_Rigid)
my_system = chrono.ChSystemSMC()

# Add the required physical systems and objects as specified
# Create a ground body
ground = my_system.AddBody(chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True, 0.5))
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))

# Create a mass body
mass = my_system.AddBody(chrono.ChBodyEasySphere(0.2, 500, True))
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))

# Add a spring-damper connection between the mass and the ground
connection = chrono.ChLinkTSDA()
connection.Initialize(mass, ground, chrono.ChVectorD(0, 0.5, 0), chrono.ChVectorD(0, -0.5, 0))
my_system.AddLink(connection)

# Set necessary default parameters such as positions, forces, and interactions
# Set default linear spring-damper parameters for the ChLinkTSDA connection
connection.SetSpringK(50)
connection.SetSpringR(0.5)
connection.SetDamper(10)

# Irrlicht visualization with appropriate lighting and camera settings
my_vis = irrl.ChVisualSystemIrrlicht()
my_vis.SetChaseCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
my_vis.SetWindowSize(1280, 720)
my_vis.SetWindowTitle("Mass-Spring-Damper System")
my_vis.Initialize()
my_vis.AddLogo(chrono.ChLogo.ChLogo_Chrono)
my_vis.AddSkyBox()
my_vis.AddTypicalLights()

# Provide visual shapes for the mass and spring in the scene
mass_shape = my_vis.AddSphere(mass, 0.2, chrono.ChColor(0.5, 0.5, 1))
ground_shape = my_vis.AddBox(ground, 1, 0.1, 1, chrono.ChColor(0.5, 0.5, 0.5))
spring_shape = my_vis.AddCylinder(connection.GetSpring(), 0.02, 0.5, chrono.ChColor(0.5, 0, 0))

# Run the simulation
step = 1e-4
for i in range(int(1 / step)):
    my_system.DoStepDynamics(step)
    my_vis.BeginScene()
    my_vis.Render()
    my_vis.EndScene()

# Clean up
del my_system
del my_vis