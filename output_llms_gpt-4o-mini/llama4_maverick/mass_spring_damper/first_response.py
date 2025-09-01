import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# 2. Create the physical system and objects.
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create a body for the mass
mass_body = chrono.ChBody()
mass_body.SetMass(1.0)  # Mass in kg
mass_body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))  # Inertia
mass_body.SetPos(chrono.ChVectorD(0, 1, 0))  # Initial position
mass_body.SetBodyFixed(False)

# Create a fixed body (ground)
ground_body = chrono.ChBody()
ground_body.SetBodyFixed(True)

# 3. Add the required physical objects and set default parameters.
# Create a ChLinkTSDA (spring-damper-actuator) between the mass and ground
link_tsda = chrono.ChLinkTSDA()
link_tsda.Initialize(mass_body, ground_body, True, chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0))
link_tsda.SetSpringCoefficient(100.0)  # Spring coefficient (N/m)
link_tsda.SetDampingCoefficient(5.0)  # Damping coefficient (N.s/m)

# Add bodies and link to the system
sys.Add(mass_body)
sys.Add(ground_body)
sys.Add(link_tsda)

# Create visualization shapes for the mass and spring
mass_sphere = chrono.ChSphereShape()
mass_sphere.GetSphereGeometry().rad = 0.2
mass_body.AddVisualShape(mass_sphere)

spring_visual = chrono.ChSpringShape(0.1, 80, 15)
spring_visual.SetColor(chrono.ChColor(1, 0, 0))  # Red color for spring visualization
link_tsda.AddVisualShape(spring_visual)

# Irrlicht visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 2), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChVectorD(2, 2, 2), 10, 2, 10, 120)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 1, 1)
    vis.EndScene()
    sys.DoStepDynamics(0.01)  # Step the simulation