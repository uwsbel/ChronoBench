import pychrono.core as chrono
import pychrono.irrlicht as irr

# Initialize the Chrono system
system = chrono.ChSystemNSC()

# Create the Irrlicht application for visualization
application = irr.ChIrrApp(system, "Mass-Spring-Damper System", irr.dimension2du(800, 600))
application.AddTypicalLights()
application.SetCameraPosition(chrono.ChVectorD(0, 1, 3))
application.SetChaseCamera(chrono.ChVectorD(0, 0, 0))
application.EnableStatistics(False)
application.DrawAllPreseed = True

# Create the ground body (fixed)
ground = chrono.ChBodyEasyBox(1.0, 0.1, 1.0, 1000, True, True)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
system.Add(ground)

# Create the mass body
mass_radius = 0.1
mass_height = 0.2
mass_body = chrono.ChBodyEasyCylinder(mass_radius, mass_height, 1000, True, True)
mass_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(mass_body)

# Add visual shape for the mass (optional, already handled by ChBodyEasyCylinder)
# (Already visualized by ChBodyEasyCylinder)

# Define spring-damper parameters
spring_stiffness = 1000  # N/m
damper_coefficient = 10  # N·s/m
rest_length = 0.5       # meters

# Create the TSDA (spring-damper) link between ground and mass
spring_link = chrono.ChLinkTSDA()
spring_link.SetId(0)
spring_link.Initialize(
    ground,          # ground body
    mass_body,       # mass body
    False,           # do not auto-assign
    chrono.ChVectorD(0, 0, 0),  # point on ground (origin)
    chrono.ChVectorD(0, 0.5, 0) # point on mass (top center)
)

# Set spring-damper parameters
spring_link.SetSpringCoefficient(spring_stiffness)
spring_link.SetDampingCoefficient(damper_coefficient)
spring_link.SetRestLength(rest_length)

system.Add(spring_link)

# Visualization for the spring (optional)
# The link has a visual representation automatically, but we can customize if needed

# Start the simulation
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
while application.GetDevice().run():
    system.DoStepDynamics(0.01)
    application.AssetUpdateAll()
    application.BeginScene()
    application.DrawAll()
    application.EndScene()