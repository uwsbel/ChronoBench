import pychrono as chrono
import pychrono.irrlicht as irrlicht

# -------------------------------
# Initialize the Chrono system
# -------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in Y direction

# -------------------------------
# Create the ground body
# -------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 1, 0))  # Positioned above the mass
system.Add(ground)

# Optional: Add a small visual shape to the ground
ground.AddVisualShape(chrono.ChBoxShape(chrono.ChVectorD(0.1, 0.1, 0.1)), chrono.ChFrameD())

# -------------------------------
# Create the mass body
# -------------------------------
mass_body = chrono.ChBody()
mass_body.SetMass(1.0)
mass_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  # Approximate inertia for a small box
mass_body.SetPos(chrono.ChVectorD(0, 0, 0))  # Start at origin

# Add a box shape for the mass
mass_box = chrono.ChBoxShape(chrono.ChVectorD(0.2, 0.2, 0.2))
mass_body.AddVisualShape(mass_box)
system.Add(mass_body)

# -------------------------------
# Create the TSDA (Translational Spring-Damper Actuator)
# -------------------------------
tsda = chrono.ChLinkTSDA()

# Connect the ground and mass at their local origins
tsda.Initialize(
    ground, mass_body,
    chrono.ChVectorD(0, 0, 0),  # Attachment point on ground
    chrono.ChVectorD(0, 0, 0)   # Attachment point on mass
)

# Set spring and damping coefficients (default values)
tsda.SetSpringCoefficient(100)  # N/m
tsda.SetDampingCoefficient(5)   # Ns/m

# Add a visual spring shape (helical)
spring_shape = chrono.ChSpringVisualShape(0.05, 0.01, 10)  # radius, wire thickness, number of coils
tsda.AddVisualShape(spring_shape)

# Add the TSDA to the system
system.Add(tsda)

# -------------------------------
# Set up Irrlicht visualization
# -------------------------------
application = irrlicht.ChIrrApp(
    system,
    'Mass-Spring-Damper System',
    irrlicht.dimension2du(1024, 768)
)

application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))  # Camera position and target

# Add lighting with shadows
application.AddLightWithShadow(
    chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0),
    10, 1, 10, 512,
    irrlicht.SColorf(1, 1, 1)
)

# Finalize visualization assets
application.AssetBindAll()
application.AssetUpdateAll()

# -------------------------------
# Simulation loop
# -------------------------------
application.SetTimestep(0.01)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())